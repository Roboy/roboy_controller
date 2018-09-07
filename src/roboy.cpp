#include "roboy_controller/roboy.hpp"


Roboy::Roboy() {
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy", ros::init_options::NoRosout);
    }

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
    spinner->start();

    string sdf;
    nh->getParam("robot_description_sdf", sdf);

    ROS_DEBUG("found robot_description_sdf: \n\n%s\n\n", sdf.c_str());

    vector<string> endeffectors;
    nh->getParam("end_effectors", endeffectors);

    vector<MuscInfo> muscInfo;
    if (!CASPR::parseSDFusion(sdf, muscInfo))
        ROS_FATAL("error parsing sdf");

    stringstream str;
    str << "initialized CASPR controllers for endeffectors:" << endl;
    for (auto endeffector:endeffectors) {
        vector<string> chain;
        nh->getParam(endeffector + "/kinematic_chain", chain);
        caspr.push_back(boost::shared_ptr<CASPR>(new CASPR(chain.front(), chain.back(), muscInfo)));
        caspr.back()->simulate = true;
        target_pos[endeffector] = &caspr.back()->target_pos;
        target_vel[endeffector] = &caspr.back()->target_vel;
        targetPosition[endeffector].setZero();
        targetRotation[endeffector] = Quaterniond(0,0,0,1);
        ik_success[endeffector] = false;
        reached_target[endeffector] = false;
        new_target[endeffector] = false;
        int control_type;
        nh->getParam(endeffector + "/control", control_type);
        control[endeffector] = control_type;
        str << endeffector << endl;
    }
    ROS_INFO_STREAM(str.str());
}

Roboy::~Roboy() {
}

void Roboy::read(double period) {
    ROS_DEBUG("read");
    for (auto casp:caspr) {
        casp->update(period);
    }
}

void Roboy::write() {
    ROS_DEBUG("write");
    for (auto casp:caspr) {
        nh->getParam(casp->end_effektor_name + "/Kp", Kp[casp->end_effektor_name]);
        nh->getParam(casp->end_effektor_name + "/Kd", Kd[casp->end_effektor_name]);
        nh->getParam(casp->end_effektor_name + "/target_pos", casp->target_pos);
        nh->getParam(casp->end_effektor_name + "/target_vel", casp->target_vel);
        casp->updateController(Kp[casp->end_effektor_name], Kd[casp->end_effektor_name]);
    }
}

void Roboy::main_loop() {
    prev_time = ros::Time::now();

    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        read(period.toSec());
        write();

        ROS_INFO_THROTTLE(5,"%s", state_strings[currentState].c_str());

        switch (currentState) {
            case CheckTargetFrames:
                for (auto casp:caspr) {
                    nh->getParam(casp->end_effektor_name+"/target_frame", target_frame[casp->end_effektor_name]);
                }
                break;
            case GetTargetPositionsAndRotations:
                for(auto casp:caspr) {
                    if(control[casp->end_effektor_name]==0) {
                        new_target[casp->end_effektor_name] = false;
                        goto_start[casp->end_effektor_name] = ros::Time::now();
                        continue;
                    }
                    tf::StampedTransform trans;
                    try {
                        if(listener.waitForTransform("world", target_frame[casp->end_effektor_name].c_str(), ros::Time(0), ros::Duration(0.01)))
                            listener.lookupTransform("world", target_frame[casp->end_effektor_name].c_str(), ros::Time(0), trans);
                        else
                            continue;
                    }
                    catch (tf::LookupException ex) {
                        ROS_WARN_THROTTLE(1, "%s", ex.what());
                        new_target[casp->end_effektor_name] = false;
                        continue;
                    }
                    // if the target changed
                    if(sqrt(pow(targetPosition[casp->end_effektor_name][0]-trans.getOrigin().x(),2.0)+
                         pow(targetPosition[casp->end_effektor_name][1]-trans.getOrigin().y(),2.0)+
                         pow(targetPosition[casp->end_effektor_name][2]-trans.getOrigin().z(),2.0))>0.01) {
                        // with offset
                        vector<float> target_offset;
                        nh->getParam(casp->end_effektor_name+"/target_offset", target_offset);
                        targetPosition[casp->end_effektor_name][0] = trans.getOrigin().x() + target_offset[0];
                        targetPosition[casp->end_effektor_name][1] = trans.getOrigin().y() + target_offset[0];
                        targetPosition[casp->end_effektor_name][2] = trans.getOrigin().z() + target_offset[0];
                        // rotation of endeffector in world frame
                        vector<float> target_rotation;
                        nh->getParam(casp->end_effektor_name+"/target_rotation", target_rotation);
                        targetRotation[casp->end_effektor_name].x() = target_rotation[0];
                        targetRotation[casp->end_effektor_name].y() = target_rotation[1];
                        targetRotation[casp->end_effektor_name].z() = target_rotation[2];
                        targetRotation[casp->end_effektor_name].w() = target_rotation[3];
                        new_target[casp->end_effektor_name] = true;
                    }else{
                        new_target[casp->end_effektor_name] = false;
                    }
                }
                break;
            case InverseKinematicsToTarget:
                for(auto casp:caspr) {
                    if(new_target[casp->end_effektor_name] && reached_target[casp->end_effektor_name]){
                        roboy_communication_middleware::InverseKinematics srv;
                        srv.request.pose.position.x = targetPosition[casp->end_effektor_name][0];
                        srv.request.pose.position.y = targetPosition[casp->end_effektor_name][1];
                        srv.request.pose.position.z = targetPosition[casp->end_effektor_name][2];
                        srv.request.pose.orientation.x = targetRotation[casp->end_effektor_name].x();
                        srv.request.pose.orientation.y = targetRotation[casp->end_effektor_name].y();
                        srv.request.pose.orientation.z = targetRotation[casp->end_effektor_name].z();
                        srv.request.pose.orientation.w = targetRotation[casp->end_effektor_name].w();

                        if (casp->InverseKinematicsService(srv.request, srv.response)) {
                            nh->setParam(casp->end_effektor_name + "/target_pos", srv.response.angles);
                            ik_success[casp->end_effektor_name] = true;
                        } else {
                            ik_success[casp->end_effektor_name] = false;
                        }
                    }
                }
                break;
            case CheckIfTargetReached:
                for(auto casp:caspr) {
                    double norm = (casp->q - casp->q_target).norm();
                    if (norm < 0.01 || (ros::Time::now() - goto_start[casp->end_effektor_name]).toSec() > goto_timeout_sec){
                        reached_target[casp->end_effektor_name] = true;
                        ROS_WARN_THROTTLE(1, "%s reached target", casp->end_effektor_name.c_str());
                    }else{
                        reached_target[casp->end_effektor_name] = false;
                        ROS_WARN_THROTTLE(1, "%s trying to reach targetPosition, error = %lf", casp->end_effektor_name.c_str(), norm);
                    }
                }
                break;
        }

        currentState = NextState(currentState);
    }
}

ActionState Roboy::NextState(ActionState s) {
    ActionState newstate;
    switch (s) {
        case CheckTargetFrames:
            newstate = GetTargetPositionsAndRotations;
            break;
        case GetTargetPositionsAndRotations:
            newstate = InverseKinematicsToTarget;
            break;
        case InverseKinematicsToTarget:
            newstate = CheckIfTargetReached;
            break;
        case CheckIfTargetReached:
            newstate = CheckTargetFrames;
            break;
    }
    return newstate;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roboy", ros::init_options::NoRosout);

    Roboy robot;

    ROS_INFO("STARTING ROBOY MAIN LOOP...");

    robot.main_loop();

    ROS_INFO("TERMINATING...");

    gazebo::shutdown();

    return 0;
}
