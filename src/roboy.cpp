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
        targetRotation[endeffector] = Quaterniond(0, 0, 0, 1);
        ik_success[endeffector] = true;
        reached_target[endeffector] = true;
        new_target[endeffector] = true;
        int control_type;
        nh->getParam(endeffector + "/control", control_type);
        control[endeffector] = control_type;
        str << endeffector << endl;
    }
    ROS_INFO_STREAM(str.str());

    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");

    reset_srv = nh->advertiseService("/CASPR/reset", &Roboy::ResetService, this);
    motor_config_srv = nh->serviceClient<roboy_communication_middleware::MotorConfigService>("/roboy/shoulder_left/middleware/MotorConfig");
    roboy_communication_middleware::MotorConfigService msg;
    msg.request.config.id = SHOULDER_LEFT;
    for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
        msg.request.config.motors.push_back(motor);
        if(motor<NUMBER_OF_MOTORS_MYOCONTROL_0){ // position control for myoControl 0
            msg.request.config.control_mode.push_back(POSITION);
            msg.request.config.outputPosMax.push_back(1000);
            msg.request.config.outputNegMax.push_back(-1000);
            msg.request.config.spPosMax.push_back(1000000);
            msg.request.config.spNegMax.push_back(-1000000);
            msg.request.config.Kp.push_back(1);
            msg.request.config.Ki.push_back(0);
            msg.request.config.Kd.push_back(0);
            msg.request.config.forwardGain.push_back(0);
            msg.request.config.deadBand.push_back(0);
            msg.request.config.IntegralPosMax.push_back(0);
            msg.request.config.IntegralNegMax.push_back(0);
            msg.request.config.outputDivider.push_back(1);
        }else{
            msg.request.config.control_mode.push_back(DISPLACEMENT);
            msg.request.config.outputPosMax.push_back(1000);
            msg.request.config.outputNegMax.push_back(-1000);
            msg.request.config.spPosMax.push_back(500);
            msg.request.config.spNegMax.push_back(0);
            msg.request.config.Kp.push_back(100);
            msg.request.config.Ki.push_back(0);
            msg.request.config.Kd.push_back(0);
            msg.request.config.forwardGain.push_back(0);
            msg.request.config.deadBand.push_back(0);
            msg.request.config.IntegralPosMax.push_back(0);
            msg.request.config.IntegralNegMax.push_back(0);
            msg.request.config.outputDivider.push_back(1);
        }
    }
    if(motor_config_srv.call(msg)){
        ROS_WARN("could not change motor config");
    }
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
    info_time_prev = ros::Time::now();

    while (ros::ok()) {
        currentState = nextState;

        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        if ((ros::Time::now() - info_time_prev).toSec() > 5) {
            ROS_INFO_STREAM( state_strings[currentState] <<
                    (allTargetsReached ? " all targets reached" : " NOT all targets reached")
                            << endl << "control frequency " << 1.0 / period.toSec() << "Hz\nq_target "
                            << caspr[info_counter]->q_target.transpose().format(fmt)
                            << "\ne=" << caspr[info_counter]->e.transpose().format(fmt)
                            << "\nde=" << caspr[info_counter]->de.transpose().format(fmt)
                            << "\ntorques=" << caspr[info_counter]->torques.transpose().format(fmt)
                            << "\nl_dot=" << caspr[info_counter]->l_dot.transpose().format(fmt));
            info_counter++;
            if (info_counter > (caspr.size() - 1))
                info_counter = 0;
            info_time_prev = ros::Time::now();
        }

        read(0.2);
        write();

        switch (currentState) {
            case CheckTargetFrames:
                for (auto casp:caspr) {
                    nh->getParam(casp->end_effektor_name + "/target_frame", target_frame[casp->end_effektor_name]);
                }
                nextState = LookAtTarget;
                break;
            case LookAtTarget:
                lookAt("head", target_frame["wrist_left_1"]);
                nextState = CheckIfHeadTargetReached;
                break;
            case CheckIfHeadTargetReached: {
                double norm = 0;
                for (int i = 0; i < caspr[0]->number_of_dofs; i++) {
                    if (((caspr[0]->joint_angle_mask >> i) & 0x1) == 0)
                        norm += abs(caspr[0]->q[i] - caspr[0]->q_target[i]);
                }
                if (norm < 0.01) {
//                    ROS_INFO_THROTTLE(1,"head to reached target, error %.3lf ", norm);
                    nextState = GetTargetPositionsAndRotations;
                } else {
                    ROS_INFO_THROTTLE(1,"waiting for head to reach target, error %.3lf", norm);
                    nextState = LookAtTarget;
                }
                break;
            }
            case GetTargetPositionsAndRotations:
                for (auto casp:caspr) {
                    if (control[casp->end_effektor_name] == 0) {
                        new_target[casp->end_effektor_name] = false;
                        goto_start[casp->end_effektor_name] = ros::Time::now();
                        continue;
                    }
                    tf::StampedTransform trans;
                    double position_difference = 0.0;
                    // with offset
                    vector<float> target_offset;
                    nh->getParam(casp->end_effektor_name + "/target_offset", target_offset);
                    try {
                        if (listener.waitForTransform("world", target_frame[casp->end_effektor_name].c_str(),
                                                      ros::Time(0), ros::Duration(0.0001))) {
                            listener.lookupTransform("world", target_frame[casp->end_effektor_name].c_str(),
                                                     ros::Time(0), trans);
                            position_difference = sqrt(pow((targetPosition[casp->end_effektor_name][0] + target_offset[0]) - trans.getOrigin().x(), 2.0) +
                                  pow((targetPosition[casp->end_effektor_name][1] + target_offset[1]) - trans.getOrigin().y(), 2.0) +
                                  pow((targetPosition[casp->end_effektor_name][2] + target_offset[2]) - trans.getOrigin().z(), 2.0));
                            new_target[casp->end_effektor_name] = true;
                        } else {
                            ROS_ERROR_THROTTLE(1, "target frame %s is not available",
                                               target_frame[casp->end_effektor_name].c_str());
                            continue;
                        }
                    }
                    catch (tf::LookupException ex) {
                        ROS_WARN_THROTTLE(1, "%s", ex.what());
                        new_target[casp->end_effektor_name] = false;
                        continue;
                    }
                    // if the previous target was reached and a new target is availables
                    if ((position_difference > 0.05 && reached_target[casp->end_effektor_name])) {

                        targetPosition[casp->end_effektor_name][0] = trans.getOrigin().x() + target_offset[0];
                        targetPosition[casp->end_effektor_name][1] = trans.getOrigin().y() + target_offset[1];
                        targetPosition[casp->end_effektor_name][2] = trans.getOrigin().z() + target_offset[2];
                        // rotation of endeffector in world frame
                        vector<float> target_rotation;
                        nh->getParam(casp->end_effektor_name + "/target_rotation", target_rotation);
                        targetRotation[casp->end_effektor_name].x() = target_rotation[0];
                        targetRotation[casp->end_effektor_name].y() = target_rotation[1];
                        targetRotation[casp->end_effektor_name].z() = target_rotation[2];
                        targetRotation[casp->end_effektor_name].w() = target_rotation[3];
                        new_target[casp->end_effektor_name] = true;
                        ROS_INFO_STREAM( "offset " << position_difference << " new target for " << casp->end_effektor_name.c_str()
                                                          << " " << target_frame[casp->end_effektor_name].c_str()
                                                          << targetPosition[casp->end_effektor_name].transpose().format(
                                                                  fmt)
                        );
                    } else {
                        new_target[casp->end_effektor_name] = false;
                    }
                }
                nextState = InverseKinematicsToTarget;
                break;
            case InverseKinematicsToTarget:
                for (auto casp:caspr) {
                    if ((allTargetsReached && new_target[casp->end_effektor_name]
                        && control[casp->end_effektor_name] == 1) || !ik_success[casp->end_effektor_name]) {
                        roboy_communication_middleware::InverseKinematics srv;
//                        srv.request.use_current_robot_pose = ik_success[casp->end_effektor_name];
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
                nextState = CheckIfTargetReached;
                break;
            case CheckIfTargetReached:
                allTargetsReached = true;
                for (auto casp:caspr) {
                    double norm = 0;
                    for(int i=0;i<casp->number_of_dofs;i++) {
                        if(((casp->joint_angle_mask >> i) & 0x1) == 0)
                            norm += abs(casp->q[i] - casp->q_target[i]);
                    }
                    if (norm < 0.01) {
                        reached_target[casp->end_effektor_name] = true;
                    } else {
                        reached_target[casp->end_effektor_name] = false;
                        ROS_INFO_STREAM_THROTTLE(1,casp->end_effektor_name << " --> "<< target_frame[casp->end_effektor_name] << " not reached, error " << norm);
                        allTargetsReached = false;
                    }
                }
                nextState = CheckTargetFrames;
                break;
        }
    }
}

bool Roboy::lookAt(string root_frame, string target_frame) {
    tf::StampedTransform root_transform, target_transform;
    try {
        if (listener.waitForTransform("world", root_frame.c_str(), ros::Time(0), ros::Duration(0.01)))
            listener.lookupTransform("world", root_frame.c_str(), ros::Time(0), root_transform);
        else
            return false;
        if (listener.waitForTransform("world", target_frame.c_str(), ros::Time(0), ros::Duration(0.01)))
            listener.lookupTransform("world", target_frame.c_str(), ros::Time(0), target_transform);
        else
            return false;
    }
    catch (tf::LookupException ex) {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        return false;
    }

    Eigen::Affine3d root, target;
    tf::transformTFToEigen(root_transform, root);
    tf::transformTFToEigen(target_transform, target);

    Vector3d eye_ray(0, -1, 0), target_ray = (target.matrix().block(0, 3, 3, 1) - root.matrix().block(0, 3, 3, 1));
    target_ray.normalize();
    eye_ray = root.matrix().block(0, 0, 3, 3) * eye_ray;

    vector<double> q;
    q.push_back(caspr[0]->q[0]);
    q.push_back(caspr[0]->q[1]);
    q.push_back(caspr[0]->q[2] - (target_ray[2] - eye_ray[2]));
    q.push_back(caspr[0]->q[3] + (target_ray[0] - eye_ray[0]));
    nh->setParam("head/target_pos", q);

//    Vector3d pos = root.matrix().block(0,3,3,1);
//    publishRay(pos,eye_ray,"world","eye_ray",1111111,COLOR(1,0,0,1),0.1);
//    publishRay(pos,target_ray,"world","eye_ray",1111112,COLOR(0,1,0,1),0.1);
//
//    ROS_INFO_STREAM_THROTTLE(1, "euler " << target_ray[2]-eye_ray[2] << " " << target_ray[0]-eye_ray[0]);
}

bool Roboy::ResetService(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res){
    for (auto casp:caspr) {
        casp->q.setZero();
        casp->qd.setZero();
        casp->qdd.setZero();
    }
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
