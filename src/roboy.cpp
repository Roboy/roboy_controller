#include "roboy_controller/roboy.hpp"
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

#include <cstdlib>
#include <map>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"

using namespace std;

bool Roboy::shutdown_flag = false;

Roboy::Roboy() {
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy", ros::init_options::NoRosout);
    }

    init_srv = nh->advertiseService("/roboy/initialize", &Roboy::initializeControllers, this);

    cmd = new double[NUMBER_OF_MOTORS_PER_FPGA];
    pos = new double[NUMBER_OF_MOTORS_PER_FPGA];
    vel = new double[NUMBER_OF_MOTORS_PER_FPGA];
    eff = new double[NUMBER_OF_MOTORS_PER_FPGA];

//    motorStatus_sub = nh->subscribe("/roboy/middleware/MotorStatus", 1, &Roboy::MotorStatus, this);

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
    for (const string &endeffector : endeffectors) {
        caspr.push_back(boost::shared_ptr<CASPR>(new CASPR(endeffector, muscInfo)));
        target_pos[endeffector] = &caspr.back()->target_pos;
        target_vel[endeffector] = &caspr.back()->target_vel;
        str << endeffector << endl;
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("xylophone/hitdetection", 1, &Roboy::detectHit, this);

    ROS_INFO_STREAM(str.str());
}

Roboy::~Roboy() {
    delete []cmd;
    delete []pos;
    delete []vel;
    delete []eff;
}

void Roboy::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    ROS_INFO_THROTTLE(10, "receiving motor status");
    for (uint motor = 0; motor < msg->position.size(); motor++) {
        pos[motor] = msg->position[motor];
        vel[motor] = msg->velocity[motor];
        eff[motor] = msg->displacement[motor];
    }
}

bool Roboy::initializeControllers(roboy_communication_middleware::Initialize::Request &req,
                                  roboy_communication_middleware::Initialize::Response &res) {
    initialized = false;

    vector<string> start_controllers;
    for (uint i = 0; i < req.idList.size(); i++) {
        char resource[100];
        sprintf(resource, "motor%d", req.idList[i]);
        uint ganglion = req.idList[i] / 4;
        uint motor = req.idList[i] % 4;
        ROS_INFO("motor%d control_mode %d", motor, req.controlmode[i]);

        // connect and register the actuator state interface
        start_controllers.push_back(resource);
        hardware_interface::ActuatorStateHandle state_handle(resource, &pos[req.idList[i]], &vel[req.idList[i]],
                &eff[req.idList[i]]);
        act_state_interface.registerHandle(state_handle);

        // connect and register the actuator interface
        hardware_interface::ActuatorHandle actuator_handle(act_state_interface.getHandle(resource), &cmd[req.idList[i]]);
        act_command_interface.registerHandle(actuator_handle);
    }

    registerInterface(&act_command_interface);

    string str;
    vector<string> resources = act_command_interface.getNames();
    if (!resources.empty())
        str.append("actuator controllers:\n");
    for (uint i = 0; i < resources.size(); i++) {
        str.append(resources[i]);
        str.append(" ");
    }

    ROS_INFO("Stopping and unloading the controllers that are already running");
    stopControllers(start_controllers);
    unloadControllers(start_controllers);

    ROS_INFO("Resources registered to hardware interface:\n%s", str.c_str());
    if (!loadControllers(start_controllers))
        return false;

    ROS_INFO("Starting controllers now...");
    if (!startControllers(start_controllers))
        ROS_WARN("could not start controllers, try starting via /controller_manager/switch_controller service");

    initialized = true;
    return true;
}

void Roboy::read() {
    ROS_DEBUG("read");
    for (auto casp : caspr) {
        casp->update();
    }
}

void Roboy::write() {
    ROS_DEBUG("write");
    for (auto casp : caspr) {
        nh->getParam(casp->end_effektor_name + "/Kp", Kp[casp->end_effektor_name]);
        nh->getParam(casp->end_effektor_name + "/Kd", Kd[casp->end_effektor_name]);
        if (currentState == IDLE) {
            nh->getParam(casp->end_effektor_name + "/target_pos", casp->target_pos);
            nh->getParam(casp->end_effektor_name + "/target_vel", casp->target_vel);
        }
        casp->updateController(Kp[casp->end_effektor_name], Kd[casp->end_effektor_name]);
    }
}

void Roboy::main_loop(controller_manager::ControllerManager *ControllerManager) {
    cm = ControllerManager;

    // Control loop
    ros::Time prev_time = ros::Time::now();
    string keyName;
    int iter = 0;

    currentState = Precompute;

    while (ros::ok()) {
        ROS_INFO_THROTTLE(5, "%s", state_strings[currentState].c_str());
        switch (currentState) {
        case Precompute: {
            precomputeTrajectories();
            currentState = NextState(currentState);
            break;
        }
        case GrabStick: {
            ROS_WARN("Waiting For Roboy to get ready");
            const ros::Time time = ros::Time::now();
            const ros::Duration period = time - prev_time;
            read();

            *target_pos["palm"] = keyStates["stick_left"];

            /*    double diffnorm = 0;
                for (int i = 0; i < target_pos.size(); i++)
                    diffnorm += pow(target_pos["palm"]->at(i) - casp->joint_pos[i], 2.0);
                diffnorm = sqrt(diffnorm);
                if (diffnorm < 0.1) {
                    ROS_INFO("Reached target position");
                    closeHand();
                    std::this_thread::sleep_for (std::chrono::seconds(5));
                    prev_time = ros::Time::now();
                    currentState = NextState(currentState);
                    break;
                }*/

            write();

            prev_time = ros::Time::now();

            break;
        }
        case WaitForInput: {
            ROS_WARN("Waiting for Input");
            const ros::Time time = ros::Time::now();
            const ros::Duration period = time - prev_time;

            read();

            nh->getParam("/key", keyName);

            if (keyStates.find(keyName) == keyStates.end()) {
                break;
            }
            else {
                currentState = NextState(currentState);
            }

            write();

            prev_time = time;
            break;
        }
        case MoveToKey: {
            ROS_WARN("Moving to Key");
            const ros::Time time = ros::Time::now();
            const ros::Duration period = time - prev_time;

            read();

            *target_pos["palm"] = keyStates[keyName];

            for (auto casp : caspr) {

                double diffnorm = 0;
                for (int i = 0; i < target_pos.size(); i++)
                    diffnorm += pow(target_pos[casp->end_effektor_name]->at(i) - casp->joint_pos[i], 2.0);
                diffnorm = sqrt(diffnorm);
                if (diffnorm < 0.1) {
                    ROS_INFO("Reached target position");
                    nh->setParam("/key", "null");
                    currentState = NextState(currentState);
                }
            }

            write();

            prev_time = time;
            break;
        }
        case HitKey: {
            ROS_WARN("Trying to hit Key");
            const ros::Time time = ros::Time::now();
            const ros::Duration period = time - prev_time;

            for (auto casp : caspr) {
                //change
                if (casp->end_effektor_name == "wrist")
                    casp->target_vel = {1.0, 1.0, 1.0, 1.0};
            }

            if (keyHit == keyName || iter > 10) {
                keyHit = "null";
                currentState = NextState(currentState);
                for (auto casp : caspr) {
                    //change
                    if (casp->end_effektor_name == "wrist")
                        casp->target_vel = {0.0, 0.0, 0.0, 0.0};
                }
            }

            std::this_thread::sleep_for (std::chrono::seconds(1));
            write();

            prev_time = time;
            break;
        }
        case IDLE: {
            const ros::Time time = ros::Time::now();
            const ros::Duration period = time - prev_time;

            read();
            write();

            prev_time = time;
            break;
        }
        }
    }
}

bool Roboy::loadControllers(vector<string> controllers) {
    bool controller_loaded = true;
    for (auto controller : controllers) {
        if (!cm->loadController(controller)) {
            controller_loaded = false;
        }
    }
    return controller_loaded;
}

bool Roboy::unloadControllers(vector<string> controllers) {
    bool controller_loaded = true;
    for (auto controller : controllers) {
        if (!cm->unloadController(controller)) {
            controller_loaded = false;
        }
    }
    return controller_loaded;
}

bool Roboy::startControllers(vector<string> controllers) {
    vector<string> stop_controllers;
    int strictness = 1; // best effort
    return cm->switchController(controllers, stop_controllers, strictness);
}

bool Roboy::stopControllers(vector<string> controllers) {
    vector<string> start_controllers;
    int strictness = 1; // best effort
    return cm->switchController(start_controllers, controllers, strictness);
}

ActionState Roboy::NextState(ActionState s) {
    ActionState newstate;
    switch (s) {
    case Precompute:
        newstate = GrabStick;
        break;
    case GrabStick:
        newstate = WaitForInput;
        break;
    case WaitForInput:
        newstate = MoveToKey;
        break;
    case MoveToKey:
        newstate = HitKey;
        break;
    case HitKey:
        newstate = WaitForInput;
        break;
    }

    return newstate;
}

//@TODO
void Roboy::closeHand() {
    vector<string> fingers = {    "left_little_limb3",
                                  "left_ring_limb3",
                                  "left_middle_limb3",
                                  "left_index_limb3",
                                  "left_thumb_limb3"
                             };
    for (auto casp : caspr) {
        for (auto const& f : fingers) {
            if (casp->end_effektor_name == f)
                casp->target_vel = {1.0, 1.0, 1.0, 1.0};
        }
    }
}

void Roboy::precomputeTrajectories() {
    map<string, geometry_msgs::Vector3> positions = getCoordinates();

    geometry_msgs::Vector3 offset;
    offset.x = 0.1;
    offset.y = 0.1;
    offset.z = 0.1;

    vector<double> bestRotation = { 0, 1, 0, 0 };

    for (auto const& p : positions)
    {
        geometry_msgs::Vector3 offsetPosition;
        offsetPosition.x = p.second.x + offset.x;
        offsetPosition.y = p.second.y + offset.y;
        offsetPosition.z = p.second.z + offset.z;

        vector<double> jointAngles;

        if (p.first == "stick_left" || p.first == "stick_right") {
            //change
            std::vector<double> stickRotation = { 0, 1, 0, 0 };
            jointAngles = getTrajectory(offsetPosition, stickRotation);
        }
        else {
            jointAngles = getTrajectory(offsetPosition, bestRotation);
        }

        keyStates[p.first] = jointAngles;
    }

}

map<string, geometry_msgs::Vector3> Roboy::getCoordinates()
{
    tf::TransformListener listener;
    //waiting for anything to get published on tf topic
    listener.waitForTransform("A_0", "base", ros::Time(), ros::Duration(1.0));

    ros::Rate rate(10.0);
    map<string, geometry_msgs::Vector3> positions;

    for (auto const& k : keyNames) {

        cout << "Getting Transform for key " << k << endl;

        tf::StampedTransform trans;
        try {
            listener.lookupTransform("palm", "torso", ros::Time(0), trans);
        }
        catch (tf::LookupException ex) {
            ROS_WARN_THROTTLE(1, "%s", ex.what());
        }

        geometry_msgs::Vector3 typeCast;
        typeCast.x = trans.getOrigin().getX();
        typeCast.y = trans.getOrigin().getY();
        typeCast.z = trans.getOrigin().getZ();
        cout << typeCast.x << " " << typeCast.y << " " << typeCast.z << endl;
        positions[k] = typeCast;
    }

    return positions;
}

vector<double> Roboy::getTrajectory(geometry_msgs::Vector3 targetPosition, vector<double> targetRotation) {

    string end_effektor = "palm";

    int type = 0;

    targetPosition.x = 0.9;
    targetPosition.y = 0.0;
    targetPosition.z = 0.5;
    targetRotation = {0, 1, 0, 0};

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<roboy_communication_middleware::InverseKinematics>("/CASPR/" + end_effektor + "/InverseKinematics");

    roboy_communication_middleware::InverseKinematics srv;

    srv.request.type = type;
    srv.request.targetPosition = targetPosition;
    srv.request.targetRotation = targetRotation;


    if (client.call(srv))
    {
        cout << "Resulting Joint Angles are: ";
        int i = 0;
        for (auto const& a : srv.response.angles) {
            cout << srv.response.angles[i] << " | ";
            i++;
        }
        cout << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service InverseKinematicsService");
    }

    return srv.response.angles;
}

void Roboy::detectHit(const std_msgs::String::ConstPtr & msg) {
    keyHit = msg->data.c_str();
}


void update(controller_manager::ControllerManager * cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10);
    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        cm->update(time, period);
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "roboy", ros::init_options::NoRosout);

    Roboy robot;

    controller_manager::ControllerManager cm(&robot);

    thread update_thread(update, &cm);

    ROS_INFO("STARTING ROBOY MAIN LOOP...");

    robot.main_loop(&cm);

    ROS_INFO("TERMINATING...");

    update_thread.join();

    gazebo::shutdown();

    return 0;
}