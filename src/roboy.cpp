#include "roboy_controller/roboy.hpp"

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
    if(!CASPR::parseSDFusion(sdf,muscInfo))
        ROS_FATAL("error parsing sdf");

    stringstream str;
    str << "initialized CASPR controllers for endeffectors:" << endl;
    for(const string &endeffector:endeffectors) {
        caspr.push_back(boost::shared_ptr<CASPR>(new CASPR(endeffector, muscInfo)));
        target_pos[endeffector] = &caspr.back()->target_pos;
        target_vel[endeffector] = &caspr.back()->target_vel;
        str << endeffector << endl;
    }
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
    for(auto casp:caspr){
        casp->update();
    }
}

void Roboy::write() {
    ROS_DEBUG("write");
    for(auto casp:caspr){
        nh->getParam(casp->end_effektor_name + "/Kp", Kp[casp->end_effektor_name]);
        nh->getParam(casp->end_effektor_name + "/Kd", Kd[casp->end_effektor_name]);
        nh->getParam(casp->end_effektor_name + "/target_pos", casp->target_pos);
        nh->getParam(casp->end_effektor_name + "/target_vel", casp->target_vel);
        casp->updateController(Kp[casp->end_effektor_name],Kd[casp->end_effektor_name]);
    }
}

void Roboy::main_loop(controller_manager::ControllerManager *ControllerManager) {
    cm = ControllerManager;

    // Control loop
    ros::Time prev_time = ros::Time::now();

    currentState = IDLE;
    for (auto casp : caspr) {
        nh->getParam(casp->end_effektor_name + "/target_pos", casp->target_pos);
        nh->getParam(casp->end_effektor_name + "/target_vel", casp->target_vel);
    }

    while (ros::ok()) {
        ROS_INFO_THROTTLE(5, "%s", state_strings[currentState].c_str());
        switch (currentState) {
            case Precompute: {

                precomputeTrajectories();

                for (auto const& k : keyNames) {

                    cout << "Set Target for key " << k << endl;

                    for (int i = 0; i < 7; i++) {
                        cout << keyStates[k][i] << ",";
                    }
                    cout << endl;
                }

                currentState = NextState(currentState);
                break;
            }
            case GrabStick: {
                /*ROS_WARN("Waiting For Roboy to get ready");
                const ros::Time time = ros::Time::now();
                const ros::Duration period = time - prev_time;
                read();
                *target_pos["palm"] = keyStates["stick_left"];
                for (auto casp : caspr) {
                    double diffnorm = 0;
                    for (int i = 0; i < target_pos.size(); i++) {
                        diffnorm += pow(target_pos["palm"]->at(i) - casp->joint_pos[i], 2.0);
                    }
                    diffnorm = sqrt(diffnorm);
                    if (diffnorm < 0.1) {
                        ROS_INFO("Reached target position");
                        closeHand();
                        std::this_thread::sleep_for (std::chrono::seconds(5));
                        prev_time = ros::Time::now();
                        currentState = NextState(currentState);
                        break;
                    }
                }
                write();
                prev_time = ros::Time::now();*/

                break;
            }
            case WaitForInput: {

                ROS_WARN_THROTTLE(1, "Waiting for Input");
                const ros::Time time = ros::Time::now();
                const ros::Duration period = time - prev_time;

                read();
                write();

                nh->getParam("/key", keyName);

                if (keyStates.find(keyName) == keyStates.end()) {
                    break;
                }
                else {
                    currentState = NextState(currentState);;
                }

                prev_time = time;
                break;
            }
            case MoveToKey: {
                ROS_WARN_THROTTLE(1,"Moving to Key");
                const ros::Time time = ros::Time::now();
                const ros::Duration period = time - prev_time;

                read();

                *target_pos["hand_left"] = keyStates[keyName];


                for (auto casp : caspr) {
                    if (casp->end_effektor_name == "hand_left") {
                        nh->setParam(casp->end_effektor_name + "/target_pos", *target_pos["hand_left"]);
                        double diffnorm = 0;
                        for (int i = 0; i < target_pos.size(); i++) {

                            diffnorm += pow(target_pos["hand_left"]->at(i) - casp->joint_pos[i], 2.0);
                            //cout << casp->end_effektor_name << ' ' << target_pos["palm"]->at(i) << ' ' << casp->joint_pos[i] << endl;
                        }

                        ROS_INFO_THROTTLE(1,"error %f", diffnorm);
                        diffnorm = sqrt(diffnorm);
                        if (diffnorm < 0.1) {
                            ROS_INFO("Reached target position");
                            nh->setParam("/key", "null");
                            currentState = NextState(currentState);
                        }
                    }
                }

                write();

                prev_time = time;
                break;
            }
            case HitKey: {
                ROS_WARN_THROTTLE(1,"Trying to hit Key");
                const ros::Time time = ros::Time::now();
                const ros::Duration period = time - prev_time;

                for (auto casp : caspr) {
                    //change
                    if (casp->end_effektor_name == "hand_left")
                        casp->target_vel = {0,0,0,0,0,0.1};
                }

                if (keyHit == keyName || iter > 1) {
                    keyHit = "null";
                    currentState = NextState(currentState);
                    for (auto casp : caspr) {
                        //change
                        if (casp->end_effektor_name == "hand_left")
                            casp->target_vel = {0,0,0,0,0,0};
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
                for (auto casp : caspr) {
                    nh->getParam(casp->end_effektor_name + "/target_pos", casp->target_pos);
                    nh->getParam(casp->end_effektor_name + "/target_vel", casp->target_vel);
                }
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
            newstate = WaitForInput;
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

//precomputes trajectories to be used later on, uses initially obtained positions
void Roboy::precomputeTrajectories() {
    map<string, geometry_msgs::Point> positions = getCoordinates();

    geometry_msgs::Point offset;
    offset.x = 0.0;
    offset.y = 0.0;
    offset.z = 0.2;

    geometry_msgs::Quaternion targetRotation;
    targetRotation.x = 0;
    targetRotation.y = 0;
    targetRotation.z = -0.7071068;
    targetRotation.w = 0.7071068;

    for (auto const& p : positions)
    {
        keyStates[p.first] = {0, 0, 0, 0, 0, 0, 0};
    }

    for (int i = 0; i < 10; i++) {
        //std::this_thread::sleep_for (std::chrono::milliseconds(500));
        read();
        write();
        for (auto const& p : positions)
        {
            bool zeros = std::all_of(keyStates[p.first].begin(), keyStates[p.first].end(), [](int i) { return i == 0; });
            if (!zeros) {
                continue;
            }
            geometry_msgs::Point offsetPosition;
            offsetPosition.x = p.second.x + offset.x;
            offsetPosition.y = p.second.y + offset.y;
            offsetPosition.z = p.second.z + offset.z;

            vector<double> jointAngles;

            if (p.first == "stick_left" || p.first == "stick_right") {
                //change
                jointAngles = getTrajectory(p.second, targetRotation);
            }
            else {

                jointAngles = getTrajectory(offsetPosition, targetRotation);
            }

            keyStates[p.first] = jointAngles;
        }
    }

}


/// gets coords for xylophone and its keys, is only called once
map<string, geometry_msgs::Point> Roboy::getCoordinates()
{
    tf::TransformListener listener;
    //blocking fct: waits for something / anything to get published on tf topic to work on reliable data later on
    //for now, random frames from roboy's model chosen
    listener.waitForTransform("C_0", "world", ros::Time(), ros::Duration(120.0));

    map<string, geometry_msgs::Point> positions;

    for (auto const& k : keyNames) {

        cout << "Getting Transform for key " << k << endl;

        tf::StampedTransform key_world_pos;
        try {
            //todo which target frame should be used? frame order should be: world->xylophone->key
            listener.lookupTransform("world", k, ros::Time(0), key_world_pos);
        }
        catch (tf::LookupException ex) {
            ROS_WARN_THROTTLE(1, "%s", ex.what());
        }

        //only takes position when defining key pose....
        geometry_msgs::Point typeCast;
        typeCast.x = key_world_pos.getOrigin().getX();
        typeCast.y = key_world_pos.getOrigin().getY();
        typeCast.z = key_world_pos.getOrigin().getZ();
        cout << typeCast.x << " " << typeCast.y << " " << typeCast.z << endl;
        positions[k] = typeCast;
    }

    return positions;
}

vector<double> Roboy::getTrajectory(geometry_msgs::Point targetPosition, geometry_msgs::Quaternion targetRotation) {

    string end_effektor = "hand_left";

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<roboy_communication_middleware::InverseKinematics>("/CASPR/" + end_effektor + "/InverseKinematics");

    roboy_communication_middleware::InverseKinematics srv;

    srv.request.pose.position = targetPosition;
    srv.request.pose.orientation = targetRotation;

    if (client.call(srv))
    {
        return srv.response.angles;
    }
    else
    {
        //std::this_thread::sleep_for (std::chrono::milliseconds(200));
        vector<double> zeros = {0, 0, 0, 0, 0, 0, 0};
        return zeros;
    }
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

    // we need an additional update thread, otherwise the controllers won't switch
    thread update_thread(update, &cm);

    ROS_INFO("STARTING ROBOY MAIN LOOP...");

    robot.main_loop(&cm);

    ROS_INFO("TERMINATING...");

    update_thread.join();

    gazebo::shutdown();

    return 0;
}
