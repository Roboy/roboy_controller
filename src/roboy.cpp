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

    //motorStatus_sub = nh->subscribe("/roboy/middleware/MotorStatus", 1, &Roboy::MotorStatus, this);

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
        target_pos[endeffector] = &caspr.back()->q_target;
        target_vel[endeffector] = &caspr.back()->qd_target;
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

void Roboy::read(ros::Duration period) {
    ROS_DEBUG("read");
    for(auto casp:caspr){
        casp->update(period);
    }
}

void Roboy::write(ros::Duration period) {
    ROS_DEBUG("write");
    for(auto casp:caspr){
        nh->getParam(casp->end_effektor_name + "/Kp", Kp[casp->end_effektor_name]);
        nh->getParam(casp->end_effektor_name + "/Kd", Kd[casp->end_effektor_name]);
        vector<double> target_pos, target_vel;
        nh->getParam(casp->end_effektor_name + "/target_pos", target_pos);
        nh->getParam(casp->end_effektor_name + "/target_vel", target_vel);
        for(int i=0;i<casp->q_target.rows();i++){
            casp->q_target[i] = target_pos[i];
            casp->qd_target[i] = target_vel[i];
        }
        casp->updateController(Kp[casp->end_effektor_name],Kd[casp->end_effektor_name],period);
    }
}

void Roboy::main_loop(controller_manager::ControllerManager *ControllerManager) {
    cm = ControllerManager;

    // Control loop
    ros::Time prev_time = ros::Time::now();

    currentState = SetpointControl;

    while (ros::ok()) {
        ROS_INFO_THROTTLE(5, "%s", state_strings[currentState].c_str());
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        switch (currentState) {
            case WaitForInitialize: {
                if (!initialized) {
                    // idle
                    continue;
                } else {
                    break;
                }
            }
            case SetpointControl: {
                read(period);
                write(period);
                break;
            }
            case TrajectoryControl: {
                read(period);
                for(auto casp:caspr){
                    if(casp->new_trajectory){
                        casp->trajectory_index = 0;
                        casp->new_trajectory = false;
                    }

                    if (casp->trajectory_index >= casp->trajectory.joint_trajectory.points.size()) {
                        for(int i=0; i<target_pos[casp->end_effektor_name]->rows(); i++) {
                            (*target_pos[casp->end_effektor_name])[i] = casp->trajectory.joint_trajectory.points[casp->trajectory_index].positions[i];
                            (*target_vel[casp->end_effektor_name])[i] = casp->trajectory.joint_trajectory.points[casp->trajectory_index].velocities[i];
                        }
                    }

                    double diffnorm = 0;
                    for (int i = 0; i < target_pos[casp->end_effektor_name]->rows(); i++)
                        diffnorm += pow((*target_pos[casp->end_effektor_name])[i] - casp->joint_pos[i], 2.0);
                    diffnorm = sqrt(diffnorm);
                    if (diffnorm < 0.1) {
                        if (casp->trajectory_index < casp->trajectory.joint_trajectory.points.size()) {
                            ROS_INFO_STREAM_THROTTLE(3,casp->end_effektor_name << " trajectory setpoint #" << casp->trajectory_index << " reached with error " << diffnorm);
                            casp->trajectory_index++;
                        }
                    }
                }
                write(period);
                break;
            }
        }
        // get next state from state machine
        currentState = NextState(currentState);
        prev_time = time;
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
        case WaitForInitialize:
            newstate = SetpointControl;
            break;
        case SetpointControl:
            newstate = SetpointControl;
            break;
        case TrajectoryControl:
            newstate = TrajectoryControl;
            break;
    }
    return newstate;
}

void update(controller_manager::ControllerManager *cm) {
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
