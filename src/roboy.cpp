#include "roboy_controller/roboy.hpp"

bool Roboy::shutdown_flag = false;

Roboy::Roboy(int argc, char *argv[]) {
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

    motorStatus_sub = nh->subscribe("/roboy/middleware/MotorStatus", 1, &Roboy::MotorStatus, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
    spinner->start();

    bool initializeJointAngles = true;

}

Roboy::~Roboy() {
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
    // nothing to be done here, since the motor status comes via ros topic /roboy/MotorStatus
}

void Roboy::write() {
    ROS_DEBUG("write");
}

void Roboy::main_loop(controller_manager::ControllerManager *ControllerManager, SimulationControl &sim_control) {
    cm = ControllerManager;

    // Control loop
    ros::Time prev_time = ros::Time::now();

    currentState = WaitForInitialize;

    while (ros::ok()) {
        ROS_INFO_THROTTLE(5, "%s", state_strings[currentState].c_str());
        switch (currentState) {
            case WaitForInitialize: {
                if (!initialized) {
                    // idle
                    continue;
                } else {
                    // go to next state
                    prev_time = ros::Time::now();
                    break;
                }
            }
            case Control: {
                const ros::Time time = ros::Time::now();
                const ros::Duration period = time - prev_time;

                read();
                write();

                prev_time = time;
                break;
            }
            case Simulate: {
                sim_control.simulate();
                break;
            }
        }
        // get next state from state machine
        currentState = NextState(currentState);
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
            newstate = Control;
            break;
        case Control:
            newstate = Simulate;
            break;
        case Simulate:
            newstate = Control;
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

    if(argc!=2)
        ROS_FATAL("please supply model name, eg roslaunch roboy_controller roboy.launch model_name:=roboy_xylophone_left_arm");

    // setup Gazebo server
    if (gazebo::setupServer()) {
        std::cout << "Gazebo server setup successful\n";
    } else {
        std::cout << "Gazebo server setup failed!\n";
    }

    SimulationControl &sim_control = SimulationControl::getInstance();
    physics::WorldPtr world = sim_control.loadWorld("worlds/empty.world");
    gazebo::sensors::run_once(true);
    gazebo::sensors::run_threads();

    physics::ModelPtr model = sim_control.loadModel(world, argv[1]);

    Roboy robot(argc, argv);

    controller_manager::ControllerManager cm(&robot);

    // we need an additional update thread, otherwise the controllers won't switch
    thread update_thread(update, &cm);

    ROS_INFO("STARTING ROBOY MAIN LOOP...");

    robot.main_loop(&cm,sim_control);

    ROS_INFO("TERMINATING...");

    update_thread.join();

    gazebo::shutdown();

    return 0;
}
