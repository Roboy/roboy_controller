#pragma once

#include "common_utilities/CommonDefinitions.h"
#include "roboy_communication_middleware/Initialize.h"
#include "roboy_communication_middleware/EmergencyStop.h"
#include "roboy_communication_middleware/Record.h"
#include "roboy_communication_middleware/RoboyState.h"
#include "roboy_communication_middleware/MotorStatus.h"
#include "roboy_communication_middleware/ArucoPose.h"
#include "geometry_msgs/Pose.h"
#include <roboy_simulation/simulationControl.hpp>

// ros
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

// std
#include <thread>
#include <vector>
#include <mutex>
#include <thread>
#include <map>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

//! enum for state machine
typedef enum {
    WaitForInitialize,
    Control,
    Simulate
} ActionState;

class Roboy : public hardware_interface::RobotHW {
public:
    /**
     * Constructor
     */
    Roboy(int argc, char* argv[]);

    /**
     * Destructor
     */
    ~Roboy();

    /**
     * This function initialises the requested motors
     */
    bool initializeControllers(roboy_communication_middleware::Initialize::Request &req,
                               roboy_communication_middleware::Initialize::Response &res);
    /**
	 * Read from hardware
	 */
    void read();

    /**
     * Write to Hardware
     */
    void write();

    /**
     * This is the main loop
     */
    void main_loop(controller_manager::ControllerManager *ControllerManager, SimulationControl &sim_control);
//    /**
//     * Handles signals and shuts down everything
//     * @param sig signals
//     */
//    static void sigintHandler(int sig);

private:
    /**
     * Subscriber callback for motor status
     * @param msg
     */
    void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
    /*
     * This function loads the controllers registered to the individual joint interfaces
     * @param controllers names of controllers
     * @return success
     */
    bool loadControllers(vector<string> controllers);

    /*
     * This function unloads the controllers registered to the individual joint interfaces
     * @param controllers names of controllers
     * @return success
     */
    bool unloadControllers(vector<string> controllers);

    /*
	 * This function starts the controllers registered to the individual joint interfaces
	 * @param controllers names of controllers
	 * @return success
	 */
    bool startControllers(vector<string> controllers);

    /*
	 * This function stops the controllers registered to the individual joint interfaces
	 * @param controllers names of controllers
	 * @return success
	 */
    bool stopControllers(vector<string> controllers);

    ros::NodeHandlePtr nh;
    double *cmd;
    double *pos;
    double *vel;
    double *eff;
    ros::Time prevTime;
    int8_t recording;
    bool initialized = false;
    static bool shutdown_flag;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber motorStatus_sub;

    hardware_interface::ActuatorStateInterface act_state_interface;
    hardware_interface::ActuatorCommandInterface act_command_interface;

    controller_manager::ControllerManager *cm = nullptr;
    ros::Subscriber steer_recording_sub, arucoMarker_sub;
    map<int, Vector3f> arucoMarkerPosition;
    ros::ServiceServer init_srv, record_srv, resetSpring_srv;
    ros::Publisher recordResult_pub, jointAnglesOffset_pub, hipCenter_pub;

    roboy_communication_middleware::RoboyState roboyStateMsg;

    vector<ros::Publisher> displacement_pub;

    //! current state of roboy
    ActionState currentState;

    /**
     * Statemachine function for next state
     * @param s current State
     * @return next state
     */
    ActionState NextState(ActionState s);

    //! state strings describing each state
    std::map<ActionState, std::string> state_strings = {
            {WaitForInitialize, "Waiting for initialization of controllers"},
            {Control,       "Control loop"},
            {Simulate,         "Simulating"}
    };
};

