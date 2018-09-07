#pragma once

#include <ros/ros.h>
#include <roboy_simulation/CASPR.hpp>
#include <common_utilities/CommonDefinitions.h>
#include "roboy_communication_middleware/Initialize.h"
#include "roboy_communication_middleware/EmergencyStop.h"
#include "roboy_communication_middleware/Record.h"
#include "roboy_communication_middleware/RoboyState.h"
#include "roboy_communication_middleware/MotorStatus.h"
#include "roboy_communication_middleware/ArucoPose.h"
#include <thread>
#include <vector>
#include <thread>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

//! enum for state machine
typedef enum {
    Idle,
    CheckTargetFrames,
    GetTargetPositionsAndRotations,
    InverseKinematicsToTarget,
    CheckIfTargetReached
} ActionState;

class Roboy{
public:
    /**
     * Constructor
     */
    Roboy();

    /**
     * Destructor
     */
    ~Roboy();

    /**
	 * Read from hardware
	 */
    void read(double period);

    /**
     * Write to Hardware
     */
    void write();

    /**
     * This is the main loop
     */
    void main_loop();
private:
    ros::NodeHandlePtr nh;
    ros::Time prev_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    map<string,double> Kp, Kd;
    map<string,vector<double>*> target_pos, target_vel;
    vector<CASPRptr> caspr;
    map<string,Vector3d> targetPosition, target_offset;
    map<string,Quaterniond> targetRotation;
    map<string,bool> ik_success, reached_target, new_target;
    map<string,string> target_frame;
    map<string,ros::Time> goto_start;
    map<string,int> control;
    double goto_timeout_sec = 5;

    //! current state of roboy
    ActionState currentState = CheckTargetFrames;

    /**
     * Statemachine function for next state
     * @param s current State
     * @return next state
     */
    ActionState NextState(ActionState s);

    //! state strings describing each state
    std::map<ActionState, std::string> state_strings = {
            {CheckTargetFrames, "CheckTargetFrames"},
            {GetTargetPositionsAndRotations, "GetTargetPositionsAndRotations"},
            {InverseKinematicsToTarget, "InverseKinematicsToTarget"},
            {CheckIfTargetReached, "CheckIfTargetReached"},
            {Idle, "Idle"}
    };

    tf::TransformListener listener;
};

