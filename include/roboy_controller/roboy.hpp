#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <roboy_simulation/CASPR.hpp>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/rviz_visualization.hpp>
#include "roboy_communication_middleware/Initialize.h"
#include "roboy_communication_middleware/EmergencyStop.h"
#include "roboy_communication_middleware/Record.h"
#include "roboy_communication_middleware/RoboyState.h"
#include "roboy_communication_middleware/MotorStatus.h"
#include "roboy_communication_middleware/MotorConfig.h"
#include "roboy_communication_middleware/MotorConfigService.h"
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
    LookAtTarget,
    CheckIfHeadTargetReached,
    CheckTargetFrames,
    GetTargetPositionsAndRotations,
    InverseKinematicsToTarget,
    CheckIfTargetReached,
    TrackRealHardwareToTarget
} ActionState;

class Roboy:private rviz_visualization{
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
    bool lookAt(string frame, string target_frame);

    bool ResetService(std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res);

    ros::NodeHandlePtr nh;
    ros::ServiceServer reset_srv;
    ros::ServiceClient motor_config_srv;
    ros::Time prev_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    map<string,double> Kp, Kd;
    map<string,vector<double>*> target_pos, target_vel;
    Eigen::IOFormat fmt;
    ros::Time info_time_prev;
    int info_counter = 0;
    vector<CASPRptr> caspr;
    map<string,Vector3d> targetPosition, target_offset;
    map<string,Quaterniond> targetRotation;
    bool allTargetsReached = true;
    map<string,bool> ik_success, reached_target, new_target;
    map<string,string> target_frame;
    map<string,ros::Time> goto_start;
    map<string,int> control;
    double goto_timeout_sec = 5;

    //! current state of roboy
    ActionState currentState = TrackRealHardwareToTarget, nextState;
    map<string,ActionState> stringToActionState = {
            {"Idle", Idle},
            {"LookAtTarget", LookAtTarget},
            {"CheckIfHeadTargetReached", CheckIfHeadTargetReached},
            {"CheckTargetFrames", CheckTargetFrames},
            {"GetTargetPositionsAndRotations", GetTargetPositionsAndRotations},
            {"InverseKinematicsToTarget", InverseKinematicsToTarget},
            {"CheckIfTargetReached", CheckIfTargetReached},
            {"TrackRealHardwareToTarget", TrackRealHardwareToTarget}
    };

    //! state strings describing each state
    std::map<ActionState, std::string> actionStateToString = {
            {Idle, "Idle"},
            {LookAtTarget, "LookAtTarget"},
            {CheckIfHeadTargetReached, "CheckIfHeadTargetReached"},
            {CheckTargetFrames, "CheckTargetFrames"},
            {GetTargetPositionsAndRotations, "GetTargetPositionsAndRotations"},
            {InverseKinematicsToTarget, "InverseKinematicsToTarget"},
            {CheckIfTargetReached, "CheckIfTargetReached"},
            {TrackRealHardwareToTarget, "TrackRealHardwareToTarget"}
    };

    tf::TransformListener listener;
};

