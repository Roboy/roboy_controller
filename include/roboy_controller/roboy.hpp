#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <roboy_simulation/CASPR.hpp>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/rviz_visualization.hpp>
#include <roboy_communication_middleware/Initialize.h>
#include <roboy_communication_middleware/EmergencyStop.h>
#include <roboy_communication_middleware/Record.h>
#include <roboy_communication_middleware/RoboyState.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorConfig.h>
#include <roboy_communication_middleware/MotorConfigService.h>
#include <roboy_communication_middleware/ArucoPose.h>
#include <roboy_communication_control/LookAtAction.h>
#include <roboy_communication_control/MoveEndEffectorAction.h>
#include <thread>
#include <vector>
#include <thread>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

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

    void sendToRealHardware(CASPRptr casp);

    /**
     * This is the main loop
     */
    void main_loop();
private:
    void lookAt(const roboy_communication_control::LookAtGoalConstPtr &goal);
    void moveEndEffector(const roboy_communication_control::MoveEndEffectorGoalConstPtr &goal);

    bool ResetService(std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res);

    void setControlMode(CASPRptr casp);

    ros::NodeHandle nh;
    boost::shared_ptr<actionlib::SimpleActionServer<roboy_communication_control::LookAtAction>> lookAt_as;
    boost::shared_ptr<actionlib::SimpleActionServer<roboy_communication_control::MoveEndEffectorAction>> moveEndEffector_as;
    ros::ServiceServer reset_srv;
    ros::ServiceClient motor_config_srv;
    ros::Time prev_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    map<string,double> Kp, Kd;
    Eigen::IOFormat fmt;
    ros::Time info_time_prev;
    int info_counter = 0;
    vector<CASPRptr> caspr;
    map<string, CASPRptr> casprByName;
    vector<string> joint_names;
    map<string,double*> q,qd;

    tf::TransformListener listener;
};

