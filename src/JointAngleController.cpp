#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <roboy_communication_middleware/ControllerState.h>
#include <roboy_communication_middleware/Steer.h>
#include <roboy_communication_middleware/SetTrajectory.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/JointAngle.h>
#include <roboy_communication_middleware/JointCommand.h>
#include <common_utilities/CommonDefinitions.h>
#include <ecl/geometry.hpp>
#include <mutex>

using namespace std;

class JointAngleController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    JointAngleController() {

    };

    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
        spinner  = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1)) ;
        spinner->start();
        // get joint name from the parameter server
        if (!n.getParam("joint_name", joint_name)) {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n.getParam("joint",jointID);
        n.getParam("extendor",extendor);
        n.getParam("init_angle",setPointAngle);
        ROS_WARN("JointAngleController %s for joint %d as %s initialized", joint_name.c_str(), jointID,(extendor?"extendor":"flexor"));
        joint = hw->getHandle(joint_name);  // throws on failure
        jointStatus_sub = n.subscribe("/roboy/middleware/JointStatus", 1, &JointAngleController::calculateForceForAngleCB, this);
        jointAngleOffset_sub = n.subscribe("/roboy/middleware/JointAngleOffset", 1, &JointAngleController::SetAngleOffsetCB, this);
        char str[100];
        sprintf(str,"/roboy/middleware/joint%d", jointID);
        jointAngle_sub = n.subscribe(str, 1, &JointAngleController::jointAngleSetpointCB, this);
        jointCommand_sub = n.subscribe("/roboy/middleware/JointCommand", 1, &JointAngleController::jointCommandCB, this);
        return true;
    }

    void update(const ros::Time &time, const ros::Duration &period) {
        double eff = joint.getEffort();
        joint.setCommand(setpoint);
    }

    void calculateForceForAngleCB(const roboy_communication_middleware::JointStatus::ConstPtr &msg){
        lock_guard<mutex> lock(mux);
        angle = (msg->relAngles[jointID]/4096.0f * 360.0f+jointAngleOffset);
        error = setPointAngle - angle;
        float pterm = Kp * error;
        float dterm = Kd * (error - error_previous);
        integral += Ki * error;
        if (integral >= integral_max) {
            integral = integral_max;
        } else if (integral <= integral_min) {
            integral = -integral_min;
        }
        float result = pterm + dterm + integral;
        if (result <= -smooth_distance) {
            if (extendor == 1)
                setpoint = offset - result;
            else if (extendor == 0)
                setpoint = offset;
        } else if (result < smooth_distance) {
            if (extendor == 1)
                setpoint = offset + powf(result - smooth_distance, 2.0f) / (4.0f * smooth_distance);
            else if (extendor == 0)
                setpoint = offset + powf(result + smooth_distance, 2.0f) / (4.0f * smooth_distance);
        } else {
            if (extendor == 1)
                setpoint = offset;
            else if (extendor == 0)
                setpoint = offset + result;
        }
        error_previous = error;
        ROS_INFO("joint%d %s setpoint %f angle %f error: %f", jointID, joint_name.c_str(), setpoint, angle, error);
    }

    void jointAngleSetpointCB(const std_msgs::Float32::ConstPtr& msg) {
        if(msg->data>=0 && msg->data<=360)
            setPointAngle = msg->data;
        else
            ROS_WARN("received invalid setpoint %f for %s", msg->data, joint_name.c_str());
    }

    void jointCommandCB(const roboy_communication_middleware::JointCommand::ConstPtr& msg) {
        ROS_INFO_THROTTLE(5,"%s receiving angle command %f", joint_name.c_str(), msg->dq[jointID] );
        setPointAngle = angle + msg->dq[jointID];
    }

    void SetAngleOffsetCB(const roboy_communication_middleware::JointAngle::ConstPtr& msg) {
        ROS_WARN("%s received new JointAngle offset %f", joint_name.c_str(), msg->angle[jointID]);
        jointAngleOffset = msg->angle[jointID];
    }

    void starting(const ros::Time &time) { ROS_INFO("controller started for %s", joint_name.c_str()); }

    void stopping(const ros::Time &time) { ROS_INFO("controller stopped for %s", joint_name.c_str()); }

private:
    hardware_interface::JointHandle joint;
    double setpoint = 0;
    float jointAngleOffset = 0, setPointAngle = 40, angle = 0;
    string joint_name;
    ros::NodeHandle n;
    ros::Subscriber jointStatus_sub, jointAngle_sub, jointAngleOffset_sub, jointCommand_sub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    int jointID = -1;
    int extendor;
    float Kp = 20, Ki = 0, Kd = 80;
    float error = 0, error_previous = 0, integral_max = 360, integral_min = 0, integral = 0;
    const float smooth_distance = 50;
    const float offset = 20;
    mutex mux;
};
PLUGINLIB_EXPORT_CLASS(JointAngleController, controller_interface::ControllerBase);