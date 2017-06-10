#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <roboy_communication_middleware/ControllerState.h>
#include <roboy_communication_middleware/Steer.h>
#include <roboy_communication_middleware/SetTrajectory.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <common_utilities/CommonDefinitions.h>
#include <ecl/geometry.hpp>
#include <mutex>

using namespace std;

class JointAngleController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    JointAngleController() {

    };

    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
        // get joint name from the parameter server
        if (!n.getParam("joint_name", joint_name)) {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n.getParam("joint",jointID);
        n.getParam("extendor",extendor);
        ROS_INFO("JointAngleController %s for joint %d initialized", joint_name.c_str(), jointID);
        joint = hw->getHandle(joint_name);  // throws on failure
        jointStatus_sub = n.subscribe("/roboy/middleware/JointStatus", 1, &JointAngleController::calculateForceForAngle, this);
        return true;
    }

    void update(const ros::Time &time, const ros::Duration &period) {
        double eff = joint.getEffort();
        joint.setCommand(setpoint);
    }

    void calculateForceForAngle(const roboy_communication_middleware::JointStatus::ConstPtr &msg){
        lock_guard<mutex> lock(mux);
        error = setpoint - (msg->relAngles[jointID] / 4096.0 * 360.0);
        float pterm = Kp * error;
        float dterm = Ki * (error - error_previous);
        integral += Ki  * error;
        if(integral>=integral_max){
            integral = integral_max;
        }else if(integral<=integral_min){
            integral = -integral_min;
        }
        float result = pterm + dterm + integral;
        if (result <= -smooth_distance) {
            if(extendor)
                setpoint = offset - result;
            else
                setpoint = offset;
        } else if (result < smooth_distance) {
            if(extendor)
                setpoint = offset + powf(result-smooth_distance, 2.0f)/(4.0f * smooth_distance);
            else
                setpoint = offset + powf(result+smooth_distance, 2.0f)/(4.0f * smooth_distance);
        } else {
            if(extendor)
                setpoint = offset;
            else
                setpoint = offset + result;
        }
        error_previous = error;
    }

    void starting(const ros::Time &time) { ROS_INFO("controller started for %s", joint_name.c_str()); }

    void stopping(const ros::Time &time) { ROS_INFO("controller stopped for %s", joint_name.c_str()); }

private:
    hardware_interface::JointHandle joint;
    double setpoint = 0;
    string joint_name;
    ros::NodeHandle n;
    ros::Subscriber jointStatus_sub;
    int jointID = -1;
    int extendor;
    float Kp = 20, Ki = 0, Kd = 80;
    float error = 0, error_previous = 0, integral_max = 360, integral_min = 0, integral = 0;
    const float smooth_distance = 50;
    const float offset = 20;
    mutex mux;
};
PLUGINLIB_EXPORT_CLASS(JointAngleController, controller_interface::ControllerBase);