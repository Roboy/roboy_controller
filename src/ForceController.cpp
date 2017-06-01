#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <roboy_communication_middleware/ControllerState.h>
#include <roboy_communication_middleware/Steer.h>
#include <roboy_communication_middleware/SetTrajectory.h>
#include <common_utilities/CommonDefinitions.h>
#include <ecl/geometry.hpp>

using namespace std;

class ForceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    ForceController() {

    };

    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
        // get joint name from the parameter server
        if (!n.getParam("joint_name", joint_name)) {
            ROS_ERROR("Could not find joint name");
            myStatus = ControllerState::UNDEFINED;
            return false;
        }
        n.getParam("id", statusMsg.id);
        ROS_INFO("ForceController %d for %s initialized", statusMsg.id, joint_name.c_str());
        joint = hw->getHandle(joint_name);  // throws on failure
        trajectory_srv = n.advertiseService("/roboy/trajectory_" + joint_name, &ForceController::trajectoryPreprocess,
                                            this);
        steer_sub = n.subscribe("/roboy/steer", 1000, &ForceController::steer, this);
        status_pub = n.advertise<roboy_communication_middleware::ControllerState>("/roboy/status_" + joint_name, 1000);
        trajectory_pub = n.advertise<std_msgs::Float32>("/roboy/trajectory_" + joint_name + "/eff", 1000);
        myStatus = ControllerState::INITIALIZED;
        statusMsg.state = myStatus;
        status_pub.publish(statusMsg);
        return true;
    }

    void update(const ros::Time &time, const ros::Duration &period) {
        double eff = joint.getEffort();
        eff_msg.data = eff;
        trajectory_pub.publish(eff_msg);

        if (steered == PLAY_TRAJECTORY) {
            dt += period.nsec * 1e-6f;
            if (dt < trajectory_duration) {
                setpoint = cubic(dt);
            } else {
                myStatus = TRAJECTORY_DONE;
                statusMsg.state = myStatus;
                steered = STOP_TRAJECTORY;
                status_pub.publish(statusMsg);
            }
            joint.setCommand(setpoint);
        }
    }

    void steer(const roboy_communication_middleware::Steer::ConstPtr &msg) {
        switch (msg->steeringCommand) {
            case STOP_TRAJECTORY:
                dt = 0;
                steered = STOP_TRAJECTORY;
                myStatus = TRAJECTORY_READY;
                ROS_INFO("%s received steering STOP", joint_name.c_str());
                break;
            case PLAY_TRAJECTORY:
                steered = PLAY_TRAJECTORY;
                myStatus = TRAJECTORY_PLAYING;
                ROS_INFO("%s received steering PLAY", joint_name.c_str());
                break;
            case PAUSE_TRAJECTORY:
                if (steered == PAUSE_TRAJECTORY) {
                    steered = PLAY_TRAJECTORY;
                    myStatus = TRAJECTORY_PLAYING;
                } else {
                    steered = PAUSE_TRAJECTORY;
                    myStatus = TRAJECTORY_READY;
                }
        }
        statusMsg.state = myStatus;
        status_pub.publish(statusMsg);
    }

    void starting(const ros::Time &time) { ROS_INFO("controller started for %s", joint_name.c_str()); }

    void stopping(const ros::Time &time) { ROS_INFO("controller stopped for %s", joint_name.c_str()); }

private:
    hardware_interface::JointHandle joint;
    double setpoint = 0;
    string joint_name;
    ros::NodeHandle n;
    ros::Subscriber steer_sub;
    ros::Publisher status_pub, trajectory_pub;
    ros::ServiceServer trajectory_srv;
    ecl::CubicSpline cubic;
    double trajectory_duration = 0;
    int8_t myStatus = UNDEFINED;
    int8_t steered = STOP_TRAJECTORY;
    std_msgs::Float32 eff_msg;
    float dt = 0;
    roboy_communication_middleware::ControllerState statusMsg;

    bool trajectoryPreprocess(roboy_communication_middleware::SetTrajectory::Request &req,
                              roboy_communication_middleware::SetTrajectory::Response &res) {
        steered = STOP_TRAJECTORY;
        myStatus = PREPROCESS_TRAJECTORY;
        statusMsg.state = myStatus;
        status_pub.publish(statusMsg);

        trajectory_duration = req.trajectory.waypoints.size() * req.trajectory.samplerate;
        ROS_INFO("New trajectory [%d elements] at sampleRate %f, duration %f",
                 (int) req.trajectory.waypoints.size(), req.trajectory.samplerate, trajectory_duration);
        if (!req.trajectory.waypoints.empty()) {
            ecl::Array<double> x(req.trajectory.waypoints.size()), y(req.trajectory.waypoints.size());
            for (uint i = 0; i < req.trajectory.waypoints.size(); i++) {
                x[i] = ((double) i * req.trajectory.samplerate);
                if (req.trajectory.waypoints[i] != FLT_MAX) {
                    y[i] = (req.trajectory.waypoints[i]);
                    cout << req.trajectory.waypoints[i] << " ";
                } else {
                    y[i] = 0.0;
                    cout << req.trajectory.waypoints[i] << " ";
                }
            }
            cout << endl;
            cubic = ecl::CubicSpline::Natural(x, y);
            myStatus = ControllerState::TRAJECTORY_READY;
            statusMsg.state = myStatus;
            status_pub.publish(statusMsg);
            dt = 0;
            return true;
        } else {
            myStatus = ControllerState::TRAJECTORY_FAILED;
            statusMsg.state = myStatus;
            status_pub.publish(statusMsg);
            dt = 0;
            trajectory_duration = 0;
            return false;
        }
    }
};
PLUGINLIB_EXPORT_CLASS(ForceController, controller_interface::ControllerBase);