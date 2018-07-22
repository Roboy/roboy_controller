#include <controller_interface/controller.h>
#include <hardware_interface/actuator_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <roboy_communication_middleware/ControllerState.h>
#include <roboy_communication_middleware/Steer.h>
#include <roboy_communication_middleware/SetTrajectory.h>
#include <common_utilities/CommonDefinitions.h>
#include <ecl/geometry.hpp>
#include <map>

using namespace std;

class MuscController : public controller_interface::Controller<hardware_interface::ActuatorCommandInterface> {
public:
    MuscController() {

    };

    bool init(hardware_interface::ActuatorCommandInterface *hw, ros::NodeHandle &n) {
        // get joint name from the parameter server
        if (!n.getParam("joint_name", actuator_name)) {
            ROS_ERROR("Could not find joint name");
            myStatus = ControllerState::UNDEFINED;
            return false;
        }
        n.getParam("id", statusMsg.id);
        ROS_INFO("MuscController %d for %s initialized", statusMsg.id, actuator_name.c_str());
        actuator = hw->getHandle(actuator_name);  // throws on failure
        trajectory_srv = n.advertiseService("/roboy/trajectory_" + actuator_name, &MuscController::trajectoryPreprocess,
                                            this);
        steer_sub = n.subscribe("/roboy/steer", 1000, &MuscController::steer, this);
        status_pub = n.advertise<roboy_communication_middleware::ControllerState>("/roboy/status_" + actuator_name,
                                                                                  1000);
        trajectory_pub = n.advertise<std_msgs::Float32>("/roboy/trajectory_" + actuator_name + "/pos", 1000);
        myStatus = ControllerState::INITIALIZED;
//        // wait for GUI subscriber
//        while (status_pub.getNumSubscribers() == 0)
//            ROS_INFO_THROTTLE(1, "MuscController %s waiting for subscriber", actuator_name.c_str());
        statusMsg.state = myStatus;
        status_pub.publish(statusMsg);
        return true;
    }

    void update(const ros::Time &time, const ros::Duration &period) {
        double pos = actuator.getPosition();
        pos_msg.data = pos;
        trajectory_pub.publish(pos_msg);

        if (steered == PLAY_TRAJECTORY) {
            ros::Duration d = time - time_old;
            time_old = time;
            dt += d.nsec * 1e-6f;
            if (dt < trajectory_duration[id]) {
                setpoint = cubic[id](dt);
            } else {
                myStatus = TRAJECTORY_DONE;
                statusMsg.state = myStatus;
                steered = STOP_TRAJECTORY;
                status_pub.publish(statusMsg);
            }
            actuator.setCommand(setpoint);
        } else {
            // hold the position
            actuator.setCommand(pos);
        }
    }

    void steer(const roboy_communication_middleware::Steer::ConstPtr &msg) {
        if (cubic.find(msg->id) != cubic.end()) {
            id = msg->id;
            switch (msg->steeringCommand) {
                case STOP_TRAJECTORY:
                    dt = 0;
                    steered = STOP_TRAJECTORY;
                    myStatus = TRAJECTORY_READY;
                    ROS_INFO("%s received steering STOP", actuator_name.c_str());
                    break;
                case PLAY_TRAJECTORY:
                    time_old = ros::Time::now();
                    steered = PLAY_TRAJECTORY;
                    myStatus = TRAJECTORY_PLAYING;
                    ROS_INFO("%s received steering PLAY", actuator_name.c_str());
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
        } else {
            ROS_WARN("%s received steering for id %d, but a trajectory with this id is NOT available, yet",
                     actuator_name.c_str(), msg->id);
        }
    }

    void starting(const ros::Time &time) { ROS_INFO("controller started for %s", actuator_name.c_str()); }

    void stopping(const ros::Time &time) { ROS_INFO("controller stopped for %s", actuator_name.c_str()); }

private:
    hardware_interface::ActuatorHandle actuator;
    double setpoint = 0;
    string actuator_name;
    ros::NodeHandle n;
    ros::Subscriber steer_sub;
    ros::Publisher status_pub, trajectory_pub;
    ros::ServiceServer trajectory_srv;
    int id;
    map<int, ecl::CubicSpline> cubic;
    map<int, double> trajectory_duration;
    int8_t myStatus = UNDEFINED;
    int8_t steered = STOP_TRAJECTORY;
    std_msgs::Float32 pos_msg;
    float dt = 0;
    ros::Time time_old;
    roboy_communication_middleware::ControllerState statusMsg;

    bool trajectoryPreprocess(roboy_communication_middleware::SetTrajectory::Request &req,
                              roboy_communication_middleware::SetTrajectory::Response &res) {
        steered = STOP_TRAJECTORY;
        myStatus = PREPROCESS_TRAJECTORY;
        statusMsg.state = myStatus;
        status_pub.publish(statusMsg);

        if (req.trajectory.waypoints.size() > 0 && req.trajectory.samplerate > 0) {
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
            trajectory_duration[req.trajectory.id] = req.trajectory.waypoints.size() * req.trajectory.samplerate;
            cubic[req.trajectory.id] = ecl::CubicSpline::Natural(x, y);
            ROS_INFO("%s new trajectory [id: %d, elements: %d] at sampleRate %f, duration %f",
                     actuator_name.c_str(), req.trajectory.id, (int) req.trajectory.waypoints.size(),
                     req.trajectory.samplerate,
                     trajectory_duration[req.trajectory.id]);
            myStatus = ControllerState::TRAJECTORY_READY;
            statusMsg.state = myStatus;
            status_pub.publish(statusMsg);
            return true;
        } else {
            if (req.trajectory.waypoints.size() <= 0)
                ROS_ERROR("%s new trajectory but it is empty!", actuator_name.c_str());
            if (req.trajectory.samplerate <= 0)
                ROS_ERROR("%s new trajectory but sample rate is %f!", actuator_name.c_str(), req.trajectory.samplerate);
            myStatus = ControllerState::TRAJECTORY_FAILED;
            statusMsg.state = myStatus;
            status_pub.publish(statusMsg);
            return false;
        }
    }
};
PLUGINLIB_EXPORT_CLASS(MuscController, controller_interface::ControllerBase);

