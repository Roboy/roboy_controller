#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <roboy_communication_middleware/ControllerState.h>
#include <roboy_communication_middleware/Steer.h>
#include <roboy_communication_middleware/SetTrajectory.h>
#include <common_utilities/CommonDefinitions.h>
#include <ecl/geometry.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <librealsense/rs.hpp>

using namespace std;
using namespace cv;

//CameraParameters aruco_camera_parameters(cameraMatrix,distortions,Size(640,480));

class DanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    DanceController();

    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);

    void update(const ros::Time &time, const ros::Duration &period);

    void steer(const roboy_communication_middleware::Steer::ConstPtr &msg);

    void starting(const ros::Time &time) { ROS_INFO("controller started for %s", joint_name.c_str()); };

    void stopping(const ros::Time &time) { ROS_INFO("controller stopped for %s", joint_name.c_str()); };

    bool trajectoryPreprocess(roboy_communication_middleware::SetTrajectory::Request &req,
                              roboy_communication_middleware::SetTrajectory::Response &res);

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
    boost::shared_ptr<rs::context> realsense_ctx;
    rs::device * realsense_dev;
    rs::intrinsics color_intrin;
    Mat camMatrix, distCoeffs;
    Ptr<aruco::DetectorParameters> detectorParams;
    Ptr<aruco::Dictionary> dictionary;
    float markerLength = 0.058f;
    float K[9];
};
