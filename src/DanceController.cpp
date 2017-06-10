#include "roboy_controller/DanceController.hpp"

DanceController::DanceController() {
    realsense_ctx = boost::shared_ptr<rs::context>(new rs::context);
    ROS_INFO("There are %d connected RealSense devices.\n", realsense_ctx->get_device_count());
    if(realsense_ctx->get_device_count() == 0) {
        ROS_ERROR("no realsense connected");
        return;
    }else {
        realsense_dev = realsense_ctx->get_device(0);
        ROS_INFO("\nUsing device 0, an %s\n     Serial number: %s\n     Firmware version: %s\n",
                 realsense_dev->get_name(), realsense_dev->get_serial(), realsense_dev->get_firmware_version());

        // Configure all streams to run at VGA resolution at 60 frames per second
        realsense_dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);

        color_intrin = realsense_dev->get_stream_intrinsics(rs::stream::color);
        float k[9] = {color_intrin.fx, 0, color_intrin.ppx, 0, color_intrin.fy, color_intrin.ppy, 0, 0, 1};
        memcpy(K,k,sizeof(float)*9);
        camMatrix = Mat(3,3,CV_32FC1,K);
        distCoeffs = Mat (1,5,CV_32FC1,color_intrin.coeffs);

        cout << camMatrix << endl;
        cout << distCoeffs << endl;

        detectorParams = aruco::DetectorParameters::create();
        detectorParams->doCornerRefinement = true;
        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_4X4_100));

        realsense_dev->start();
    }
};

bool DanceController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    // get joint name from the parameter server
    if (!n.getParam("joint_name", joint_name)) {
        ROS_ERROR("Could not find joint name");
        myStatus = ControllerState::UNDEFINED;
        return false;
    }
    n.getParam("id", statusMsg.id);
    ROS_INFO("ForceController %d for %s initialized", statusMsg.id, joint_name.c_str());
    joint = hw->getHandle(joint_name);  // throws on failure
    trajectory_srv = n.advertiseService("/roboy/trajectory_" + joint_name, &DanceController::trajectoryPreprocess,
                                        this);
    steer_sub = n.subscribe("/roboy/steer", 1000, &DanceController::steer, this);
    status_pub = n.advertise<roboy_communication_middleware::ControllerState>("/roboy/status_" + joint_name, 1000);
    trajectory_pub = n.advertise<std_msgs::Float32>("/roboy/trajectory_" + joint_name + "/eff", 1000);
    myStatus = ControllerState::INITIALIZED;
    statusMsg.state = myStatus;
    status_pub.publish(statusMsg);
    return true;
}

void DanceController::update(const ros::Time &time, const ros::Duration &period) {
    realsense_dev->wait_for_frames();
    const uint8_t *color_frame = reinterpret_cast<const uint8_t *>(realsense_dev->get_frame_data(rs::stream::color));
    Mat image = Mat(480, 640, CV_8UC3, (uint8_t*)color_frame), imageCopy;
    cv::cvtColor(image, image, CV_RGB2BGR);
    image.copyTo(imageCopy);

    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    vector< Vec3d > rvecs, tvecs;

    // detect markers and estimate pose
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);

    // draw results
    if(ids.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        for(unsigned int i = 0; i < ids.size(); i++) {
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                            markerLength * 0.5f);
        }

    }

    imshow("imageCopy", imageCopy);
    waitKey(1);
    ROS_INFO_THROTTLE(1, "aruco ids visible: %d", (int)ids.size());


}

void DanceController::steer(const roboy_communication_middleware::Steer::ConstPtr &msg) {
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

bool DanceController::trajectoryPreprocess(roboy_communication_middleware::SetTrajectory::Request &req,
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
PLUGINLIB_EXPORT_CLASS(DanceController, controller_interface::ControllerBase);