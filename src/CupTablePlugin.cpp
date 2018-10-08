#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <boost/algorithm/string.hpp>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <common_utilities/rviz_visualization.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <vector>

static const std::string ZED_LEFT = "zed left", ZED_RIGHT = "zed right";
using namespace gazebo;
using namespace cv;
using namespace Eigen;
using namespace std;

class CupTablePlugin : public ModelPlugin, public rviz_visualization {

public:
    CupTablePlugin() : ModelPlugin() {
    }

    ~CupTablePlugin() {
        cv::destroyWindow(ZED_LEFT);
        cv::destroyWindow(ZED_RIGHT);
    }

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) {
        this->model = _model;

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "CupTablePlugin",
                      ros::init_options::NoSigintHandler);
        }

        nh.reset(new ros::NodeHandle());
        boost::shared_ptr<ros::AsyncSpinner> spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(2));
        spinner->start();

        left_zed_camera_sub = nh->subscribe("/zed/camera/left/image_raw_color", 1, &CupTablePlugin::leftCameraCB, this);
//        right_zed_camera_sub = nh->subscribe("/zed/camera/right/image_raw_color", 1, &CupTablePlugin::rightCameraCB, this);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&CupTablePlugin::OnUpdate, this, _1));


        this->model->SetGravityMode(0);

        camMatrix = Mat(3, 3, CV_32FC1, K);
        distCoeffs = Mat(1, 5, CV_32FC1, D);

        detectorParams = aruco::DetectorParameters::create();
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detectorParams->cornerRefinementMaxIterations = 100;
        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_ARUCO_ORIGINAL));

//        cv::namedWindow(ZED_LEFT);
//        cv::namedWindow(ZED_RIGHT);
//        moveWindow(ZED_LEFT, 0, 0);
//        moveWindow(ZED_RIGHT, 700, 0);
        ROS_WARN("cup table plugin ready");
    }

    void leftCameraCB(const sensor_msgs::Image::ConstPtr &msg) {
        try {
            zed_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void rightCameraCB(const sensor_msgs::Image::ConstPtr &msg) {
        try {
            zed_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

public:
    void OnUpdate(const common::UpdateInfo & /*_info*/) {
        if ((ros::Time::now() - last_published).toSec() < 0.1)
            return;
        last_published = ros::Time::now();
        int message_counter = 1000;
        tf::StampedTransform trans;
        try {
            tf_listener.lookupTransform("world", "head", ros::Time(0), trans);
        }
        catch (tf::TransformException ex) {
            ROS_WARN_THROTTLE(1, "%s", ex.what());
        }
        math::Pose pose_head;
        pose_head.pos.x = trans.getOrigin().x();
        pose_head.pos.y = trans.getOrigin().y();
        pose_head.pos.z = trans.getOrigin().z();
        pose_head.rot.x = trans.getRotation().x();
        pose_head.rot.y = trans.getRotation().y();
        pose_head.rot.z = trans.getRotation().z();
        pose_head.rot.w = trans.getRotation().w();
        for (auto link:model->GetLinks()) {
            string link_name = link->GetName();
            if ( link_name == "head") {
                link->SetWorldPose(pose_head);
                tf::Transform trans;
                trans.setRotation(tf::Quaternion(0,0,0,1));
                trans.setOrigin(tf::Vector3(-0.061, -0.128, 0.073));
                tf_broadcaster.sendTransform(
                        tf::StampedTransform(trans, ros::Time::now(), "head", "zed_camera_right_lense"));
                trans.setOrigin(tf::Vector3(0.059, -0.128, 0.073));
                tf_broadcaster.sendTransform(
                        tf::StampedTransform(trans, ros::Time::now(), "head", "zed_camera_left_lense"));
            }
            math::Pose pose = link->GetWorldPose();
            pose.rot.Normalize();

            visualization_msgs::Marker mesh;
            mesh.header.frame_id = "world";
            mesh.ns = "cup_table";
            mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
            mesh.color.r = 1.0f;
            mesh.color.g = 1.0f;
            mesh.color.b = 1.0f;
            mesh.color.a = 1;
            mesh.scale.x = 0.001;
            mesh.scale.y = 0.001;
            mesh.scale.z = 0.001;
            mesh.lifetime = ros::Duration(0);
            mesh.header.stamp = ros::Time::now();
            mesh.action = visualization_msgs::Marker::ADD;
            mesh.id = message_counter++;

            mesh.pose.position.x = pose.pos.x;
            mesh.pose.position.y = pose.pos.y;
            mesh.pose.position.z = pose.pos.z;
            mesh.pose.orientation.x = pose.rot.x;
            mesh.pose.orientation.y = pose.rot.y;
            mesh.pose.orientation.z = pose.rot.z;
            mesh.pose.orientation.w = pose.rot.w;
            char meshpath[200];
            sprintf(meshpath, "package://roboy_models/%s/meshes/CAD/%s.stl",
                    model->GetName().c_str(), link->GetName().c_str());
            mesh.mesh_resource = meshpath;
            visualization_pub.publish(mesh);
            if (link_name != "head") { // head is published by caspr
                tf::Transform trans;
                trans.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));
                trans.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
                tf_broadcaster.sendTransform(
                        tf::StampedTransform(trans, ros::Time::now(), "world", link->GetName().c_str()));
            }
        }
        if (zed_left_ptr != nullptr) {
            detectAruco();
            cv::waitKey(1);
        }
    }

    void detectAruco() {
        {
            vector<int> ids;
            vector<vector<Point2f> > corners, rejected;
            vector<Vec3d> rvecs, tvecs;

            // detect markers and estimate pose
            aruco::detectMarkers(zed_left_ptr->image, dictionary, corners, ids, detectorParams, rejected);
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
            // draw results
            if (ids.size() > 0) {
                aruco::drawDetectedMarkers(zed_left_ptr->image, corners, ids);
                for (unsigned int i = 0; i < ids.size(); i++) {
                    if (!arucoIDs.empty()) {
                        if (std::find(arucoIDs.begin(), arucoIDs.end(), ids[i]) == arucoIDs.end())
                            continue;
                    }
                    aruco::drawAxis(zed_left_ptr->image, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                    markerLength * 0.5f);
//                    double theta = sqrt(pow(rvecs[i][0], 2.0) + pow(rvecs[i][1], 2.0) + pow(rvecs[i][2], 2.0));
//                    Quaterniond q(rvecs[i][0] / theta, rvecs[i][1] / theta, rvecs[i][2] / theta, theta);
//                    q.normalize();
                    tf::Transform trans;
                    trans.setRotation(tf::Quaternion(0,0,0,1));
                    Quaterniond q_cv_coordinates_to_gazebo(0, 0, 0.7071068, 0.7071068);
                    Vector3d pos(tvecs[i][0], tvecs[i][1],tvecs[i][2]);
                    pos = q_cv_coordinates_to_gazebo.matrix()*pos;
                    trans.setOrigin(tf::Vector3(pos[0],-pos[1],pos[2]));
                    char str[100];
                    sprintf(str, "zed_left_aruco_%d", ids[i]);
                    tf_broadcaster.sendTransform(
                            tf::StampedTransform(trans, ros::Time::now(), "zed_camera_left_lense", str));
                }
            }
            cv::imshow(ZED_LEFT, zed_left_ptr->image);
        }
//        {
//            vector<int> ids;
//            vector<vector<Point2f> > corners, rejected;
//            vector<Vec3d> rvecs, tvecs;
//
//            // detect markers and estimate pose
//            aruco::detectMarkers(zed_right_ptr->image, dictionary, corners, ids, detectorParams, rejected);
//            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
//            // draw results
//            if (ids.size() > 0) {
//                aruco::drawDetectedMarkers(zed_right_ptr->image, corners, ids);
//                for (unsigned int i = 0; i < ids.size(); i++) {
//                    if (!arucoIDs.empty()) {
//                        if (std::find(arucoIDs.begin(), arucoIDs.end(), ids[i]) == arucoIDs.end())
//                            continue;
//                    }
//                    aruco::drawAxis(zed_right_ptr->image, camMatrix, distCoeffs, rvecs[i], tvecs[i],
//                                    markerLength * 0.5f);
////                    double theta = sqrt(pow(rvecs[i][0], 2.0) + pow(rvecs[i][1], 2.0) + pow(rvecs[i][2], 2.0));
////                    Quaterniond q(rvecs[i][0] / theta, rvecs[i][1] / theta, rvecs[i][2] / theta, theta);
////                    q.normalize();
//                    tf::Transform trans;
//                    trans.setRotation(tf::Quaternion(0,0,0,1));
//                    Quaterniond q_cv_coordinates_to_gazebo(0, 0, 0.7071068, 0.7071068);
//                    Vector3d pos(tvecs[i][0], tvecs[i][1],tvecs[i][2]);
//                    pos = q_cv_coordinates_to_gazebo.matrix()*pos;
//                    trans.setOrigin(tf::Vector3(pos[0],-pos[1],pos[2]));
//                    char str[100];
//                    sprintf(str, "cup_zed_right_%d", ids[i]);
//                    tf_broadcaster.sendTransform(
//                            tf::StampedTransform(trans, ros::Time::now(), "zed_camera_right_lense", str));
//                }
//            }
//            cv::imshow(ZED_RIGHT, zed_right_ptr->image);
//        }
    }

private:
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    ros::NodeHandlePtr nh;
    ros::Time last_published;
    ros::Subscriber left_zed_camera_sub, right_zed_camera_sub;
    cv_bridge::CvImagePtr zed_left_ptr, zed_right_ptr;
    Mat camMatrix, distCoeffs;
    Ptr<aruco::DetectorParameters> detectorParams;
    Ptr<aruco::Dictionary> dictionary;
    vector<int> arucoIDs;
    float markerLength = 0.07f;
    float K[9] = {687.8062263328862, 0.0, 320.5, 0.0, 687.8062263328862, 240.5, 0.0, 0.0, 1.0}, D[5] = {0, 0, 0, 0, 0};
private:
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

};

GZ_REGISTER_MODEL_PLUGIN(CupTablePlugin)