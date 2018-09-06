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

static const std::string OPENCV_WINDOW = "Image window";
using namespace gazebo;
using namespace cv;
using namespace Eigen;
using namespace std;

class CupTablePlugin : public ModelPlugin, public rviz_visualization {

public:
    CupTablePlugin() : ModelPlugin() {
    }
    ~CupTablePlugin(){
        cv::destroyWindow(OPENCV_WINDOW);
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
        boost::shared_ptr<ros::AsyncSpinner> spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
        spinner->start();

        camera_sub = nh->subscribe("/zed/camera/left/image_raw",1, &CupTablePlugin::leftCameraCB, this);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&CupTablePlugin::OnUpdate, this, _1));


        this->model->SetGravityMode(0);

        camMatrix = Mat(3,3,CV_32FC1,K);
        distCoeffs = Mat (1,5,CV_32FC1,D);

        detectorParams = aruco::DetectorParameters::create();
        detectorParams->cornerRefinementMaxIterations = 100;
        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_ARUCO_ORIGINAL));

        cv::namedWindow(OPENCV_WINDOW);
        ROS_WARN("cup table plugin ready");
    }

    void leftCameraCB(const sensor_msgs::Image::ConstPtr &msg){
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

public:
    void OnUpdate(const common::UpdateInfo & /*_info*/) {
        if(cv_ptr!= nullptr)
            detectAruco();
        cv::waitKey(1);

        if((ros::Time::now()-last_published).toSec()<0.01)
            return;
        last_published = ros::Time::now();
        int message_counter = 1000;
        for(auto link:model->GetLinks()) {
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
            sprintf(meshpath, "package://roboy_models/%s/meshes/CAD/%s.dae",
                    model->GetName().c_str(), link->GetName().c_str());
            mesh.mesh_resource = meshpath;
            visualization_pub.publish(mesh);
            tf::Transform trans;
            trans.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));
            trans.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
            tf_broadcaster.sendTransform(
                    tf::StampedTransform(trans, ros::Time::now(), "world", link->GetName().c_str()));
        }

    }

    void detectAruco(){
        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams, rejected);
        aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
        // draw results
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
            for(unsigned int i = 0; i < ids.size(); i++) {
                if(!arucoIDs.empty()) {
                    if (std::find(arucoIDs.begin(), arucoIDs.end(), ids[i]) == arucoIDs.end())
                        continue;
                }
                aruco::drawAxis(cv_ptr->image, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                markerLength * 0.5f);
                double theta = sqrt(pow(rvecs[i][0],2.0) + pow(rvecs[i][1],2.0) + pow(rvecs[i][2],2.0));
                Quaterniond q(rvecs[i][0]/theta,rvecs[i][0]/theta,rvecs[i][0]/theta,theta);
                q.normalize();
                tf::Transform trans;
                trans.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
                trans.setOrigin(tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]-1)); // TODO camera should have its own link, so we dont need to correct the pose here
                char str[100];
                sprintf(str,"cup_%d",ids[i]);
                tf_broadcaster.sendTransform(
                        tf::StampedTransform(trans, ros::Time::now(), "table", str));
            }
        }
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    }
private:
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    ros::NodeHandlePtr nh;
    ros::Time last_published;
    ros::Subscriber camera_sub;
    cv_bridge::CvImagePtr cv_ptr;
    Mat camMatrix, distCoeffs;
    Ptr<aruco::DetectorParameters> detectorParams;
    Ptr<aruco::Dictionary> dictionary;
    vector<int> arucoIDs;
    float markerLength = 0.07f;
    float K[9] = {687.8062263328862, 0.0, 320.5, 0.0, 687.8062263328862, 240.5, 0.0, 0.0, 1.0}, D[5] = {0,0,0,0,0};
private:
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

};

GZ_REGISTER_MODEL_PLUGIN(CupTablePlugin)