
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <iostream>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <functional>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

//TF stuff
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <common_utilities/rviz_visualization.hpp>

using namespace gazebo;

class CupTablePlugin : public ModelPlugin, public rviz_visualization {

public:
    CupTablePlugin() : ModelPlugin() {

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

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&CupTablePlugin::OnUpdate, this, _1));


        this->model->SetGravityMode(0);
        ROS_WARN("hit detection ready");
    }


public:
    void OnUpdate(const common::UpdateInfo & /*_info*/) {
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
            sprintf(meshpath, "package://roboy_models/%s/meshes/CAD/%s.stl",
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
private:
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    ros::NodeHandlePtr nh;


private:
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

};

GZ_REGISTER_MODEL_PLUGIN(CupTablePlugin)