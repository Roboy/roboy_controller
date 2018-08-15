
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


namespace gazebo
{
	class HitDetectionPlugin : public ModelPlugin
	{

		public: HitDetectionPlugin() : ModelPlugin()
		{

              //printf("Hello World!\n");
		}

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
		{

			std::cerr << "\nThe Hit Detection Plugin plugin is attached to model[" <<
			_model->GetName() << "]\n";

			if(_model->GetJointCount() == 0)
			{
				std::cerr << "Invalid joint count, Hit Detection plugin not loaded\n";
				return;
			}

			this->model = _model;

			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "HitDetection",
					ros::init_options::NoSigintHandler);
			}
			this->rosNode.reset(new ros::NodeHandle());
			hit_detection_pub = rosNode->advertise<std_msgs::String>("xylophone/hitdetection", 1);
			boost::shared_ptr<ros::AsyncSpinner> spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
			spinner->start();
			//ros::Rate loop_rate(10);


			/*ros::SubscribeOptions so =
			ros::SubscribeOptions::create<std_msgs::Float32>(
				"/" + this->model->GetName() + "/HitDetection",
				1,
				boost::bind(&HitDetectionPlugin::OnRosMsg, this, _1),
				ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			this->rosQueueThread =
			std::thread(std::bind(&HitDetectionPlugin::QueueThread, this));*/

			//std::cerr << "Get Joints";
			physics::Joint_V joints = model->GetJoints();
			for (auto joint : joints) {
				prevVelocity[joint->GetName()] = 0.0;
				prevTime[joint->GetName()] = 0.0;
			}

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&HitDetectionPlugin::OnUpdate, this, _1));


		}



		public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			tf::TransformBroadcaster br;
			tf::Transform transform;

			//update xylophonepose based on darttracker
			setXylophonePose();

			std::string model_name = model->GetName();
			physics::Link_V links = model->GetLinks();
			//ROS_INFO_STREAM("Model \"" << model_name << "\" links:");
			// List all links of the model
			for (auto link : links) {
				std::string link_name = link->GetName();
				auto linkPose = link->GetWorldPose().pos;
				transform.setOrigin( tf::Vector3(linkPose.x, linkPose.y, linkPose.z) );
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "xylophone", link_name));

				//ROS_INFO_STREAM("  " << link_name<< " "<<link->GetWorldPose());
			}

			//ROS_INFO_STREAM("Model \"" << model_name << "\" joints:");
			physics::Joint_V joints = model->GetJoints();
			// List all joints of the model
			for (auto joint : joints) {
				std::string joint_name = joint->GetName();
				//std_msgs::String msg;
				//msg.data = joint_name;
				// //std::cerr << msg.data;
				// std::cerr<<joint_name;
				//std::cerr<<"Publishing";
				//hit_detection_pub.publish(msg);
				//if(joint_name == "C_0")
				//{
				auto accel = GetAcceleration(joint);
				if (accel > 300)
				{
					while (ros::ok())
					{
						std_msgs::String msg;
						msg.data = joint_name;

						hit_detection_pub.publish(msg);
						ros::spinOnce();
					//loop_rate.sleep();
					}
				}
				//
				//}
				//ROS_INFO_STREAM("  " << joint_name);//<<" "<<joint->GetForce(0));
				//ROS_INFO_STREAM(joint->GetForceTorque(0));

			}
			//std::chrono::milliseconds timespan(10000);
    		//std::this_thread::sleep_for(timespan);


		}

		public: double GetAcceleration(physics::JointPtr joint)
		{
			//std::cerr << "current_vel";
			//double current_ve = prevVelocity[joint->GetName()];
			//std::cerr<< current_ve << "\n";
			//std::cerr << "current_time";
			//double current_tim = prevTime[joint->GetName()];
			//std::cerr<< current_tim << "\n";
			//std::cerr << "current_time";
			double current_time = this->model->GetWorld()->GetSimTime().Double();
			//std::cerr<< current_time << "\n";
			//std::cerr << "current_vel";
			double current_vel = joint->GetVelocity(0);
			//std::cerr<< current_vel << "\n";
			//std::cerr << "accel";
			double accel = (current_vel-prevVelocity[joint->GetName()])/(current_time-prevTime[joint->GetName()]);
			//std::cerr<< accel << "\n";
			prevVelocity[joint->GetName()] = current_vel;
			prevTime[joint->GetName()] = current_time;
			return accel;;

		}



		private: event::ConnectionPtr updateConnection;
		private: physics::ModelPtr model;
		private: std::map<std::string,double> prevVelocity;
		private: std::map<std::string,double> prevTime;
		private: std::unique_ptr<ros::NodeHandle> rosNode;
		private: ros::Publisher hit_detection_pub;


	private:

		/// gets and sets coords for xylophone based on values from DartTracker publisher
		void setXylophonePose()
		{
			//blocking fct: waits for something / anything to get published on tf topic to work on reliable data later on
			//for now, random frames from roboy's model chosen
			//returns bool -> true if received, false if timeout
			listener.waitForTransform("xylophone", "world", ros::Time(), ros::Duration(1.0));

			std::cout << "Getting Transform for xylophone \n";
			tf::StampedTransform xylophone_world_pos;
			try {
				//todo which target frame should be used? frame order should be: world->xylophone->key
				listener.lookupTransform("world", "xylophone", ros::Time(0), xylophone_world_pos);
			}
			catch (tf::LookupException ex) {
				ROS_WARN_THROTTLE(1, "%s", ex.what());
			}
			//only takes position when defining key pose....
			const ignition::math::Vector3d *pos= new ignition::math::Vector3d(xylophone_world_pos.getOrigin().getX(), xylophone_world_pos.getOrigin().getY(), xylophone_world_pos.getOrigin().getZ());
			const ignition::math::Quaterniond *rot = new ignition::math::Quaterniond();
			const ignition::math::Pose3d pose = ignition::math::Pose3d(*pos, *rot);

			//listener ONLY TRANSPORTS TRANSFORM!! -> no rotation publisher possible (except for if we use a *ros topic*
			/*
			pose.rot[0] = xylophone_world_pos.getRotation().getZ();
			pose.rot[1] = xylophone_world_pos.getOrigin().getZ();
			pose.rot[2] = xylophone_world_pos.getOrigin().getZ();
			pose.rot[3] = xylophone_world_pos.getOrigin().getZ();
			*/

			this->model->SetWorldPose(pose);
		}

		//subscribes to tf broadcaster (?)
		tf::TransformListener listener;
	};

	GZ_REGISTER_MODEL_PLUGIN(HitDetectionPlugin)
}
