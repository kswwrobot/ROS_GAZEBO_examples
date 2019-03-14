#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cmath>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <string>
#include <vector>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include <thread>
#include "std_msgs/Int32MultiArray.h"

namespace gazebo {

    int left_wheel_speed = 0;
    int right_wheel_speed = 0;        

    class RobotCarPlugin : public ModelPlugin {

        private: 
            transport::SubscriberPtr wheelSubscriber;
            
        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

        // Gazebo初始化後載入plugin，會進入Load
        public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/ ) {                                              
                                   
            //這一段是用來使ROS持續的接收 在 '"/" + this->model->GetName() + "/vel_cmd"'裡面的消息，收到的消息的時候呼叫OnRosMsg控制速度
                                   
            this -> model = _model;

            // Create our node for communication
            gazebo::transport::NodePtr node1(new gazebo::transport::Node());
            node1 -> Init();

            gazebo::transport::NodePtr node2(new gazebo::transport::Node());
            node2 -> Init();                                   

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
              int argc = 0;
              char **argv = NULL;
              ros::init(argc, argv, "gazebo_client",
                  ros::init_options::NoSigintHandler);
            }
            
            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
              ros::SubscribeOptions::create<std_msgs::Int32MultiArray>(
                  "/" + this->model->GetName() + "/vel_cmd",
                  1,
                  boost::bind(&RobotCarPlugin::OnRosMsg, this, _1),
                  ros::VoidPtr(), &this->rosQueue);
            
            this->rosSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
              std::thread(std::bind(&RobotCarPlugin::QueueThread, this));

        }
        
        //控制輪子的速度 this->model指向gazebo世界裡面的機器人 ::robot_car_left_wheel_joint是joint的名字
        public: void SetVelocity(const std::vector<std::int32_t> &_vel)
        {            
            this -> model -> GetJoint(this -> model -> GetName() + "::robot_car_left_wheel_joint") -> SetVelocity(0, _vel[0]);
            this -> model -> GetJoint(this -> model -> GetName() + "::robot_car_right_wheel_joint") -> SetVelocity(0, _vel[1]);
        }

        
        public: void OnRosMsg(const std_msgs::Int32MultiArray::ConstPtr &_msg)
        {
            this->SetVelocity(_msg->data);
        }

        // brief ROS helper function that processes messages
        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }        

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(RobotCarPlugin)
}
