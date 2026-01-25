#ifndef FRAME_OF_TELEOP_HPP
#define FRAME_OF_TELEOP_HPP


#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>
#include <geometry_msgs/Transform.h>
#include <string>
#include <std_msgs/Bool.h>
#include <boost/bind/bind.hpp>




class Frame_of_teleop{

    private:
        ros::NodeHandle node_;
        ros::Subscriber sub_angular_velocity_;
        ros::Subscriber sub_translational_velocity_;
        ros::Subscriber sub_return_to_mission_;
        ros::Subscriber sub_follow_EE_;
        std::mutex lock;
        std::string arm_id_;
        bool initialized_ ;
        // Variables for storing the current pose of the control frame
        double wx_=0.0;
        double wy_=0.0;
        double wz_=0.0;
        double vx_=0.0;
        double vy_=0.0;
        double vz_=0.0;
        // 
        double translation_x_=0.0;
        double translation_y_=0.0;
        double translation_z_=0.0;
        double rotation_x_=0.0;
        double rotation_y_=0.0;
        double rotation_z_=0.0;
        double rotation_w_=1.0;
        ros::Time last_time_;
        geometry_msgs::TransformStamped transform_hand_displayed_; // transform to be broadcasted
        tf2_ros::Buffer tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr= std::make_shared<tf2_ros::TransformBroadcaster>();
        double deltat_;
        bool integrate_pos_ = false;
        bool integrate_ori_ = false;
      
        void subscribeCallbacks(){
            sub_angular_velocity_ = node_.subscribe("/"+ arm_id_ + "/target_angular_velocity", 10, &Frame_of_teleop::Callback_target_angular_velocity, this);
            sub_translational_velocity_ = node_.subscribe("/"+ arm_id_ + "/target_translational_velocity", 10, &Frame_of_teleop::Callback_target_translational_velocity, this);
            sub_return_to_mission_ = node_.subscribe<std_msgs::Bool>("/"+ arm_id_ + "/my_joint_velocity_controller/return_to_mission/", 10, &Frame_of_teleop::Callback_return_to_mission, this);
            sub_follow_EE_ = node_.subscribe<std_msgs::Bool>("/"+ arm_id_ + "/my_joint_velocity_controller/teleop_cmd_received/", 10, &Frame_of_teleop::Callback_follow_EE, this);
        }
        void Callback_target_angular_velocity(const geometry_msgs::Vector3::ConstPtr& msg){
            std::lock_guard<std::mutex> guard(lock);
            wx_ = msg->z ;
            wy_ = -msg->x ;
            wz_ = msg->y ;
            integrate_ori_ = true;
           
        }
        void Callback_target_translational_velocity(const geometry_msgs::Vector3::ConstPtr& msg){
            std::lock_guard<std::mutex> guard(lock);
            vx_ = msg->z ;
            vy_ = -msg->x ;
            vz_ = msg->y ;
            integrate_pos_ = true;
        }
        void Callback_return_to_mission(const std_msgs::Bool::ConstPtr& msg){
            if (msg->data){
                setFrameToEE();
            }
        }
        void Callback_follow_EE(const std_msgs::Bool::ConstPtr& msg){
            if (msg->data){
                setFrameToEE();
            }
            std::cout << "Follow EE command received: " << msg->data << std::endl;
        }
        void sendTransform( ){
            transform_hand_displayed_.header.frame_id = arm_id_ + "_link0";
            transform_hand_displayed_.child_frame_id = arm_id_ + "_teleop_frame";
            transform_hand_displayed_.header.stamp =ros::Time::now();
            {std::lock_guard<std::mutex> guard(lock);
            setTranslation();
            setRotation();
            }
            tf_broadcaster_ptr->sendTransform(transform_hand_displayed_);
        }
        void setTranslation(){
            transform_hand_displayed_.transform.translation.x = translation_x_;
            transform_hand_displayed_.transform.translation.y = translation_y_;
            transform_hand_displayed_.transform.translation.z = translation_z_;
        }
        void setRotation(){
            tf2::Quaternion q_rot( rotation_x_, rotation_y_, rotation_z_, rotation_w_);
            transform_hand_displayed_.transform.rotation.x = q_rot.x();
            transform_hand_displayed_.transform.rotation.y = q_rot.y();
            transform_hand_displayed_.transform.rotation.z = q_rot.z();
            transform_hand_displayed_.transform.rotation.w = q_rot.w();
        }

        void setFrameToEE(){
            std::lock_guard<std::mutex> guard(lock);
            try {
                auto T = tfBuffer_.lookupTransform(arm_id_+"_link0", arm_id_+"_EE", ros::Time(0), ros::Duration(1.0));
                translation_x_ = T.transform.translation.x;
                translation_y_ = T.transform.translation.y;
                translation_z_ = T.transform.translation.z;
                rotation_x_ = T.transform.rotation.x;
                rotation_y_ = T.transform.rotation.y;
                rotation_z_ = T.transform.rotation.z;
                rotation_w_ = T.transform.rotation.w;}
            catch (tf2::TransformException &ex) {
                ROS_ERROR("TF lookup failed: %s", ex.what());
                ros::Duration(1.0).sleep();
                return;
            }
        }
        void updateTargetPose(const ros::Duration& delta_t){
            std::lock_guard<std::mutex> guard(lock);
            if (integrate_pos_){
                // update translation
                translation_x_ +=  vx_ * delta_t.toSec();
                translation_y_ +=  vy_ * delta_t.toSec();
                translation_z_ +=  vz_ * delta_t.toSec();
                integrate_pos_ = false;
                }
                //std::cout << "Translation teleop frame: [" << translation_x_ << ", " << translation_y_ << ", " << translation_z_ << "]" << std::endl;
            if (integrate_ori_){
                // update orientation
                tf2::Quaternion q( rotation_x_, rotation_y_, rotation_z_, rotation_w_); 
                if (q.length2() < 1e-9) q.setValue(0,0,0,1);
                q.normalize();

                double omega_norm = std::sqrt(wx_ * wx_ + wy_ * wy_ + wz_ * wz_);
                if (omega_norm > 1e-9){
                    tf2::Vector3 axis( wx_, wy_, wz_ );
                    axis.normalize();
                    double angle = omega_norm*delta_t.toSec();
                    tf2::Quaternion delta_q( axis, angle );
                    q= delta_q * q;
                    q.normalize();
                }
                rotation_x_ = q.x();
                rotation_y_ = q.y();
                rotation_z_ = q.z();
                rotation_w_ = q.w();
                integrate_ori_ = false;
            }

                // set transform
            transform_hand_displayed_.transform.translation.x = translation_x_;
            transform_hand_displayed_.transform.translation.y = translation_y_;
            transform_hand_displayed_.transform.translation.z = translation_z_;
            tf2::Quaternion q_rot( rotation_x_, rotation_y_, rotation_z_, rotation_w_);
            if (q_rot.length2() < 1e-9) q_rot.setValue(0,0,0,1);
            q_rot.normalize();
            transform_hand_displayed_.transform.rotation = tf2::toMsg(q_rot);
            
        }
        
       
        public: 

        Frame_of_teleop(ros::NodeHandle& node): node_(node){
            tf_listener_ptr= std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
              if(!node_.getParam("arm_id", arm_id_)){
                ROS_ERROR("Parameter arm_id not set");
                return;
            }
            subscribeCallbacks();
        };
        ~Frame_of_teleop()= default;
        void starting(){
            // wait for TF buffer to fill
            ros::Duration(1.0).sleep();
            //if can transform set frame to EE
            try{
                setFrameToEE();
            }
            catch (tf2::TransformException &ex) {
                ROS_ERROR("TF lookup failed: %s", ex.what());
                return;
            }
        };
        void update(double delta_t){
            updateTargetPose(ros::Duration(delta_t));            
            sendTransform();

        };
   
};

#endif // FRAME_OF_TELEOP_HPP

