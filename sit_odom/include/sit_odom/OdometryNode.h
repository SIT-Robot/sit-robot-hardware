//
// Created by sit on 2021/5/27.
//

#pragma once

#include "Odometry.h"
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

/**
 * 里程计ROS节点
 */
class OdometryNode{
private:

    ros::NodeHandle &nh;
    Odometry odometry;  //里程计对象

    ros::Publisher odom_pub;    //里程计发布器
    tf2_ros::TransformBroadcaster odom_broadcaster;  //里程计tf发布器

    ros::Subscriber real_speed_sub; //订阅实时速度
    ros::Subscriber imu_subscriber; //订阅实时IMU数据

public:
    explicit OdometryNode(ros::NodeHandle &nh):nh(nh){
        odometry = Odometry(true);  //初始化里程计对象，包含IMU输入
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10); //发布里程计信息

        real_speed_sub = nh.subscribe<geometry_msgs::TwistStamped>("/real_speed", 10, [this](const geometry_msgs::TwistStamped::ConstPtr &msg){
            auto stamp = msg->header.stamp;
            double vx = msg->twist.linear.x;
            double vy = msg->twist.linear.y;
            double vw = msg->twist.angular.z;
            odometry.updateSpeed(vx,vy,vw,stamp);
        }); //订阅速度信息

        imu_subscriber = nh.subscribe<sensor_msgs::Imu>("/imu_raw",10,[this](const sensor_msgs::Imu::ConstPtr& msg){
            double yaw = tf::getYaw(msg->orientation);
            odometry.updateYaw(msg->angular_velocity.z,yaw);
        }); //订阅IMU信息

    }
    
    void spinOnce(){
        //发布TF变换
        odom_broadcaster.sendTransform(getOdomTfData());
        auto odom_msg = getOdomMsg();

        //发布里程计信息
        odom_pub.publish(odom_msg);

        double x = odom_msg.pose.pose.position.x;
        double y = odom_msg.pose.pose.position.y;
        double yaw = tf::getYaw(odom_msg.pose.pose.orientation);
        ROS_INFO("odom-->x:%f y:%f yaw:%f", x,y,yaw);
    }

    /**
     * 获取里程计的tf消息
     * @return 里程计tf消息
     */
    geometry_msgs::TransformStamped getOdomTfData(){
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = odometry.getCurrentTime();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        double x,y,yaw;
        odometry.getOdometry(x,y,yaw);  //获取里程计信息

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);

        return odom_trans;
    }

    /**
     * @brief 获取里程计消息
     * 
     * @return nav_msgs::Odometry 
     */
    nav_msgs::Odometry getOdomMsg(){
        nav_msgs::Odometry odom;
        ros::Time stamp = odometry.getCurrentTime();
        double x,y,yaw;
        double vx,vy,vw;
        odometry.getOdometry(x,y,yaw);
        odometry.getCurrentTwist(vx,vy,vw);

        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;

        tf::Quaternion qut = tf::createQuaternionFromYaw(yaw);
        odom.pose.pose.orientation.x = qut.getX();
        odom.pose.pose.orientation.y = qut.getY();
        odom.pose.pose.orientation.z = qut.getZ();
        odom.pose.pose.orientation.w = qut.getW();

        //set the velocity
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vw;
        return odom;
    }
};