#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "./AChassis.h"
class ChassisNode
{
private:
    AChassis *pChassis; //底盘指针
    ros::NodeHandle &nh;                //ROS节点句柄

    ros::Publisher speed_publisher;               //发布底盘真实速度
    ros::Publisher real_no_stamp_speed_publisher; //发布不包含时间戳的真实速度反馈
    ros::Subscriber speed_subscriber;             //订阅底盘期望速度

public:
    explicit ChassisNode(ros::NodeHandle &nh, AChassis *pChassis)
        : nh(nh), pChassis(pChassis)
    {
        speed_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,&ChassisNode::onSpeedSubHandler,this); //速度订阅

        speed_publisher = nh.advertise<geometry_msgs::TwistStamped>("/real_speed", 10); //真实速度发布

        real_no_stamp_speed_publisher = nh.advertise<geometry_msgs::Twist>("/real_speed_no_stamp", 10); //发布不包含时间戳的真实速度反馈
    }

    // 期望速度订阅回调
    void onSpeedSubHandler(const geometry_msgs::Twist::ConstPtr &ptr){
        pChassis->setSpeed(ptr->linear.x,
                           ptr->linear.y,
                           ptr->angular.z); //设置速度
    }

    // 发布速度信息
    void publishSpeedMsg(double vx, double vy, double vw)
    {
        geometry_msgs::TwistStamped speedMsg;
        speedMsg.header.frame_id = "/base_footprint";
        speedMsg.header.stamp = ros::Time::now();
        speedMsg.twist.linear.x = vx;
        speedMsg.twist.linear.y = vy;
        speedMsg.twist.angular.z = vw;

        // 发布带有时间戳信息的真实速度
        speed_publisher.publish(speedMsg);

        // 发布不包含时间戳的真实速度反馈
        real_no_stamp_speed_publisher.publish(speedMsg.twist);
    }

    // 轮询速度
    void spinOnce()
    {
        double vx,vy,vw;
        pChassis->getSpeed(vx,vy,vw);
        publishSpeedMsg(vx,vy,vw);
    }
};
