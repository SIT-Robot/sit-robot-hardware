#pragma once
#include "./AThreeChassis.h"

#include <ros/ros.h>
#include <sit_protocol_msgs/CmdDataFrame.h>
#include <sit_protocol_msgs/RequestDataFrame.h>

/**
 * @brief 具体实现的三轮底盘
 * 
 */

class ThreeChassis : public AThreeChassis
{
private:
    ros::NodeHandle &nh;            //ros节点句柄
    ros::ServiceClient _request_client;  //速度请求客户端

    double K{}; //轮子线速度与下发的速度指令的比例系数

    /**
     * @brief 设置电机原始转速
     * 
     * @param v1 1号电机原始转速
     * @param v2 2号电机原始转速
     * @param v3 3号电机原始转速
     */
    bool setPrimitiveSpeed(int16_t v1, int16_t v2, int16_t v3)
    {
        sit_protocol_msgs::RequestDataFrame request_data;

        request_data.request.request.cmd = 0x03;
        request_data.request.request.address = 0x01;    //下发速度指令
        request_data.request.waitCmd = 0x30;
        request_data.request.request.data = {
            uint8_t(v1 >> 8),
            uint8_t(v1 & 0xff),
            uint8_t(v2 >> 8),
            uint8_t(v2 & 0xff),
            uint8_t(v3 >> 8),
            uint8_t(v3 & 0xff),
        };
        //待发送的数据包
        if(_request_client.call(request_data)){
            //下发成功
            ROS_INFO("Set speed [%d %d %d] successful", v1, v2, v3);
            return true;
        }else{
            //下发失败
            ROS_INFO("Set speed [%d %d %d] failed", v1, v2, v3);
            return false;
        }
    }

    /**
     * @brief 获取原始车轮转速
     * 
     * @param v1 1号电机原始转速
     * @param v2 2号电机原始转速
     * @param v3 3号电机原始转速
     */
    bool getPrimitiveSpeed(int16_t& v1, int16_t& v2, int16_t& v3)
    {
        sit_protocol_msgs::RequestDataFrame request_data;
        request_data.request.request.cmd = 0x02;
        request_data.request.request.address = 0x01;    //下发速度指令
        request_data.request.waitCmd = 0x20;
        if(_request_client.call(request_data)){
            auto data = request_data.response.response.data;
            v1 = int16_t(data[0] << 8 | data[1]);
            v2 = int16_t(data[2] << 8 | data[3]);
            v3 = int16_t(data[4] << 8 | data[5]);
            ROS_INFO("Receive speed [%d %d %d]", v1, v2, v3);
            return true;
        }else{
            ROS_INFO("无法获取底盘速度");
            v1 = 0;
            v2 = 0;
            v3 = 0;
            return false;
        }
    }

public:
    explicit ThreeChassis(ros::NodeHandle &nh, double radius = 0.1543) : AThreeChassis(radius), nh(nh)
    {
        nh.param("radius",radius,radius);   //获取底盘半径参数
        if(nh.hasParam("k")){   //如果存在比例系数，那就设置比例系数
            double k;
            nh.getParam("k",k);
            this->setK(k);
        }else{
            this->setK(3000);
        }

        _request_client = nh.serviceClient<sit_protocol_msgs::RequestDataFrame>("/protocol_forwarder/RequestDataFrame");
    }

    /**
     * @brief 标定车轮系数
     * 真实线速度 = K * 下发的速度指令值
     * 
     * @param k 
     */
    void setK(double k)
    {
        this->K = k;
        ROS_INFO("Set wheel K: %f", k);
    }

    
    void setWheelSpeed(double v1, double v2, double v3) override
    {
        setPrimitiveSpeed(int(v1 * K), int(v2 * K), int(v3 * K));
    }

    void getWheelSpeed(double& v1, double& v2, double& v3) override
    {
        int16_t pv1,pv2,pv3;
        getPrimitiveSpeed(pv1,pv2,pv3); //获取原始转速

        //设置速度
        v1 = pv1 / K;
        v2 = pv2 / K;
        v3 = pv3 / K;
    }

};
