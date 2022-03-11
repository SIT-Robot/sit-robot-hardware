#pragma once

#include <ros/ros.h>
#include <sit_protocol_msgs/CmdDataFrame.h>
#include <sit_protocol/SerialCommunicator.h>
#include <sit_protocol_msgs/RequestDataFrame.h>
/**
 * @brief 基于ROS请求响应机制实现机器人的串口通讯从独占设备变为共享设备，并实现了RPC服务器

 */

class ProtocolNode
{
private:
    std::map<uint8_t,std::unique_ptr<SerialCommunicator>>& device_pool;
    ros::ServiceServer server;

private:

    /**
     * @brief 发起一个数据帧请求
     * 
     * @param request 请求
     * @param response 响应
     */
    bool request_data(sit_protocol_msgs::RequestDataFrame::Request& request,
                      sit_protocol_msgs::RequestDataFrame::Response& response)
    {
        static int failed_count = 0;
        CmdDataFrame req_data{
                .address = request.request.address,
                .cmd = request.request.cmd,
                .data = request.request.data,
        };

        CmdDataFrame resp_data{
            .cmd = request.waitCmd
        };

        if(!device_pool.count(req_data.address)){
            //若设备不存在
            ROS_INFO("设备地址: %d 不存在",req_data.address);
            return false;
        }

        //设备存在
        bool success = device_pool[req_data.address]->requestFrame(req_data,
                                                                   resp_data,
                                                                   request.timeout);

        if(!success){
            //不成功

            // 连续超时3次以上就关闭该节点
            if(++failed_count > 3){
                ros::shutdown();
            }
            return false;
        }
        //成功
        failed_count = 0;
        response.response.cmd = resp_data.cmd;
        response.response.data = resp_data.data;
        return true;
    }

public:
    explicit ProtocolNode(std::map<uint8_t,std::unique_ptr<SerialCommunicator>>& device_pool, ros::NodeHandle &nh)
        : device_pool(device_pool)
    {
        server = nh.advertiseService("RequestDataFrame", &ProtocolNode::request_data, this);
    }
};
