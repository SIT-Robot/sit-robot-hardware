//
// Created by sit on 2021/4/25.
//
#pragma once

#include <serial/serial.h>
#include "DataFrameParser.h"
#include <thread>
class SerialCommunicator{

private:
    std::unique_ptr<serial::Serial> ser;
    DataFrameParser dataFrameParser;

    std::vector<uint8_t> readAvailableBytes(){
        int len = int(ser->available());
        std::vector<uint8_t> bytes;
        bytes.reserve(len);
        ser->read(bytes,len);
        return bytes;
    }

public:
    explicit SerialCommunicator(std::unique_ptr<serial::Serial> ser):ser(std::move(ser)){}

    void close(){
        ser->close();
    }

    /**
     * 获取地址,若获得
     * @return
     */
    uint8_t getAddress(){
        CmdDataFrame req{
            .address = 0xFF,    //广播地址
            .cmd = 0x01,
        };
        CmdDataFrame resp{
            .cmd = 0x10
        };
        if(requestFrame(req,resp)){
            //请求成功
            return resp.data[0];
        }else{
            //请求超时
            return 0x00;
        }
    }

    /**
     * 发送数据帧
     * @param frame
     */
    void sendFrame(CmdDataFrame& frame){
        std::vector<uint8_t> bytes = frame.generateBytes();
        try{
            ser->write(bytes.data(),int(bytes.size()));
        }catch (serial::SerialException &e){
            ROS_ERROR("%s",e.what());
            ros::shutdown();
        }
    }

    /**
     * @brief 实现了请求响应机制的通信
     *
     * @param req_frame 请求的数据帧
     * @param resp_cmd 响应的数据帧的命令cmd数据项
     * @param timeout 超时时间
     * @return CmdDataFrame 响应的数据帧
     */
    bool requestFrame(CmdDataFrame req_frame,
                      CmdDataFrame& resp_frame,
                      uint16_t timeout = 40){

//        ROS_INFO("%s",req_frame.toByteString().c_str());
        ros::Time before_time = ros::Time::now();
        CmdDataFrame frame_resp;    //响应帧
        do{
            do{
                ros::Duration duration = ros::Time::now() - before_time;
                if(duration > ros::Duration(double(timeout)/1000)){
                    //超时
                    ROS_INFO("请求指令 %d 超时",req_frame.cmd);
                    return false;
                }
                sendFrame(req_frame);  //发送请求帧
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            } while (!receivable());
            frame_resp = receive();
        } while (frame_resp.cmd != resp_frame.cmd);
        resp_frame.data = frame_resp.data;  //数据域覆盖
        auto duration = ros::Time::now() - before_time;
        ROS_INFO("请求指令 %d 成功，用时：%d ms",req_frame.cmd,int(duration.toSec() * 1000));
        return true;
    }

    /**
     * 是否能够接收数据帧
     * @return 是否能够
     */
    bool receivable(){
        std::vector<uint8_t> bytes = readAvailableBytes();
        dataFrameParser.pushBytes(bytes.data(),int(bytes.size()));
        return !dataFrameParser.empty();
    }

    /**
     * 接收一帧数据
     * @return 一帧数据
     */
    CmdDataFrame receive(){
        auto frame =  dataFrameParser.popFrame();
        return frame;
    }
};
