//
// Created by sit on 2021/5/27.
//

#pragma once

#include <ctime>
#include <ros/ros.h>
/**
 * 里程计类
 * 该类的使用流程
 * 构造该类(可根据是否具有外挂IMU选择yawInput开关参数)
 * 不断执行updateSpeed来进行速度积分
 * 如果有IMU则根据updateYaw实时更新相对准确的偏航角信息
 * 
 */
class Odometry{
private:
    double distX;       //x方向的位移
    double distY;       //y方向的位移
    double yaw;      //偏航角

    double current_vx; //当前x方向的线速度
    double current_vy; //当前y方向的线速度
    double current_vw;  //当前z方向的角速度

    ros::Time current_time; //当前时间
    ros::Time last_time;    //上一个数据帧的时间
    bool yawInput;      //是否有直接的yaw偏航角输入
public:
    /**
     * 初始化一个里程计类，该类输入较高频率的瞬时速度，可通过积分得到里程计数据
     * @param yawInput 是否存在直接的yaw偏航角输入，默认值为false,将通过积分得到yaw角
     */
    explicit Odometry(bool yawInput = false)
    :yawInput(yawInput){

    }

    /**
     * 获取里程计数据
     * @param x 引用传递x，接收x方向的位移
     * @param y 引用传递y，接收y方向的位移
     * @param yaw_ 引用传递yaw_，接收z方向的角位移
     * @return 返回该里程计数据的时刻
     */

    ros::Time getOdometry(double& x,double& y,double& yaw_){
        x = distX;
        y = distY;
        yaw_ = yaw;
        return current_time;
    }

    /**
     * 获取当前时刻的瞬时速度
     * @param vx 引用传递vx，接收x方向的线速度
     * @param vy 引用传递vy，接收y方向的线速度
     * @param vw 引用传递vw，接收z方向的自转角速度
     * @return as
     */
    ros::Time getCurrentTwist(double& vx,double& vy,double& vw){
        vx = current_vx;
        vy = current_vy;
        vw = current_vw;
        return current_time;
    }

    /**
     * 获取里程计的时间戳
     * @return
     */
    ros::Time getCurrentTime(){
        return current_time;
    }

    /**
     * 积分的速度输入,该函数需要以较高的频率不断采样瞬时速度来对里程计数据进行积分
     * @param vx x轴方向上的瞬时线速度
     * @param vy y轴方向上的瞬时线速度
     * @param vw z轴方向上的瞬时自转角速度
     * @param stamp 该速度的时间戳
     */
    void updateSpeed(double vx,double vy,double vw,ros::Time stamp){
        current_time = stamp;
        current_vx = vx;
        current_vy = vy;

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(yaw) - vy * sin(yaw)) * dt;
        double delta_y = (vx * sin(yaw) + vy * cos(yaw)) * dt;
        double delta_th = vw * dt;

        distX += delta_x;
        distY += delta_y;

        //如果不存在yaw输入，那就积分计算yaw
        if(!yawInput){
            current_vw = vw;
            yaw += delta_th;
        }

        last_time = current_time;
    }

    /**
     * 直接设置一个准确的偏航角,一旦调用过该函数，偏航角将启动外部输入，停止积分
     */
    void updateYaw(double vw,double yaw_){
        static bool first = true;   //是否为第一次
        static double firstTh = 0;  //第一次的yaw角度输入

        this->current_vw = vw;
        if(first){
            yawInput = true;    //表示启用外部yaw角度输入，禁止积分得出yaw角度
            first = false;      //不是第一次了
            firstTh = yaw_;     //保存第一次的数据
        }
        yaw = yaw_ - firstTh;    //得出里程计的偏航值
    }

};