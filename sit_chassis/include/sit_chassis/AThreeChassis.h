#pragma once

#include <ros/ros.h>
#include "./AChassis.h"

/**
 * @brief 抽象的三轮底盘
 * 
 */
class AThreeChassis : public AChassis
{
private:
    double L; //底盘运动半径L

public:
    /**
     * @brief 设置底盘的运动半径
     * 
     * @param radius 底盘的运动半径
     */
    void setChassisRadius(double radius)
    {
        this->L = radius;
        ROS_INFO("Set Chassis Radius: %f", radius);
    }

protected:
    /**
     * @brief 设置车轮的期望线速度，具体通过怎样的方式下发速度指令，继续由子类实现，
     * 车轮线速度单位为 m/s
     * 
     * @param v1 1号车轮的线速度
     * @param v2 2号车轮的线速度
     * @param v3 3号车轮的线速度
     */
    virtual void setWheelSpeed(double v1, double v2, double v3) = 0;

    /**
     * @brief 获取当前时刻车轮速度
     * 
     * @param v1 1号车轮的线速度
     * @param v2 2号车轮的线速度
     * @param v3 3号车轮的线速度
     */
    virtual void getWheelSpeed(double& v1,double& v2,double& v3) = 0;


public:
    //实现父类纯虚函数
    void setSpeed(double vx, double vy, double vw) override
    {
        const double M_PI_6 = M_PI / 6;
        //ROS_INFO("Speed:[%f, %f, %f]", vx, vy, vw);
        //需要将速度进行分解到三个轮子上
        double vb = -vy + vw * L;
        double vl = -vx * cos(M_PI_6) + vy * sin(M_PI_6) + vw * L;
        double vr = vx * cos(M_PI_6) + vy * sin(M_PI_6) + vw * L;

        //设置轮速
        setWheelSpeed(vb, vr, vl);
    }

    void getSpeed(double& vx,double& vy,double& vw) override
    {
        //需要获取三个轮子的轮速，合成出轮子速度

        //获取三个轮子速度
        double v1,v2,v3;
        getWheelSpeed(v1,v2,v3);

        //速度合成
        vx = (v2 - v3) * (sqrt(3) / 3);
        vy = (-2 * v1+ v2+ v3) / 3;
        vw = (v1 + v2 + v3) / (3 * L);
    }
    explicit AThreeChassis(double radius) : L(radius) {}
};
