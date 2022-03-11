#pragma once

/**
 * @brief 底盘的抽象类
 * 
 */
class AChassis
{

public:

    /**
     * @brief 对底盘设置一个期望速度
     * 
     * @param vx x轴方向的期望速度
     * @param vy y轴方向上的期望速度
     * @param vw 环绕z轴方向的转速
     */
    virtual void setSpeed(double vx,double vy,double vw) = 0;

    /**
     * @brief 获取底盘当前时刻的速度
     * 
     * @param vx x轴方向的期望速度
     * @param vy y轴方向上的期望速度
     * @param vw 环绕z轴方向的转速
     */
    virtual void getSpeed(double& vx,double& vy,double& vw) = 0;
};