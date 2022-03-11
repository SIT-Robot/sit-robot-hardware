#include <ros/ros.h>
#include <sit_chassis/ThreeChassis.h>
#include <sit_chassis/ChassisNode.h>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "three_chassis"); //初始化三轮底盘节点
    ros::NodeHandle nh("~");
    ThreeChassis chassis(nh);

    ChassisNode cn(nh,&chassis);
    for (ros::Rate rate(20); ros::ok(); rate.sleep())
    {
        cn.spinOnce();
        ros::spinOnce();
    }
    return 0;
}