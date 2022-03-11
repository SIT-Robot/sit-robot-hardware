//
// Created by sit on 2021/4/10.
//

#include <ros/ros.h>

#include "sit_odom/OdometryNode.h"

int main(int argc,char **argv){
    ros::init(argc,argv,"odom_publisher");  //初始化里程计发布器
    ros::NodeHandle nh("~");
    OdometryNode on(nh);

    for(ros::Rate rate(30);ros::ok();rate.sleep()){
        ros::spinOnce();
        on.spinOnce();
    }
    return 0;
}
