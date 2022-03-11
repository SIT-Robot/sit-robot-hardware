#include <ros/ros.h>
#include <sit_protocol_msgs/CmdDataFrame.h>
#include <sit_protocol_msgs/RequestDataFrame.h>
#include <sit_chassis_imu/ImuData.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @brief 大疆RM C板自带IMU数据获取
 * 
 * IMU坐标系如下
 * y
 * |
 * |
 * z_____x
 * 坐标轴位于板子中心
 *  ______________
 * |              |
 * |              |
 * |              |
 * |  ROBOMASTER  |
 * |              |
 * |              |
 * |              |
 * ---------------- 
 */

#include <tf2/convert.h>
class ImuNode
{
private:

    ros::ServiceClient _request_client;  //IMU数据请求客户端
    ros::Publisher _imuMsgPublisher; //imu消息发布器

    /**
     * @brief 发布IMU数据
     */
    void publishImuData(ImuData imuData)
    {
        sensor_msgs::Imu imuMsg;
        imuMsg.header.frame_id = "imu_link";
        imuMsg.header.stamp = ros::Time::now();

        tf2::Quaternion qtn;
        qtn.setRPY(
            imuData.euler.r,
            imuData.euler.p,
            imuData.euler.y
        );

        imuMsg.orientation.x = qtn.getX();
        imuMsg.orientation.y = qtn.getY();
        imuMsg.orientation.z = qtn.getZ();
        imuMsg.orientation.w = qtn.getW();

        // ROS_INFO("[r: %f, p: %f, y: %f]",imuData.euler.r,imuData.euler.p,imuData.euler.y);
        //ROS_INFO("[%f,%f,%f,%f]",qtn.getX(),qtn.getY(),qtn.getZ(),qtn.getW());

        imuMsg.angular_velocity.x = imuData.angular_vel.x;
        imuMsg.angular_velocity.y = imuData.angular_vel.y;
        imuMsg.angular_velocity.z = imuData.angular_vel.z;

        imuMsg.linear_acceleration.x = imuData.linear_acc.x;
        imuMsg.linear_acceleration.y = imuData.linear_acc.y;
        imuMsg.linear_acceleration.z = imuData.linear_acc.z;

        _imuMsgPublisher.publish(imuMsg);
    }

    /**
     * @brief 请求IMU数据
     * 
     * @return ImuData 返回IMU数据
     */
    bool requestImu(ImuData& imuData){
        sit_protocol_msgs::RequestDataFrame req;
        req.request.request.cmd = 0x04;
        req.request.request.address = 0x01;
        req.request.waitCmd = 0x40; //等待响应0x8A
        req.request.request.address = 0x01;
        if(_request_client.call(req)){
            ROS_INFO("IMU请求数据成功");
            memcpy(&imuData, req.response.response.data.data(), 52); //52字节
            return true;
        }else{
            ROS_INFO("IMU请求数据失败");
            return false;
        }
    }


public:
    explicit ImuNode(ros::NodeHandle &nh)
    {
        _request_client = nh.serviceClient<sit_protocol_msgs::RequestDataFrame>("/protocol_forwarder/RequestDataFrame");
        _imuMsgPublisher = nh.advertise<sensor_msgs::Imu>("/imu_raw", 10);
    }

    /**
     * @brief 回旋函数，用于在主循环中不断地请求IMU数据
     * 
     */
    void spinOnce()
    {
        ImuData imuData{};
        if(requestImu(imuData)){
            publishImuData(imuData);
        }
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");  
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh("~");

    ImuNode imuNode(nh);
    for (ros::Rate rate(10); ros::ok(); rate.sleep())
    {
        imuNode.spinOnce(); //imu节点回旋，不断请求imu数据
        ros::spinOnce();    //ros回旋函数
    }
    return 0;
}