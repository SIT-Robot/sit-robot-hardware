#include <ros/ros.h>

#include <serial/serial.h>
#include <sit_protocol/SerialCommunicator.h>
#include <sit_protocol/ProtocolNode.h>
#include <thread>
#include <boost/format.hpp>
/**
 * 初始化串口设置
 * @param ser 串口对象
 * @param serial_port 串口号
 * @param baud_rate 波特率
 * @return 初始化是否成功
 */
bool initSerial(std::unique_ptr<serial::Serial>& ser, std::string &serial_port, uint32_t baud_rate) {

    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); //设置串口超时时间
    ROS_INFO("Try Serial port: %s", serial_port.c_str());
    ROS_INFO("Serial baud rate: %d", baud_rate);

    ser->setPort(serial_port);   //设置串口端口
    ser->setBaudrate(baud_rate); //设置串口波特率
    ser->setTimeout(timeout);    //设置串口超时时间

    try {
        ser->open();
    }
    catch (serial::IOException &e) {
        ROS_ERROR("Unable to open port %s", serial_port.c_str());
        ROS_ERROR("%s",e.what());
        return false;
    }
    if (!ser->isOpen()) {
        ser->close();
        return false;
    }
    ROS_INFO("Serial port initialized");
    return true;
}



int main(int argc, char **argv) {
    setlocale(LC_ALL, "");                       //添加语言支持
    ros::init(argc, argv, "protocol_forwarder"); //初始化节点
    ros::NodeHandle nh("~");                     //定义节点句柄

    std::map<uint8_t,std::unique_ptr<SerialCommunicator>> device_pool;  //设备池

    //初始化设备池
    for(auto& port:serial::list_ports()){
        //过滤出ttyUSB设备
        if(port.port.find("ttyUSB") != std::string::npos){
            auto ser_ptr = std::make_unique<serial::Serial>();                           //定义串口
            if (!initSerial(ser_ptr, port.port, 115200)) {//初始化串口
                //串口初始化失败
                ROS_ERROR("Serial initialize failed");
            }else{
                //初始化成功
                //开始请求地址

                auto sc = std::make_unique<SerialCommunicator>(std::move(ser_ptr));
                uint8_t addr = sc->getAddress();
                if(addr != 0){
                    //是合法的设备
                    ROS_INFO("Find device %d",addr);
                    device_pool.insert(std::pair<uint8_t,std::unique_ptr<SerialCommunicator>>(
                            addr,
                            std::move(sc)));
                }
            }
        }
    }
    if(device_pool.empty()){
        // 找不到设备
        ROS_INFO("No device!!!");
        return -1;
    }

    ProtocolNode protocolNode(device_pool, nh);

    ros::spin();
    return 0;
}