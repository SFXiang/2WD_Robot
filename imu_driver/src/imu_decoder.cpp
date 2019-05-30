/*
 * ros driver node for JY901 imu
 * 444051989@qq.com
 * 2018.11.22
 */

#define Pi 3.14159265359
#define Grad2Rad  3.141592/180.0

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include "sensor_msgs/Imu.h"

#include "JY901.h"

using namespace std;

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 		stcAngle;
struct SMag 		stcMag;
struct SDStatus 	stcDStatus;
struct SPress 		stcPress;
struct SLonLat 		stcLonLat;
struct SGPSV 		stcGPSV;


//convert serial data to jy901 data
void CopeSerialData(std::string str_in)
{
    unsigned int str_length = str_in.size();
    static unsigned char chrTemp[2000];
    static unsigned char ucRxCnt = 0;
    static unsigned int usRxLength = 0;

    memcpy(chrTemp,str_in.data(),str_length);
    usRxLength += str_length;
    while (usRxLength >= 11)
    {
        if (chrTemp[0] != 0x55)
        {
            usRxLength--;
            memcpy(&chrTemp[0],&chrTemp[1],usRxLength);
            continue;
        }
        switch(chrTemp[1])
        {
            case 0x50:	memcpy(&stcTime,&chrTemp[2],8);break;
            case 0x51:	memcpy(&stcAcc,&chrTemp[2],8);break;
            case 0x52:	memcpy(&stcGyro,&chrTemp[2],8);break;
            case 0x53:	memcpy(&stcAngle,&chrTemp[2],8);break;
            case 0x54:	memcpy(&stcMag,&chrTemp[2],8);break;
            case 0x55:	memcpy(&stcDStatus,&chrTemp[2],8);break;
            case 0x56:	memcpy(&stcPress,&chrTemp[2],8);break;
            case 0x57:	memcpy(&stcLonLat,&chrTemp[2],8);break;
            case 0x58:	memcpy(&stcGPSV,&chrTemp[2],8);break;
        }
        usRxLength -= 11;
        memcpy(&chrTemp[0],&chrTemp[11],usRxLength);
    }
}

//EulerToQuaternion, euler in rad
template<typename T>
vector<T> EulerToQuaternion(T roll, T pitch, T yaw)
{
    vector<T> q;
    double x, y, z, w;
    double a = roll/2.0;
    double b = pitch/2.0;
    double g = yaw/2.0;
    w = cos(a)*cos(b)*cos(g) + sin(a)*sin(b)*sin(g);
    x = sin(a)*cos(b)*cos(g) - cos(a)*sin(b)*sin(g);
    y = cos(a)*sin(b)*cos(g) + sin(a)*cos(b)*sin(g);
    z = cos(a)*cos(b)*sin(g) - sin(a)*sin(b)*cos(g);
    q.push_back(w);
    q.push_back(x);
    q.push_back(y);
    q.push_back(z);
    return q;
}

//QuaternionToEuler, euler in rad
template<typename T>
vector<T> QuaternionToEuler(vector<T> q)
{
    vector<T> e;
    T roll = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
    T pitch = asin(2*(q[0]*q[2]-q[1]*q[3]));
    T yaw = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    e.push_back(roll);
    e.push_back(pitch);
    e.push_back(yaw);
    return e;
}


int main (int argc, char** argv)
{
    //param
    serial::Serial serial_port;
    std::string port;
    int baudrate;
    int looprate;

    //ros init
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    //get param from launch file
    pnh.param<int>("baudrate", baudrate, 115200);
    pnh.param<std::string>("port", port, "/dev/ttyUSB0");
    pnh.param<int>("looprate", looprate, 100);

    pnh.getParam("baudrate",baudrate);
    pnh.getParam("port",port);
    pnh.getParam("looprate", looprate);

    //ros pub and sub
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1000);

    try
    {
        serial_port.setPort(port);
        serial_port.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(to);
        serial_port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port ");
        return -1;
    }

    //check if serial port is open
    if(serial_port.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    //set looprate
    ros::Rate loop_rate(looprate);
    while(ros::ok())
    {
        if(serial_port.available()){
            //convert serial string to JY901 data
            CopeSerialData(serial_port.read(serial_port.available()));

            vector<double> quaternion = EulerToQuaternion(
                    stcAngle.Angle[0]/32768*Pi,
                    stcAngle.Angle[1]/32768*Pi,
                    stcAngle.Angle[2]/32768*Pi);

            tf::Quaternion qt = tf::createQuaternionFromRPY((float)stcAngle.Angle[0]/32768*Pi, (float)stcAngle.Angle[1]/32768*Pi, (float)stcAngle.Angle[2]/32768*Pi);

            //vector<double> euler = QuaternionToEuler(quaternion);
            //ROS_INFO("%.8f\t%.8f\t%.8f", euler[0],euler[1],euler[2]);

            //imu sensor msg pub
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu_link";
            //imu_msg.orientation.w = quaternion[0];
            //imu_msg.orientation.x = quaternion[1];
            //imu_msg.orientation.y = quaternion[2];
            //imu_msg.orientation.z = quaternion[3];
            imu_msg.orientation.w = qt[0];
            imu_msg.orientation.x = qt[1];
            imu_msg.orientation.y = qt[2];
            imu_msg.orientation.z = qt[3];
            imu_msg.linear_acceleration.x = (float)stcAcc.a[0]/32768*16.0*9.8;
            imu_msg.linear_acceleration.y = (float)stcAcc.a[1]/32768*16.0*9.8;
            imu_msg.linear_acceleration.z = (float)stcAcc.a[2]/32768*16.0*9.8;
            imu_msg.angular_velocity.x = ((float)stcGyro.w[0]/32768*2000)/180.0*Pi;
            imu_msg.angular_velocity.y = ((float)stcGyro.w[1]/32768*2000)/180.0*Pi;
            imu_msg.angular_velocity.z = ((float)stcGyro.w[2]/32768*2000)/180.0*Pi;
            imu_msg.orientation_covariance[0] = 999999;
            imu_msg.orientation_covariance[4] = 9999999;
            imu_msg.orientation_covariance[8] = 999999;
            imu_msg.angular_velocity_covariance[0] = 9999;
            imu_msg.angular_velocity_covariance[4] = 9999;
            imu_msg.angular_velocity_covariance[8] = 0.02;
            imu_msg.linear_acceleration_covariance[0] = 0.2;
            imu_msg.linear_acceleration_covariance[4] = 0.2;
            imu_msg.linear_acceleration_covariance[8] = 0.2;
            imu_pub.publish(imu_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
