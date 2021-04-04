#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>

#include "seriallib/SerialPort.h"

typedef struct {
    float   vx;
    float   vy;
    float   vth;
}cmd_vel_msg;

/* 全局参数 */
bool param_use_debug_imu;
bool param_use_debug_cmd;
bool param_use_debug_vel;

/* IMU发布消息 */
sensor_msgs::Imu            pub_msg_imu;
sensor_msgs::MagneticField  pub_msg_mag;

/* 位置数据发布 */
geometry_msgs::TwistStamped pub_msg_pose;

/* 串口变量 */
std::string param_port_path;
int         param_baudrate;
SerialPort  *serial_rxtx;

/* 消息类型 */
typedef enum {
    ROS_DATA_GYRO   = 1,        /* 陀螺仪 */
    ROS_DATA_ACCEL  = 2,        /* 加速计 */
    ROS_DATA_MEGY   = 3,        /* 磁力计 */
    ROS_DATA_EULER  = 4,        /* 欧拉角 */
    ROS_DATA_VEL    = 5,        /* 线速度 */
    ROS_CMD_VEL     = 100,      /* 速度控制 */
}msg_type;

/* 传感器数据 */
typedef struct {
    float       x;
    float       y;
    float       z;
}sensor_msg;

void parse_callback(msg_type type, void *msg)
{   
    switch(type)
    {
    case ROS_DATA_GYRO:
        pub_msg_imu.angular_velocity.x = ((sensor_msg*)msg)->x;
        pub_msg_imu.angular_velocity.y = ((sensor_msg*)msg)->y;
        pub_msg_imu.angular_velocity.z = ((sensor_msg*)msg)->z;
        //ROS_INFO("GYRO x:%f y:%f z:%f\r\n", ((sensor_msg*)msg)->x, ((sensor_msg*)msg)->y, ((sensor_msg*)msg)->z);
    break;
    case ROS_DATA_ACCEL:
        pub_msg_imu.linear_acceleration.x = ((sensor_msg*)msg)->x;
        pub_msg_imu.linear_acceleration.y = ((sensor_msg*)msg)->y;
        pub_msg_imu.linear_acceleration.z = ((sensor_msg*)msg)->z;
        //ROS_INFO("ACCEL x:%f y:%f z:%f\r\n", ((sensor_msg*)msg)->x, ((sensor_msg*)msg)->y, ((sensor_msg*)msg)->z);
    break;
    case ROS_DATA_MEGY:
        pub_msg_mag.magnetic_field.x = ((sensor_msg*)msg)->x;
        pub_msg_mag.magnetic_field.y = ((sensor_msg*)msg)->y;
        pub_msg_mag.magnetic_field.z = ((sensor_msg*)msg)->z;
        //ROS_INFO("MEGY x:%f y:%f z:%f\r\n", ((sensor_msg*)msg)->x, ((sensor_msg*)msg)->y, ((sensor_msg*)msg)->z);
    break;
    case ROS_DATA_VEL:
        pub_msg_pose.twist.linear.x = ((sensor_msg*)msg)->x;
        pub_msg_pose.twist.linear.y = ((sensor_msg*)msg)->y;
        pub_msg_pose.twist.linear.z = ((sensor_msg*)msg)->z;
        //ROS_INFO("VEL x:%f y:%f z:%f\r\n", ((sensor_msg*)msg)->x, ((sensor_msg*)msg)->y, ((sensor_msg*)msg)->z);
    break;
    
    }
}

int send_pack(msg_type type, void *msg, int len)
{
    unsigned char start[5] = {0XAA, 0XBC};
    unsigned char *ptr = (unsigned char*)msg;
    int  ret = 0;
    
    start[2] = type;
    start[3] = (unsigned char)len;
    for (int i = 0; i < len; i++)
    {
        start[4] ^= ptr[i];
    }
    
    ret = serial_rxtx->writeBuffer(start, 4);
    ret += serial_rxtx->writeBuffer((unsigned char *)msg, len);
    ret += serial_rxtx->writeBuffer(&start[4], 1);

    return ret;
}

static unsigned char uart_sample_get_char(void)
{
    unsigned char   ch;
    int             result;

    if (1 != (result = serial_rxtx->readbyte(&ch)))
    {
        ROS_ERROR("serial_rx waitfordata ERROR, result:%d", result);
        delete serial_rxtx;
        serial_rxtx = NULL;
        exit(0);
    }
    return ch;
}

static void rx_thread_func(void)
{
    unsigned char ch;
    unsigned char data[300];
    unsigned char crc  = 0;
    int step = 0;
    int i    = 0;

    while (1)
    {
        ch = (unsigned char)uart_sample_get_char();
        if(step == 0 && 0xaa == ch)     /* 协议头 */
        {
            step++;
            continue;
        }
        else if(step == 1 && 0xbc == ch)/* 协议头 */
        {
            step++;
            continue;
        }
        else if(step == 2)              /* 数据类型 */
        {
            step++;
            data[i++] = ch;
            continue;
        }
        else if(step == 3)              /* 数据长度 */
        {
            step++;
            data[i++] = ch;
            continue;
        }
        else if(step == 4 && i-2 < data[1])/* 接收数据 */
        {
            data[i++] = ch;
            continue;
        }
        else if(step == 4 && i-2 == data[1])/* 校验数据 */
        {
            data[i] = ch;
            crc = data[2];
            for (int j = 1; j < data[1]; j++)
            {
                crc ^= data[j+2];
            }
            /* 校验成功 */
            if (crc == data[i])
            {
                parse_callback((msg_type)data[0], (void *)&data[2]);
            }
            /* 校验失败 */
            else
            {
                for(int x=0; x <= i; x++)
                {
                    printf("%x ",data[x]);
                }
                printf("\r\n");
                printf("step:%d len:%d crc:%x\r\n", step, i, crc);
                ROS_INFO("ros base serial rx crc error!");
            }
            crc = 0;
            step=0;
            i=0;
            continue;
        }
        else {
            crc = 0;
            step=0;
            i=0;
            continue;
        }
    }
}


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& message)
{
    sensor_msg msg;
    msg.x = message->linear.x;
    msg.y = message->linear.y;
    msg.z = message->angular.z;
    send_pack(ROS_CMD_VEL, &msg, sizeof(msg));
    ROS_INFO("vx:%f vy:%f vz:%f vaz:%f",message->linear.x, message->linear.y, message->linear.z, message->angular.z);
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "base_control_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param<bool>("debug_imu", param_use_debug_imu, false);
    nh.param<bool>("debug_imu", param_use_debug_cmd, false);
    nh.param<bool>("debug_imu", param_use_debug_vel, false);
    nh.param<std::string>("port", param_port_path, "/dev/ttyUSB0");
    nh.param<int>("baudrate", param_baudrate, 115200);

    /* 发布imu、数度数据 */
    ros::Publisher pub_raw_pose = n.advertise<geometry_msgs::TwistStamped>("raw_vel", 100);
    ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
    ros::Publisher pub_mag = n.advertise<sensor_msgs::MagneticField>("imu/mag", 100);
    /* 订阅小车控制主题 */
    ros::Subscriber sub_cmd_vel = n.subscribe<geometry_msgs::Twist>("cmd_vel", 100, cmd_vel_callback);
    /* 创建接受数据线程 */
    serial_rxtx = new SerialPort();
    int i =10;
    while (i--)
    {
        if (-1 == serial_rxtx->Open(param_port_path.c_str(), param_baudrate))
        {
            ROS_ERROR("%s open error\r\n", param_port_path.c_str());
            sleep(5);
            continue;
        }
        break;
    }
    boost::thread rx_tread(rx_thread_func);

    ros::Rate r(100);
    sensor_msg msg = {3.0, 2.0, 1.0};
    while (ros::ok())
    {
        /* code for l1.0oop body */
        /* 发布线速度消息 */
        pub_msg_pose.header.stamp = ros::Time::now();
        pub_raw_pose.publish(pub_msg_pose);

        /* 发布IMU数据 */
        pub_msg_imu.header.stamp = ros::Time::now();
        pub_msg_imu.header.frame_id = "imu_link";
        pub_imu.publish(pub_msg_imu);
        pub_msg_mag.header.stamp = ros::Time::now();
        pub_msg_mag.header.frame_id = "imu_link";
        pub_mag.publish(pub_msg_mag);

        ros::spinOnce();
        r.sleep();

    }
    
    delete serial_rxtx;

    return 0;
}