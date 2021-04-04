#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

/* 获取原始的线速度和角速度，然后积分算出坐标 */
double    vel_ang_z;
double    vel_x;
double    vel_y;
double    imu_diff_time;
double    vel_diff_time;

void imu_callback(const sensor_msgs::Imu& imu)
{
  static ros::Time last_imu_time = ros::Time::now();
  static ros::Time cur_imu_time;

  cur_imu_time = ros::Time::now();
  imu_diff_time = (cur_imu_time - last_imu_time).toSec();

  if(imu.angular_velocity.z > -0.01 && imu.angular_velocity.z < 0.01)
  {
    vel_ang_z = 0;
  }
  else 
  {
    vel_ang_z = imu.angular_velocity.z;
  }
  last_imu_time = cur_imu_time;
}

void vel_callback(const geometry_msgs::TwistStamped& vel)
{
  static ros::Time last_vel_time = ros::Time::now();
  static ros::Time cur_vel_time;

  cur_vel_time = ros::Time::now();
  vel_diff_time = (cur_vel_time - last_vel_time).toSec();
  vel_x = vel.twist.linear.x;
  vel_y = vel.twist.linear.y;

  last_vel_time = cur_vel_time;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  /* 订阅原始线速度角速度 */
  ros::Subscriber sub_vel = n.subscribe("raw_vel", 50, vel_callback);
  ros::Subscriber sub_imu = n.subscribe("imu/data", 50, imu_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  //坐标积分
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vel_x * cos(th) - vel_y * sin(th)) * dt;
    double delta_y = (vel_x * sin(th) + vel_y * cos(th)) * dt;
    double delta_th = vel_ang_z * dt;
    //double delta_x = (vel_x * cos(th) - vel_y * sin(th)) * vel_diff_time;
    //double delta_y = (vel_x * sin(th) + vel_y * cos(th)) * vel_diff_time;
    //double delta_th = vel_ang_z * imu_diff_time;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = vel_ang_z;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}