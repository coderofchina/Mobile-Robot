#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include<unistd.h>

#include <fstream>   
#include <geometry_msgs/TwistStamped.h>  
#include <nav_msgs/Odometry.h>    
#include "tf/LinearMath/Matrix3x3.h"    
#include "geometry_msgs/Quaternion.h"    
#include <sensor_msgs/LaserScan.h>

#include <signal.h>
#include <tf/transform_listener.h>
#include <string.h>
using namespace std;

ros::Publisher cmdVelPub;
  void shutdown(int sig)
  {
  cmdVelPub.publish(geometry_msgs::Twist());
  ROS_INFO("odom_out_and_back.cpp ended!");
  ros::shutdown();
  }

void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;    
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));    
    double yaw, pitch, roll;    
    mat.getEulerYPR(yaw, pitch, roll);

  //How fast will we update the robot's movement?
  double rate = 20;
  int count = 0;//Loop through the two legs of the trip

  std::string topic = "/cmd_vel";
  ros::NodeHandle node;
  ros::NodeHandle cmdh;
  ros::Publisher cmdVelPub= cmdh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);;
  cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 1);

  //Set the equivalent ROS rate variable
  ros::Rate loopRate(rate);
  geometry_msgs::Twist speed; // 控制信号载体 Twist message
  signal(SIGINT, shutdown);
  ROS_INFO("The robot starts moving!");

  //Set the forward linear speed to 0.2 meters per second 线速度
  float linear_speed = 0.2;

  //Set the travel distance to 1.0 meters  移动距离
  float goal_distance = 1.0;


  //Set the rotation speed to 0.5 radians per second 角速度
  float angular_speed = 0.5;

  //Set the rotation angle to Pi radians (180 degrees) 角位移
  double goal_angle = M_PI;

  //Set the angular tolerance in degrees converted to radians
  double angular_tolerance = 2.5*M_PI/180; //角度转换成弧度:deg*PI/180

  tf::TransformListener listener;
  tf::StampedTransform transform;

  //Find out if the robot uses /base_link or /base_footprint
  std::string odom_frame = "/odom";
  std::string base_frame;
  try
  {
    listener.waitForTransform(odom_frame, "/base_footprint", ros::Time(), ros::Duration(2.0) );
    base_frame = "/base_footprint";
  }
  catch (tf::TransformException & ex)
  {
      try
      {
        listener.waitForTransform(odom_frame, "/base_link", ros::Time(), ros::Duration(2.0) );
        base_frame = "/base_link";
      }
      catch (tf::TransformException ex)
      {
          ROS_INFO("Cannot find transform between /odom and /base_link or /base_footprint");
          cmdVelPub.publish(geometry_msgs::Twist());
          ros::shutdown();
      }
  }
  //Loop once for each leg of the trip
  int label=2;
  while(ros::ok())
  { 
    ROS_INFO("go straight...");
    speed.linear.x = linear_speed; // 设置线速度，正为前进，负为后退
    //Get the starting position values
    listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
    float x_start = transform.getOrigin().x();
    float y_start = transform.getOrigin().y();
    // Keep track of the distance traveled
    float distance = 0;
    while( (distance < goal_distance) && (ros::ok()) && label!=0)
    {
         //Publish the Twist message and sleep 1 cycle
         cmdVelPub.publish(speed);
         loopRate.sleep();
         listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
         //Get the current position
         float x = transform.getOrigin().x();
         float y = transform.getOrigin().y();
         //Compute the Euclidean distance from the start
         distance = sqrt(pow((x - x_start), 2) +  pow((y - y_start), 2));
    }
    //Stop the robot before the rotation
    cmdVelPub.publish(geometry_msgs::Twist());
    ros::Duration(1).sleep(); // sleep for  a second
    ROS_INFO("rotation...");
    //Now rotate left roughly 180 degrees
    speed.linear.x = 0;
    //Set the angular speed
    speed.angular.z = angular_speed; // 设置角速度，正为左转，负为右转

    //yaw是围绕Y轴旋转，也叫偏航角
    //Track the last angle measured
    double last_angle = fabs(tf::getYaw(transform.getRotation()));
    //Track how far we have turned
    double turn_angle = 0;
    while( (fabs(turn_angle + angular_tolerance) < M_PI) && (ros::ok()) && label!=0 )
    {
        //Publish the Twist message and sleep 1 cycle
        cmdVelPub.publish(speed);
        loopRate.sleep();
        // Get the current rotation
        listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
        //C++: abs()求得是正数的绝对值,fabs()求得是浮点数的绝对值;python:abs(x),参数可以是：负数、正数、浮点数或者长整形
        double rotation = fabs(tf::getYaw(transform.getRotation()));

        //Compute the amount of rotation since the last loop
        double delta_angle = fabs(rotation - last_angle);

        //Add to the running total
        turn_angle += delta_angle;

        last_angle = rotation;
    }
    //Stop the robot before the rotation
    //Set the angular speed
    
    cmdVelPub.publish(geometry_msgs::Twist());
    ros::Duration(1).sleep(); // sleep for  a second
    speed.angular.z = 0;

    label--;
   if(label==0)
      {
       speed.angular.z = 0; 
       speed.linear.x = 0;
       cmdVelPub.publish(geometry_msgs::Twist());
       ros::Duration(1).sleep(); // sleep for  a second
       ROS_INFO("The robot move stops!");
       ros::shutdown();
      }
  }

}

int main(int argc,char** argv)
{

    ros::init(argc, argv, "laser_listener");
    ros::NodeHandle nh;
    ros::Rate r(60);
 
    while(1){
	ros::Subscriber sub = nh.subscribe("/odom", 1, odoCallback);
        ros::spin();
        sleep(1); 
     }
        
    return 0;
}
