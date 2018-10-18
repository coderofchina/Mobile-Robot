#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <unistd.h>
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
//杀死ROS函数
void shutdown(int sig)
{
 	cmdVelPub.publish(geometry_msgs::Twist());
	ROS_INFO("odom_out_and_back.cpp ended!");
	ros::shutdown();
}

//odom回调函数
void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double rate = 20;
	int count = 0;
	//定义发布主题节点
	std::string topic = "/cmd_vel";
	ros::NodeHandle node;
	ros::NodeHandle cmdh;
	ros::Publisher cmdVelPub= cmdh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);;
	cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 1);

	ros::Rate loopRate(rate);
	//geometry_msgs::Twist speed;  //定义速度的发布节点
	//speed.linear.x=speed.linear.y=speed.linear.z=speed.angular.x=speed.angular.y=speed.angular.z=0.0;
	signal(SIGINT, shutdown);
	ROS_INFO("The robot starts moving!");

	//初始化速度和距离数据
	float linear_speed = 0.2;  //线速度
	float goal_distance = 1.0; //移动距离
	float angular_speed = 0.5; //角速度
	double goal_angle = M_PI;  //角位移
	double angular_tolerance = 2.5*M_PI/180; //角度转换成弧度:deg*PI/180

	tf::TransformListener listener;
	tf::StampedTransform transform;
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


	int label=4;   //运动次数标记
	//运动循环
	while(ros::ok())
	{ 
		ROS_INFO("go straight...");
                geometry_msgs::Twist speed; 
		speed.linear.x = linear_speed; // 设置线速度，正为前进，负为后退
                speed.angular.z = 0;      //旋转角速度设为0
		//设置直线运动的参数
		listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
		float x_start = transform.getOrigin().x();
		float y_start = transform.getOrigin().y();

		float distance = 0;     //已经运动的距离

		//直线运动循环
		while( (distance < goal_distance) && (ros::ok()) && label!=0)
		{
			cmdVelPub.publish(speed);
			loopRate.sleep();
			listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
			//获得当前位置信息
			float x = transform.getOrigin().x();
			float y = transform.getOrigin().y();
			distance = sqrt(pow((x - x_start), 2) +  pow((y - y_start), 2));//计算当前位置点与起始点之间的距离
		}

		cmdVelPub.publish(geometry_msgs::Twist());
		ros::Duration(1).sleep();   // 机器人运动停止（过渡作用）
		ROS_INFO("rotation...");

		//初始化机器人旋转参数
		speed.linear.x = 0; //设置线速度为0，即不做前后运动，只旋转
		speed.angular.z = angular_speed; // 设置角速度，正为左转，负为右转
		double last_angle = fabs(tf::getYaw(transform.getRotation()));  //yaw是围绕Y轴旋转，也叫偏航角
		double turn_angle = 0; //记录当前已旋转的角度

		//角度旋转循环
		while( (fabs(turn_angle) < M_PI/2) && (ros::ok()) && label!=0 )
		{
			//发布运动消息并循环
			cmdVelPub.publish(speed);
			loopRate.sleep();

			listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);  
			double rotation = fabs(tf::getYaw(transform.getRotation()));   //获得当前偏航角
			double delta_angle = fabs(rotation - last_angle);              //记录转过的角度（中间变量）
			turn_angle += delta_angle;                                     //累计转过的角度
			last_angle = rotation;                   
		}
 
		cmdVelPub.publish(geometry_msgs::Twist());
		ros::Duration(1).sleep(); // 机器人运动停止（过渡作用）
		
		
		//判断运动次数，若达到目标运动次数，则停止
		label--;
		if(label==0)
		{
			speed.angular.z = 0; 
			speed.linear.x = 0;
			cmdVelPub.publish(geometry_msgs::Twist());
			ros::Duration(1).sleep();  // 机器人运动停止
			ROS_INFO("The robot move stops!");
			ros::shutdown();
		}
	}

}

//主函数
int main(int argc,char** argv)
{

	ros::init(argc, argv, "laser_listener");
	ros::NodeHandle nh;
	ros::Rate r(60);

	//调用回调函数
	ros::Subscriber sub = nh.subscribe("/odom", 1, odoCallback);
        ros::spin();
        sleep(1); 
   	return 0;
}
