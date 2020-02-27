#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <geometry_msgs/Twist.h>

// Global publisher
ros::Publisher cmd_vel_pub;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, 
	ball_chaser::DriveToTarget::Response& res){
	ROS_INFO("DriveToTarget Request recieved - linear.x: %1.2f, angular.z: %1.2f", float(req.linear_x), float(req.angular_z));
	geometry_msgs::Twist msg;
	msg.linear.x = float(req.linear_x);
	msg.angular.z = float(req.angular_z);
	cmd_vel_pub.publish(msg);
	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "drive_bot");
	ros::NodeHandle n;

	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);


	ros::spin();
	return 0;
}