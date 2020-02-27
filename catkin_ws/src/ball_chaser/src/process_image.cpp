#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client; 


// call the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z){
	// call the service 
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	if(!client.call(srv)) {
		ROS_ERROR("Failed to call DriveToTarget service");
	}


}


// parses and executes the image data 
void process_image_callback(const sensor_msgs::Image img) {
	int white_pixel = 255;

	float sections = img.step / 3;

	float left_bound = sections;
	float right_bounds = sections * 2;


	for(int i = 0; i < img.height * img.step; i++) {
		if(img.data[i] == white_pixel){
			// figure out if the img is to the left, right, or middle of us 

			int relative_pos = i % img.step;


			if(relative_pos > left_bound && relative_pos < right_bounds){
				// go forward
				drive_robot(0.5, 0.0);
				return;
			}
			else if(relative_pos < left_bound) { // pixel is left of img 
				// go left 
				drive_robot(0.0, 0.5);
				return;
			}
			else if(relative_pos > right_bounds) { // pixel is right of img
				// go right
				drive_robot(0.0, -0.5);
				return;
			}
		
		}

	}

	// if there is no white pixel stop the robot
	drive_robot(0.0, 0.0);
	return;
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "process_image");
	ros::NodeHandle n; 

	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	ros::spin();
	return 0;
}