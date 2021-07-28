#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <fstream>
// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
	ROS_INFO("Move the robot [%1.2f], [%1.2f]", lin_x, ang_z);

	ball_chaser::DriveToTarget target;
	target.request.linear_x = lin_x;
	target.request.angular_z = ang_z;

	if (!client.call(target))
    ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
	const int c_white_pixel = 255;
  std::vector<int> pixels_sum = {0, 0, 0};
  int step_count = 0;
  bool ball = false; 
  for (int i = 0; i < img.height; i++)
	{
    for (int j = 0; j < img.width; j+=3)
    {
      int base_index = j + step_count;
      if (img.data[base_index] == c_white_pixel && img.data[base_index+1] == c_white_pixel && img.data[base_index+2] == c_white_pixel)
      {
        drive_robot(0.0, 0.5);
        pixels_sum[0]++;
        ball = true;
        break;
      }

      base_index += img.width/3; 
      if (img.data[base_index] == c_white_pixel && img.data[base_index+1] == c_white_pixel && img.data[base_index+2] == c_white_pixel)
      {
        drive_robot(0.5, 0.0);
        pixels_sum[1]++;
        ball = true;
        break;
      }

      base_index += img.width/3; 
      if (img.data[base_index] == c_white_pixel && img.data[base_index+1] == c_white_pixel && img.data[base_index+2] == c_white_pixel)
      {
        pixels_sum[2]++;
        drive_robot(0.0, -0.5);
        ball = true;
        break;
      }
    }
    step_count += img.step;
    if(ball)
      break;
  }

  if((pixels_sum[0] + pixels_sum[1] + pixels_sum[2]) == 0)
    drive_robot(0.0, 0.0);
}

int main(int argc, char** argv)
{
	// Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	// Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	// Handle ROS communication events
	ros::spin();

	return 0;
}