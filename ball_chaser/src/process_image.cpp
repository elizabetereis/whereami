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

void process_image_callback(const sensor_msgs::Image img)
{
  int c_white_pixel = 255;
  int left = 0, center = 0, right = 0;
  bool ball;
  
  for (int i = 0; i < img.height; i++)
  {   
    int row_offset = i*img.step;
    for (int j = 0; j < img.step; j+=3)
    {
      if ( (img.data[row_offset + j] == c_white_pixel) 
          && (img.data[row_offset + j + 1] == c_white_pixel) 
          && (img.data[row_offset + j + 2] == c_white_pixel) )
      {
        int index = j%img.step;
        ball = true;
        if (index < img.step/3)
          left++;
        else if (index < img.step/3 * 2)
          center++;
        else
          right++;     
      }        
    }

    if (ball)
      break;
  }

  if (!ball)
  {
    drive_robot(0.0, 0.0); // Stop
    return;
  }
  
  if (left > center && left > right)
    drive_robot(0.0, 0.5);
  else if (center > left && center > right)
    drive_robot(0.5, 0.0);
  else
    drive_robot(0.0, -0.5);
}
// This callback function continuously executes and reads the image data
/*void process_image_callback(const sensor_msgs::Image img)
{
	const int c_white_pixel = 255;
  int ball = -1;
  int index;
  int left = 0, center = 0, right = 0;
  for (int i = 0; i < img.height; i++)
  {
    for (int j = 0; j < img.width; j++)
    {
      index = (i*img.height+j) * 3;
      if (img.data[index] == c_white_pixel)
      {
        ball = j;
        if (ball < img.step/3)
          left++;
        else if (ball > img.step * 2/3 )
          center++;
        else
          right++;

        break;
      }
    }
    if (ball > -1)
      break;
  }

  if (ball == -1)
  {
    drive_robot(0.0, 0.0);
    return;
  }

  if (left > center && left > right)
    drive_robot(0.0, 0.5);
  else if (center > left && center > right)
    drive_robot(0.5, 0.0);
  else
    drive_robot(0.0, -0.5);
}*/

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