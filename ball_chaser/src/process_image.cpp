#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the robot");

    // Request appropriate velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int pixel_num = -1;
    int pixel_pos;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	
    // Loop through each pixel in the image and determine if it's bright white
    for (int i = 0; i < img.height * img.step; i = i + 3) {
	    
		if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
			// Determine location of bright white pixel and provide appropriate command
			pixel_num = i;
			break;
		}
    }
    pixel_pos = pixel_num % img.step;	
    if (pixel_num >= 0) {
	    if (pixel_pos <= int(img.step / 3)) {
		    ROS_INFO("White ball detected on left");
		    drive_robot(0.5, 0.5);
		
	    }else if (pixel_pos >= int(img.step * 2/3)) {
		    ROS_INFO("White ball detected on right");
		    drive_robot(0.5, -0.5);
	
	    }else {
		    ROS_INFO("White ball detected in middle");	
		    drive_robot(0.5, 0);
	    }
    }
    else if (pixel_num < 0) {
	drive_robot(0.0, 0.0);
}
	
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
