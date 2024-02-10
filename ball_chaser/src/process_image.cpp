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
    srv.request.linear.x = lin_x;
    srv.request.angular.z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
	int pixel_pos;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	
    // Loop through each pixel in the image and determine if it's bright white
    for (int i = 0; i < img.height * img.step; i = i + 3) {
		
		if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel){
			// Determine location of bright white pixel and provide appropriate command
			pixel_pos = i % img.step;
			
			if (pixel_pos <= int(img.step / 3)){
				ROS_INFO("White ball detected on left");
				drive_robot(0.0, 0.5);
				
			}else if(img.width <= int(pixel_pos * 2/3)){
				ROS_INFO("White ball detected in middle");	
				drive_robot(0.5, 0);
				
			}else if(img.width >= int(pixel_pos * 2/3)){
				ROS_INFO("White ball detected on right");
				drive_robot(0.0, -0.5);	
				
			}else{
				drive_robot(0.0, 0.0);
			}
		}
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


#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

// Define global vector of joints last position, moving state of the arm, and the client that can request services
std::vector<double> joints_last_position{ 0, 0 };
bool moving_state = false;
ros::ServiceClient client;

// This function calls the safe_move service to safely move the arm to the center position
void move_arm_center()
{
    ROS_INFO_STREAM("Moving the arm to the center");

    // Request centered joint angles [1.57, 1.57]
    simple_arm::GoToPosition srv;
    srv.request.joint_1 = 1.57;
    srv.request.joint_2 = 1.57;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}

// This callback function continuously executes and reads the arm joint angles position
void joint_states_callback(const sensor_msgs::JointState js)
{
    // Get joints current position
    std::vector<double> joints_current_position = js.position;

    // Define a tolerance threshold to compare double values
    double tolerance = 0.0005;

    // Check if the arm is moving by comparing its current joints position to its latest
    if (fabs(joints_current_position[0] - joints_last_position[0]) < tolerance && fabs(joints_current_position[1] - joints_last_position[1]) < tolerance)
        moving_state = false;
    else {
        moving_state = true;
        joints_last_position = joints_current_position;
    }
}

// This callback function continuously executes and reads the image data
void look_away_callback(const sensor_msgs::Image img)
{

    bool uniform_image = true;

    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] - img.data[0] != 0) {
            uniform_image = false;
            break;
        }
    }

    // If the image is uniform and the arm is not moving, move the arm to the center
    if (uniform_image == true && moving_state == false)
        move_arm_center();
}

int main(int argc, char** argv)
{
    // Initialize the look_away node and create a handle to it
    ros::init(argc, argv, "look_away");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from safe_move
    client = n.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    // Subscribe to /simple_arm/joint_states topic to read the arm joints position inside the joint_states_callback function
    ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states", 10, joint_states_callback);

    // Subscribe to rgb_camera/image_raw topic to read the image data inside the look_away_callback function
    ros::Subscriber sub2 = n.subscribe("rgb_camera/image_raw", 10, look_away_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

