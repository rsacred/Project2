#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher; 

// This handle_drive_request callback function executes whenever a drive_bot service is requested
// This function  publishes the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback is returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){
	
	ROS_INFO("DriveToTargetRequest received - x:%1.2f, z:%1.2f", (float)req.linear.x, (float)req.angular.z);
	
	// Create a motor_command object of type geometry_msgs::Twist
	geometry_msgs::Twist motor_command;
	
	// Set wheel velocities, forward
	motor_command.linear.x = req.linear.x;
	motor_command.angular.z = req.angular.z;
	
	// Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);
	
	// Return a response message
	res.msg_feedback = "Motor commands set - x: " + std::to_string(motor_command.linear.x) + " , z: " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);
	
	return true
}

int main(int argc, char** argv){

    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
	ros::ServiceServer service = n.advertiseService(`/ball_chaser/command_robot`, handle_drive_request);
    ROS_INFO("Ready to send drive commands");
	     
    // Handle ROS communication events
    ros::spin();

    return 0;
}
