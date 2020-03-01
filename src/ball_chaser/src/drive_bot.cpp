#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

class DriveRequest
{
public:
bool call_back_drive_request(ball_chaser::DriveToTarget::Request& req,
           ball_chaser::DriveToTarget::Response& res);
};

bool DriveRequest::call_back_drive_request(ball_chaser::DriveToTarget::Request& req,
           ball_chaser::DriveToTarget::Response& res)
{

    ROS_INFO("DriveToTarget received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);


    geometry_msgs::Twist motor_command;
        // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
        // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "DriveToTarget - linear_x: " + std::to_string(motor_command.linear.x) + " , angular_z: " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;

}



int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    //topic name = /cmd_vel, quesize = 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Set driver request class
    // Service name: "/ball_chaser/command_robot"
    // Call back function: &DriveRequest::call_back_drive_request
    // Reference class: &driverequest
    DriveRequest driverequest;
   
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", &DriveRequest::call_back_drive_request,&driverequest);
    ROS_INFO("Ready to send command_robot");
    ros::spin();

    return 0;
}
