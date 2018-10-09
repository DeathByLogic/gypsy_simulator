/*
 * gypsydriver.cpp
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

#include <cmath>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include "gypsy_simulator.h"

// Macros
#define constrain(val, min, max) (val > max)?max:(val < min)?min:val

// Global variables
float wheel_base;
float max_speed;
float max_turn;

int motor_cmd_max;
int publish_rate;

float drive_cmd;
float turn_cmd;

// Left and right wheel speed
std_msgs::Float32 left_speed;
std_msgs::Float32 right_speed;

int main (int argc, char** argv) {
	// Store current and previous time
	ros::Time current_time, last_time;

	// Current position and velocity
	geometry_msgs::Pose2D current_pos;
	geometry_msgs::Pose2D current_vel;

	// Topics
	std::string drive_topic, turn_topic;

	// Init ROS
	ros::init(argc, argv, "gypsy_simulator");

	// Create a new node
	ros::NodeHandle n;

	// Get parameters
	if (!n.getParam("gypsy_simulator/wheel_base", wheel_base)) {
		ROS_ERROR("Sabertooth: Failed to get param wheel_base");

		// Quit on error
		exit(EINVAL);
	}

	if (!n.getParam("gypsy_simulator/max_speed", max_speed)) {
		ROS_ERROR("Sabertooth: Failed to get param max_speed");

		// Quit on error
		exit(EINVAL);
	}

	if (!n.getParam("gypsy_simulator/max_turn", max_turn)) {
		ROS_ERROR("Sabertooth: Failed to get param max_turn");

		// Quit on error
		exit(EINVAL);
	}

	n.param<int>("gypsy_simulator/motor_cmd_max", motor_cmd_max, 127);
	n.param<int>("gypsy_simulator/publish_rate", publish_rate, 20);

	n.param<std::string>("gypsy_simulator/drive_topic", drive_topic, "/drive_command");
	n.param<std::string>("gypsy_simulator/turn_topic", turn_topic, "/turn_command");

	// Subscribe to the motor commands
	ros::Subscriber drive_sub = n.subscribe(drive_topic, 50, drive_callback);
	ros::Subscriber turn_sub = n.subscribe(turn_topic, 50, turn_callback);

	// Create publication handlers
	ros::Publisher left_speed_pub = n.advertise<std_msgs::Float32>("left_wheel_speed", 10);
	ros::Publisher right_speed_pub = n.advertise<std_msgs::Float32>("right_wheel_speed", 10);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	// Get and save current time
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(publish_rate);
	while (n.ok()) {
		// Check for incoming messages
		ros::spinOnce();
		
		// Calculate change in time (dt)
		current_time = ros::Time::now();		
		double dtt = (current_time - last_time).toSec();

		// Calculate velocites
		current_vel.x = (right_speed.data + left_speed.data) / 2;
		current_vel.y = 0.0;
		current_vel.theta = (right_speed.data - left_speed.data) / wheel_base;

		// Calculate position
		current_pos = update_location(current_pos, right_speed.data * dtt, left_speed.data * dtt);

		// Create a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_pos.theta);

		// Publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = current_pos.x;
		odom_trans.transform.translation.y = current_pos.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		// Send the transform
		odom_broadcaster.sendTransform(odom_trans);

		// Publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		// Set the position
		odom.pose.pose.position.x = current_pos.x;
		odom.pose.pose.position.y = current_pos.y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// Set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = current_vel.x;
		odom.twist.twist.linear.y = current_vel.y;
		odom.twist.twist.angular.z = current_vel.theta;

		// Publish the message
		odom_pub.publish(odom);

		// Publish current left and right velocity
		left_speed_pub.publish(left_speed);
		right_speed_pub.publish(right_speed);

		// Update time
		last_time = current_time;

		// Wait for next period
		r.sleep();
	}

	return 0;
}

void drive_callback(const std_msgs::Float64& cmd_msg) {
	drive_cmd = cmd_msg.data;

	update_speed(drive_cmd, turn_cmd);
}

void turn_callback(const std_msgs::Float64& cmd_msg) {
	turn_cmd = cmd_msg.data;

	update_speed(drive_cmd, turn_cmd);
}

void update_speed(float drive, float turn) {
	float left_command = constrain(drive - turn / 2.0, -motor_cmd_max, motor_cmd_max);
	float right_command = constrain(drive + turn / 2.0, -motor_cmd_max, motor_cmd_max);

	left_speed.data = left_command / (float)motor_cmd_max * max_speed;
	right_speed.data = right_command / (float)motor_cmd_max * max_speed;
}

// Update the calculated position with new data
geometry_msgs::Pose2D update_location(const geometry_msgs::Pose2D current_position, float left_delta, float right_delta) {
	geometry_msgs::Pose2D new_position = current_position;

	float theta_l;
	float theta_r;
	float theta_sum;

	// Caclulate new theta
	theta_l = -left_delta / wheel_base;
	theta_r = right_delta / wheel_base;

	new_position.theta += theta_l + theta_r;
	
	// Calculate new position
	new_position.x += 0.5 * wheel_base * (sin(current_position.theta) - sin(current_position.theta + theta_l));
	new_position.y += 0.5 * wheel_base * (cos(current_position.theta + theta_l) - cos(current_position.theta));
	
	new_position.x += 0.5 * wheel_base * (sin(current_position.theta + theta_r) - sin(current_position.theta));
	new_position.y += 0.5 * wheel_base * (cos(current_position.theta) - cos(current_position.theta + theta_r));

	// Keep theta within +- PI
	if (new_position.theta > PI) {
		new_position.theta -= 2 * PI;
	}

	if (new_position.theta < -PI) {
		new_position.theta += 2 * PI;
	}

	return new_position;
}

