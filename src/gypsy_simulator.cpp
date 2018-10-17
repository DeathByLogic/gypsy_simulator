/*
 * gypsydriver.cpp
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

// C Includes
#include <cmath>
#include <unistd.h>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

// Application Includes
#include "gypsy_simulator.h"

// Macros
#define constrain(val, min, max) (val > max)?max:(val < min)?min:val

// Global variables
float wheel_base;
float wheel_diam;
float max_speed;

int motor_cmd_max;
int publish_rate;

float drive_cmd, turn_cmd;
float left_speed, right_speed;
float left_pos, right_pos;

int main (int argc, char** argv) {
	// Store current and previous time
	ros::Time current_time, last_time;

	// Messages
	geometry_msgs::Pose2D current_pos;
	geometry_msgs::Pose2D current_vel;
	sensor_msgs::JointState joint_state;

	// Topics
	std::string drive_topic, turn_topic;

	// Init ROS
	ros::init(argc, argv, "gypsy_simulator");

	// Create a new node
	ros::NodeHandle n;

	// Get parameters
	if (!n.getParam("gypsy_simulator/wheel_base", wheel_base)) {
		ROS_ERROR("Gypsy Simulator: Failed to get param wheel_base");

		// Quit on error
		exit(EINVAL);
	}

	if (!n.getParam("gypsy_simulator/wheel_diameter", wheel_diam)) {
		ROS_ERROR("Gypsy Simulator: Failed to get param wheel_diameter");

		// Quit on error
		exit(EINVAL);
	}

	if (!n.getParam("gypsy_simulator/max_speed", max_speed)) {
		ROS_ERROR("Gypsy Simulator: Failed to get param max_speed");

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
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	// Resize and name the joint states
	joint_state.name.resize(2);
	joint_state.position.resize(2);
	joint_state.velocity.resize(2);

	joint_state.name[0] ="left_wheel_joint";
	joint_state.name[1] ="right_wheel_joint";

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
		current_vel.x = (right_speed + left_speed) / 2;
		current_vel.y = 0.0;
		current_vel.theta = (right_speed - left_speed) / wheel_base;

		// Calculate position
		//current_pos = update_location(current_pos, dtt);
		current_pos = update_location(current_pos, left_speed * dtt, right_speed * dtt);

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

		// Update the Joint State
		joint_state.header.stamp = current_time;

		joint_state.position[0] += (left_speed * dtt) / (wheel_diam / 2.0);
		joint_state.velocity[0] = left_speed;

		joint_state.position[1] += (right_speed * dtt) / (wheel_diam / 2.0);
		joint_state.velocity[1] = right_speed;

		// Publish the joint state
		joint_pub.publish(joint_state);

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

void update_speed(float drive_command, float turn_command) {
	float left_command = constrain(drive_command - turn_command / 2.0, -motor_cmd_max, motor_cmd_max);
	float right_command = constrain(drive_command + turn_command / 2.0, -motor_cmd_max, motor_cmd_max);

	left_speed = left_command / (float)motor_cmd_max * max_speed;
	right_speed = right_command / (float)motor_cmd_max * max_speed;
}

// Update the calculated position with new data
geometry_msgs::Pose2D update_location(const geometry_msgs::Pose2D current_position, float left_delta, float right_delta) {
	geometry_msgs::Pose2D new_position;

	float theta_l;
	float theta_r;

	// Caclulate new theta
	theta_l = -left_delta / wheel_base;
	theta_r = right_delta / wheel_base;
	
	// Keep Theta within -PI to PI
	new_position.theta = fmod(current_position.theta + theta_r + theta_l + PI, 2 * PI) - PI;

	// Calculate new position
	new_position.x = current_position.x;
	new_position.y = current_position.y;

	new_position.x += 0.5 * wheel_base * (sin(current_position.theta) - sin(current_position.theta + theta_l));
	new_position.y += 0.5 * wheel_base * (cos(current_position.theta + theta_l) - cos(current_position.theta));
	
	new_position.x += 0.5 * wheel_base * (sin(current_position.theta + theta_r) - sin(current_position.theta));
	new_position.y += 0.5 * wheel_base * (cos(current_position.theta) - cos(current_position.theta + theta_r));

	return new_position;
}

