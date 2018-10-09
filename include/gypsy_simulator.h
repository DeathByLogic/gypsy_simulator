/*
 * gypsy_simulator.h
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

#ifndef GYPSY_SIMULATOR_H_
#define GYPSY_SIMULATOR_H_

// Constants
#define PI 3.14159

// Functions
void drive_callback(const std_msgs::Float64& cmd_msg);
void turn_callback(const std_msgs::Float64& cmd_msg);
void update_speed(float drive, float turn);
geometry_msgs::Pose2D update_location(const geometry_msgs::Pose2D current_position, float left_delta, float right_delta);

#endif /* GYPSY_SIMULATOR_H_ */
