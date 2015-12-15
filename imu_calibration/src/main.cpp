/*
 * imu_calibration, Calibrates an IMU
 *
 * Copyright (C) 2015 Davide Tateo
 * Versione 1.0
 *
 * This file is part of SerialToRos.
 *
 * SerialToRos is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SerialToRos is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with SerialToRos.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>

#include <ros/ros.h>

#include "CalibrationLogic.h"


int main(int argc, char** argv)
{

	// Initialise ROS
	ros::init(argc, argv, "ImuCalibration");
	ros::NodeHandle ros_node;

	//start calibration dispatcher
	CalibrationLogic logic(ros_node);

	ros::spin();

	return 0;
}
