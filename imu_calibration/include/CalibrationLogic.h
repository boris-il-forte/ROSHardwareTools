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

#ifndef INCLUDE_CALIBRATIONLOGIC_H_
#define INCLUDE_CALIBRATIONLOGIC_H_

#include <hardware_tools_msgs/ImuRaw.h>
#include <ros/ros.h>

#include <vector>

class CalibrationLogic
{
public:
	CalibrationLogic(ros::NodeHandle& n);
	void imuMsgHandler(const hardware_tools_msgs::ImuRaw& msg);


	void calibrateAccelerometer();
	void calibrateMagnetometer();
	void calibrateGyroscope();


private:
	ros::NodeHandle& n;
	ros::Subscriber imuSubscriber;

	//Filtering algorithm
	std::vector<hardware_tools_msgs::ImuRaw> dataSet;

};



#endif /* INCLUDE_CALIBRATIONLOGIC_H_ */
