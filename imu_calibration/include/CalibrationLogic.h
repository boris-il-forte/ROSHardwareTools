/*
 * CalibrationLogic.h
 *
 *  Created on: 12 dic 2015
 *      Author: dave
 */

#ifndef INCLUDE_CALIBRATIONLOGIC_H_
#define INCLUDE_CALIBRATIONLOGIC_H_

#include <hardware_tools_msgs/ImuRaw.h>
#include <ros/ros.h>

#include "FIRfilter.h"

class CalibrationLogic
{
public:
	CalibrationLogic(ros::NodeHandle& n);
	void imuMsgHandler(const hardware_tools_msgs::ImuRaw& msg);

private:
	ros::NodeHandle& n;
	ros::Subscriber imuSubscriber;

	//Filtering algorithm
	FIRfilter filter;

};



#endif /* INCLUDE_CALIBRATIONLOGIC_H_ */
