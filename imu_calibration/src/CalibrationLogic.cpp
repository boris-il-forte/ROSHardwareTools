/*
 * CalibrationLogic.cpp
 *
 *  Created on: 12 dic 2015
 *      Author: dave
 */


#include "CalibrationLogic.h"

CalibrationLogic::CalibrationLogic(ros::NodeHandle& n) : n(n)
{
	imuSubscriber = n.subscribe("imu_raw", 1000, &CalibrationLogic::imuMsgHandler, this);
}

void CalibrationLogic::imuMsgHandler(const hardware_tools_msgs::ImuRaw& msg)
{

	arma::vec data(9);
	for(unsigned int i = 0; i < 9; i++)
		data[i] = msg.data.elems[i];

	filter.add(data);

}

