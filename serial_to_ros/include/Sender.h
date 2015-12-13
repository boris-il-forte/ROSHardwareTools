/*
 * SerialToRos, Read stuff from serial and publish to ros
 *
 * Copyright (C) 2012 Politecnico di Milano
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

#ifndef INCLUDE_SENDER_H_
#define INCLUDE_SENDER_H_

#include <std_msgs/String.h>
#include <ros/ros.h>

#include "SerialComunicationHandler.h"

class Sender
{
public:
	Sender(ros::NodeHandle& n, SerialComunicationHandler& comunicationHandler);
	void stringSubscriberHandler(const std_msgs::String& msg);

private:
	ros::NodeHandle& n;
	ros::Subscriber stringSubscriber;
	SerialComunicationHandler& comunicationHandler;

};



#endif /* INCLUDE_SENDER_H_ */
