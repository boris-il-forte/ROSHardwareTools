/*
 * SerialToRos, Read stuff from serial and publish to ros
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Marcello Pogliani, Davide Tateo
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

#include "CommandLineParser.h"

int main(int argc, char** argv)
{
	CommandLineParser cl;

	// Initialise ROS
	ros::init(argc, argv, "SignalGenerator");
	ros::NodeHandle ros_node;

	// Parse command line
	bool ok = cl.parseCommandLine(argc, argv);
	Signal* signal =  nullptr;
	signal = cl.getSignal(ros_node);
	double freq = cl.getSampligFrequency();

	if (ok && signal)
	{
		ros::Rate r(freq);

		// Start serial comunication
		while (ros::ok())
		{
			signal->emitSignalStep(freq);
			ros::spinOnce();
			r.sleep();
		}

		delete signal;
	}

	return 0;
}
