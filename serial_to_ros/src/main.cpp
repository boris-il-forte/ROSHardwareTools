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

#include "CommandLineReader.h"
#include "ImuRawReader.h"
#include "SerialComunicationHandler.h"
#include "Sender.h"

int main(int argc, char** argv)
{
	CommandLineParser cl;

	// Initialise ROS
	ros::init(argc, argv, "SerialToRos");
	ros::NodeHandle ros_node;

	// Parse command line
	cl.parseCommandLine(argc, argv);
	std::string serialFilename = cl.getSerialDevice();
	double freq = cl.getFrequency();
	std::string command = cl.getInitialCommand();


	// Start serial comunication
	SerialComunicationHandler comunicationHandler(serialFilename);

	ros::Duration delta;
	delta.fromSec(1.0 / freq);

	//init sender
	Sender sender(ros_node, comunicationHandler);

	//Init parser
	Reader* reader = cl.getReader(ros_node);

	//send initial command
	comunicationHandler.sendStringCommand(command.c_str());

	while (ros::ok())
	{
		if (comunicationHandler.readData() == 0)
		{
			ros::Time stamp = ros::Time::now();
			unsigned int n_line = comunicationHandler.getLineToParseNum();

			for (unsigned int i = 0; i < n_line; i++)
			{

				std::string l = comunicationHandler.getLine();
				if (!l.empty())
				{
					std::cout << l << std::endl;
					reader->parseLine(l, stamp);
					stamp = stamp + delta;
				}
			}
		}

		ros::spinOnce();
	}

	delete reader;

	return 0;
}
