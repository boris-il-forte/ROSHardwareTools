/*
 * ImuRawParser.cpp
 *
 *  Created on: 04/nov/2015
 *      Author: dave
 */

#include <hardware_tools_msgs/ImuRaw.h>
#include "../include/ImuRawReader.h"

ImuRawReader::ImuRawReader(ros::NodeHandle& n) :
			Reader(n)
{
	publisher = n.advertise<hardware_tools_msgs::ImuRaw>("imu_raw", 1000);
}

void ImuRawReader::parseLine(const std::string& line, const ros::Time& stamp)
{

	try
	{
		std::vector<std::string> tokenVec;
		tokenize(line, " ", tokenVec);

		hardware_tools_msgs::ImuRaw msg;

		for(unsigned i = 0; i < 9; i++)
		{
			msg.data[i] = std::stoi(tokenVec[i]);
		}

		msg.header.frame_id = "imu_link";
		msg.header.stamp = stamp;

		publisher.publish(msg);

	}
	catch (std::runtime_error & e)
	{
		std::cout << "Discarting wrong line: " << e.what() << std::endl;
	}
	catch(std::invalid_argument & e)
	{
		std::cout << "Parse error: " << e.what() << std::endl;
	}

}
