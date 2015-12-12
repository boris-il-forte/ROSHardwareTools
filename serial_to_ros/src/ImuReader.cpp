/*
 * ImuRawParser.cpp
 *
 *  Created on: 04/nov/2015
 *      Author: dave
 */

#include "../include/ImuReader.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

ImuReader::ImuReader(ros::NodeHandle& n) :
			Reader(n)
{
	imuPublisher = n.advertise<sensor_msgs::Imu>("imu", 1000);
	magPublisher = n.advertise<sensor_msgs::MagneticField>("mag", 1000);
}

void ImuReader::parseLine(const std::string& line, const ros::Time& stamp)
{

	try
	{
		std::vector<std::string> tokenVec;
		tokenize(line, " ", tokenVec);


		sensor_msgs::Imu imuMsg;
		sensor_msgs::MagneticField magMsg;

		imuMsg.linear_acceleration.x = std::stod(tokenVec[0]);
		imuMsg.linear_acceleration.y = std::stod(tokenVec[1]);
		imuMsg.linear_acceleration.z = std::stod(tokenVec[2]);

		imuMsg.angular_velocity.x = std::stod(tokenVec[3]);
		imuMsg.angular_velocity.y = std::stod(tokenVec[4]);
		imuMsg.angular_velocity.z = std::stod(tokenVec[5]);

		magMsg.magnetic_field.x = std::stod(tokenVec[6]);
		magMsg.magnetic_field.y = std::stod(tokenVec[7]);
		magMsg.magnetic_field.z = std::stod(tokenVec[8]);

		imuMsg.header.frame_id = "imu_link";
		imuMsg.header.stamp = stamp;

		magMsg.header.frame_id = "imu_link";
		magMsg.header.stamp = stamp;

		imuPublisher.publish(imuMsg);
		magPublisher.publish(magMsg);

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
