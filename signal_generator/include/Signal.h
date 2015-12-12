/*
 * Signal.h
 *
 *  Created on: 11 nov 2015
 *      Author: dave
 */

#ifndef INCLUDE_SIGNAL_H_
#define INCLUDE_SIGNAL_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>

class Signal
{
public:
	Signal(ros::NodeHandle& n, const std::string& topic) :
				t(0)
	{
		pub = n.advertise<std_msgs::Float32>(topic, 1);
	}

	virtual void emitSignalStep(double f)
	{
		std_msgs::Float32 msg;

		msg.data = computeSignalValue(t/f);

		pub.publish(msg);

		t++;
	}

	virtual ~Signal()
	{
	}

protected:
	virtual double computeSignalValue(double t) = 0;

private:
	int t;
	ros::Publisher pub;

};

#endif /* INCLUDE_SIGNAL_H_ */
