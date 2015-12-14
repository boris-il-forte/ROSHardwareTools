/*
 * SinSignal.h
 *
 *  Created on: 11 nov 2015
 *      Author: dave
 */

#ifndef INCLUDE_SINSIGNAL_H_
#define INCLUDE_SINSIGNAL_H_

#include "Signal.h"

class SinSignal : public Signal
{
public:
	SinSignal(ros::NodeHandle& n, const std::string& topic, double frequency, double amplitude, double offset)
		: Signal(n, topic)
	{
		a = amplitude;
		omega = frequency*2*M_PI;
		o = offset;
	}

	virtual double computeSignalValue(double t) override
	{
		return a*std::sin(omega*t)+o;
	}



private:
	double omega;
	double a;
	double o;


};



#endif /* INCLUDE_SINSIGNAL_H_ */
