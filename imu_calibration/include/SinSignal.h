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
	SinSignal(ros::NodeHandle& n, const std::string& topic, double frequency, double amplitude)
		: Signal(n, topic)
	{
		a = amplitude;
		omega = frequency*2*M_PI;
	}

	virtual double computeSignalValue(double t) override
	{
		return a*std::sin(omega*t);
	}



private:
	double omega;
	double a;


};



#endif /* INCLUDE_SINSIGNAL_H_ */
