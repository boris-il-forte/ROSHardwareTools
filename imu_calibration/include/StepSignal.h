/*
 * StepSignal.h
 *
 *  Created on: 11 nov 2015
 *      Author: dave
 */

#ifndef INCLUDE_STEPSIGNAL_H_
#define INCLUDE_STEPSIGNAL_H_

class StepSignal : public Signal
{
public:
	StepSignal(ros::NodeHandle& n, const std::string& topic, double amplitude, double time)
		: Signal(n, topic)
	{
		a = amplitude;
		tUp = time;
	}

	virtual double computeSignalValue(double t) override
	{
		return (t > tUp) ? a : 0.0;
	}



private:
	double tUp;
	double a;


};



#endif /* INCLUDE_STEPSIGNAL_H_ */
