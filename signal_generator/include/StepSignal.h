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
	StepSignal(ros::NodeHandle& n, const std::string& topic, double amplitude, double time, double offset)
		: Signal(n, topic)
	{
		a = amplitude;
		tUp = time;
		o = offset;
	}

	virtual double computeSignalValue(double t) override
	{
		return (t > tUp) ? a + o : o;
	}



private:
	double tUp;
	double a;
	double o;


};



#endif /* INCLUDE_STEPSIGNAL_H_ */
