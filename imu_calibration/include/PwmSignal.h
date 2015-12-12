/*
 * PwmSignal.h
 *
 *  Created on: 11 nov 2015
 *      Author: dave
 */

#ifndef INCLUDE_PWMSIGNAL_H_
#define INCLUDE_PWMSIGNAL_H_

class PwmSignal : public Signal
{
public:
	PwmSignal(ros::NodeHandle& n, const std::string& topic, double frequency, double amplitude, double dutyCycle)
		: Signal(n, topic)
	{
		a = amplitude;
		tUp = dutyCycle/frequency;
		T = 1/frequency;
	}

	virtual double computeSignalValue(double t) override
	{
		double tMod = std::fmod(t, T);

		return (tMod > tUp) ? 0 : a;
	}



private:
	double tUp;
	double a;
	double T;


};



#endif /* INCLUDE_PWMSIGNAL_H_ */
