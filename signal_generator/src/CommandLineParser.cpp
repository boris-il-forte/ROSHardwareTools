/*
 * SerialToRos, Read stuff from serial and publish to ros
 *
 * Copyright (C) 2012 Politecnico di Milano
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

#include "CommandLineParser.h"

#include "SinSignal.h"
#include "PwmSignal.h"
#include "StepSignal.h"

#include <iostream>

using namespace std;
using namespace boost::program_options;

enum SupportedSignals
{
	SIN, PWM, STEP
};

void validate(boost::any& v, const std::vector<std::string>& values,
			SupportedSignals* target_type, int)
{
	using namespace boost::program_options;

	// Make sure no previous assignment to 'a' was made.
	validators::check_first_occurrence(v);
	// Extract the first string from 'values'. If there is more than
	// one string, it's an error, and exception will be thrown.
	const string& s = validators::get_single_string(values);

	//Check if is valid
	if (s == "sin")
		v = SIN;
	else if (s == "pwm")
		v = PWM;
	else if (s == "step")
		v = STEP;
	else
		throw validation_error(validation_error::invalid_option_value);
}

CommandLineParser::CommandLineParser() :
			desc("Usage")
{
	desc.add_options() //
	("help,h", "produce help message") //
	("signal,s", value<SupportedSignals>()->default_value(SIN, "sin"),
				"selects the signal to generate") //
	("sampling-frequency,x", value<double>()->default_value(100),
				"set the sampling frequency of signal") //
	("output-topic,o", value<string>()->default_value("/signal"),
				"set the output topic") //
	("amplitude,a", value<double>()->default_value(1.0), "set amplitude") //
	("frequency,f", value<double>()->default_value(5.0), "set frequency") //
	("duty-cycle,d", value<double>()->default_value(0.5), "set duty cycle") //
	("time,t", value<double>()->default_value(0.0), "set start time") //
	("offset", value<double>()->default_value(0.0));

}

bool CommandLineParser::parseCommandLine(int argc, char **argv)
{
	try
	{
		store(parse_command_line(argc, argv, desc), vm);
		if (vm.count("help"))
		{
			cout << desc << endl;
			return false;
		}

		notify(vm);

	}
	catch (error& e)
	{
		cout << e.what() << endl;
		cout << desc << endl;
		return false;
	}

	return true;

}

double CommandLineParser::getSampligFrequency()
{
	return vm["sampling-frequency"].as<double>();
}

Signal* CommandLineParser::getSignal(ros::NodeHandle& n)
{
	SupportedSignals reader = vm["signal"].as<SupportedSignals>();
	std::string topic = vm["output-topic"].as<string>();

	double frequency = vm["frequency"].as<double>();
	double dutyCycle = vm["duty-cycle"].as<double>();
	double amplitude = vm["amplitude"].as<double>();
	double time = vm["time"].as<double>();
	double offset = vm["offset"].as<double>();

	switch (reader)
	{
		case SIN:
		{
			return new SinSignal(n, topic, frequency, amplitude, offset);
		}
		case PWM:
		{
			return new PwmSignal(n, topic, frequency, amplitude, dutyCycle, offset);
		}
		case STEP:
		{
			return new StepSignal(n, topic, amplitude, time, offset);
		}
		default:
			throw std::logic_error("This should not happen");
	}
}

