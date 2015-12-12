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

#include <iostream>

#include "../include/ImuReader.h"
#include "CommandLineReader.h"
#include "ImuRawReader.h"

using namespace std;
using namespace boost::program_options;

enum SupportedReaders
{
	Imu, ImuRaw
};

void validate(boost::any& v,
              const std::vector<std::string>& values,
			  SupportedReaders* target_type, int)
{
    using namespace boost::program_options;

    // Make sure no previous assignment to 'a' was made.
    validators::check_first_occurrence(v);
    // Extract the first string from 'values'. If there is more than
    // one string, it's an error, and exception will be thrown.
    const string& s = validators::get_single_string(values);

    //Check if is valid
    if (s == "Imu")
        v = Imu;
    else if (s == "ImuRaw")
    	v = ImuRaw;
    else
        throw validation_error(validation_error::invalid_option_value);
}

CommandLineParser::CommandLineParser() :
			desc("Usage")
{

	desc.add_options() //
	("help,h", "produce help message") //
	("device,d", value<string>()->default_value("/dev/ttyACM0"), "sets the device to read") //
	("command,c", value<string>()->default_value("imuraw"), "sets the command to launch at startup") //
	("frequency,f", value<double>()->default_value(100), "sets the default frequency of messages") //
	("reader,r", value<SupportedReaders>()->default_value(ImuRaw, "ImuRaw"), "sets the message reader");

}

void CommandLineParser::parseCommandLine(int argc, char **argv)
{
	try
	{
		store(parse_command_line(argc, argv, desc), vm);
		if (vm.count("help"))
		{
			cout << desc << endl;
			return;
		}

		notify(vm);

	}
	catch (error& e)
	{
		cout << e.what() << endl;
		cout << desc << endl;
		return;
	}

}

string CommandLineParser::getInitialCommand()
{
	return vm["command"].as<string>() + "\r\n";
}

string CommandLineParser::getSerialDevice()
{
	return vm["device"].as<string>();
}

double CommandLineParser::getFrequency()
{
	return vm["frequency"].as<double>();
}

Reader* CommandLineParser::getReader(ros::NodeHandle& n)
{
	SupportedReaders reader = vm["reader"].as<SupportedReaders>();

	switch(reader)
	{
		case Imu:
			return new ImuRawReader(n);
		case ImuRaw:
			return new ImuReader(n);

		default:
			throw std::logic_error("This should not happen");
	}
}




