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

#include "../include/Reader.h"

#include <boost/tokenizer.hpp>

Reader::Reader(ros::NodeHandle& n) :
			n(n)
{

}

Reader::~Reader()
{

}

void Reader::tokenize(const std::string& line, const std::string& sepChar,
			std::vector<std::string>& tokenVec)
{
	boost::char_separator<char> sep(sepChar.c_str());
	boost::tokenizer<boost::char_separator<char> > tokens(line, sep);

	unsigned int n = 0;
	for (std::string token : tokens)
	{
		if (n > 9)
			throw std::runtime_error("Too long");

		tokenVec.push_back(token);
		n++;
	}

	if (n != 9)
		throw std::runtime_error("too short");
}

