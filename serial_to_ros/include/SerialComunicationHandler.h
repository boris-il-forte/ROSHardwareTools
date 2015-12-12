/*
 * SerialToRos, Read stuff from serial and publish to ros
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Marcello Pogliani, Davide Tateo
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

#ifndef SERIALCOMUNICATION_H_
#define SERIALCOMUNICATION_H_

#include "CharCircularBuffer.h"

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include <exception>
#include <vector>
#include <string>

class SerialDeviceException: public std::exception
{
public:
	virtual const char* what() const throw ();
};

class SerialComunicationHandler
{
	enum WaitState
	{
		wait_ok = 1, wait_tout = 0, wait_err = -1
	};

public:
	SerialComunicationHandler(std::string serialDevice)
				throw (SerialDeviceException);
	int sendStringCommand(const char *cmd);
	std::string getLine();
	bool isReady();
	int readData();
	unsigned int getLineToParseNum();
	~SerialComunicationHandler();

private:
	WaitState waitData(int msec_tout);

private:
	int fd;  	//descrittore del file per leggere/scrivere sulla seriale
	termios oldtio, newtio;
	CharCircularBuffer * buffer;
	char * tmp_buf;
	static const int MAX_TMP_BUF = 256;
	struct pollfd ufd[1];

};

#endif
