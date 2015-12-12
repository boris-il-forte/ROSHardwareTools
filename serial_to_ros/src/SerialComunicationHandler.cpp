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

#include "SerialComunicationHandler.h"

#include <sstream>
#include <errno.h>
#include <iostream>

#define BAUDRATE 		B115200
#define TOUT			300 //msec
#define MAX_C_RECV 		40
#define CHAR_PAUSE 		3000 //usec
#define MAX_BUF_CHAR	32767

using namespace std;

const char* SerialDeviceException::what() const throw ()
{
	return "Serial port open error!";
}

SerialComunicationHandler::SerialComunicationHandler(std::string serialDevice)
			throw (SerialDeviceException)
{
	fd = open(serialDevice.c_str(), O_RDWR | O_NOCTTY);
	if (fd >= 0)
	{
		tcgetattr(fd, &oldtio);
		bzero(&newtio, sizeof(termios));
		newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD | CLOCAL;
		newtio.c_iflag = 0;
		newtio.c_oflag = 0;
		newtio.c_lflag &= ~ICANON;

		ufd[0].fd = fd;
		ufd[0].events = POLLIN;

		newtio.c_cc[VTIME] = 0;
		newtio.c_cc[VMIN] = 1;

		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);
	}
	else
	{
		throw SerialDeviceException();
	}

	tmp_buf = new char[MAX_TMP_BUF];
	buffer = new CharCircularBuffer(MAX_BUF_CHAR, "\r\n");
}

SerialComunicationHandler::~SerialComunicationHandler()
{
	if (fd >= 0)
	{
		close(fd);
	}
	if (buffer)
	{
		delete buffer;
	}
	if (tmp_buf)
	{
		delete[] tmp_buf;
	}
}

bool SerialComunicationHandler::isReady()
{
	return fd >= 0;
}

int SerialComunicationHandler::readData()
{
	if (fd < 0)
	{
		return -1;
	}

	int res = 0;
	unsigned int count_c = 0;

	do
	{
		switch (waitData(TOUT))
		{
			case wait_ok:
				res = read(fd, tmp_buf, MAX_TMP_BUF);
				if (res <= 0)
				{
					cerr << "(1) READ Error on fileno " << fd << endl;
					res = -1;
				}
				if (buffer->addNChar(tmp_buf, res) != res)
				{
					count_c = count_c + res;
					cerr << "(1) BUF_FULL on fileno " << fd << endl;
					res = -1;
				}
				break;
			case wait_err:
				cerr << "(1) Error on fileno " << fd << endl;
				res = -1;
				break;
			case wait_tout:
				cerr << "(1) TOUT on fileno " << fd << endl;
				res = -1;
				break;
		}

		if (res == -1)
			break;

	} while (count_c < MAX_C_RECV && buffer->getLineCount() <= 0);

	if (res <= 0)
		return -1;
	return 0;
}

std::string SerialComunicationHandler::getLine()
{
	if (buffer->getLineCount() == 0)
		return "Error";
	int len = buffer->removeLine(tmp_buf, MAX_TMP_BUF);
	if (len <= 0)
		return "";
	if (tmp_buf[len - 1] == '\n')
		tmp_buf[len - 1] = '\0';
	return tmp_buf;
}

unsigned int SerialComunicationHandler::getLineToParseNum()
{
	return buffer->getLineCount();
}

int SerialComunicationHandler::sendStringCommand(const char *cmd)
{
	std::cerr << cmd << std::endl;
	if (fd < 0)
		return -1;
	int len = strlen(cmd);
	for (int i = 0; i < len; i++)
	{
		if (write(fd, &cmd[i], 1) == -1)
		{
			return -1;
			perror("Error writing to the serial port!");
		}
		usleep(CHAR_PAUSE);
	}
	usleep(500000);
	return 0;
}

SerialComunicationHandler::WaitState SerialComunicationHandler::waitData(int msec_tout)
{
	bool rexec;
	int v;
	int fd = ufd[0].fd;

	do
	{
		rexec = false;
		v = poll(ufd, 1, msec_tout);
		if (v < 0)
		{
			if (errno == EINTR)
			{
				fprintf(stderr, "poll EINTR on file %d\n", fd);
				rexec = true;
			}
			else
			{
				fprintf(stderr, "poll error on file %d,errno=%d\n", fd, errno);
				return wait_err;
			}
		}
		else if (v == 0)
		{
			return wait_tout;
		}
	} while (rexec);

	return wait_ok;
}
