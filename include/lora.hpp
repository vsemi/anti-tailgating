/**
 * Copyright (C) 2023 Visionary Semiconductor Inc.
 *
 * @{
 */
#ifndef SERIAL_CONNECTION_H
#define SERIAL_CONNECTION_H

#include <memory.h>
#include <vector>
#include <string>
#include <list>
#include <iostream>
#include <cstring>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <queue>

//! implementation of LoRaUart
/*!
* This class implements some specific functionality for the LoRaUart
*/
class LoRaUart
{    
public:
	LoRaUart();
	~LoRaUart();

	bool openConnection(std::string port);
	void closeConnection();

	size_t sendData(char *data, int len);
	int readData(int len);

	std::vector<char> rxArray;

private:
	std::string port;

	int fileDescription;

	int setInterfaceAttribs (int speed);
};

#endif // SERIAL_CONNECTION_H

/** @} */
