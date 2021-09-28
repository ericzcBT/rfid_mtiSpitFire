/* RFID_BT Copyright BrameTec (2020-2021)
 * This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>
 * */
#ifndef SERIALNC_H
#define SERIALNC_H

#include <stdint.h>
#include <stdio.h> // standard input / output functions
#include <stdlib.h>
#include <iostream>
#include <stdint.h>
#include <string.h>  // string function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions

class serialSpitFire
{
private:
    /* data */
    const char *serialPort;
    int serialFileDescriptor;
    struct timeval selectTimeout;
    fd_set read_fds, write_fds, except_fds;
    speed_t baudrate;

public:
    serialSpitFire(/* args */) : serialPort("/dev/ttyS0"), serialFileDescriptor(-1), baudrate(B115200){};
    serialSpitFire(const char *port, speed_t baud) : serialPort(port), serialFileDescriptor(-1), baudrate(baud){};
    serialSpitFire(const char *port) : serialPort(port), serialFileDescriptor(-1), baudrate(B115200){};
    ~serialSpitFire();

    void closeSerial();
    void setUsbTTY(char *devPath);
    int serialSetup();
    void serialSpitFireSetup(struct termios tty);
    bool available(int secondsTimeout, int uSecondsTimeout);
    void setserialPort(char *devPath);
    int writeByte(uint8_t *byte);
    int readByte(uint8_t *byte);

    
    void printConnectionStats();

};

#endif