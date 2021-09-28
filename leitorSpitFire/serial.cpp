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
#include "serial.h"



serialSpitFire::~serialSpitFire()
{
    if (serialFileDescriptor != -1)
    {
        this->closeSerial();
        serialFileDescriptor = -1;
    }
}


void serialSpitFire::printConnectionStats(){
    std::cout << "Serial:" << std::string(this->serialPort) << std::endl
    << "FD: " << this->serialFileDescriptor << std::endl
    << "BAUD" << this->baudrate << std::endl;
}

void serialSpitFire::closeSerial()
{
    close(serialFileDescriptor);
    serialFileDescriptor = -1;
}

void serialSpitFire::setserialPort(char *devPath){
    serialPort = devPath;
}

int serialSpitFire::serialSetup() {
    std::cout << "Opening file descriptor on " << serialPort << std::endl;
    serialFileDescriptor = open(serialPort, O_RDWR);

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    /* Error Handling */
    if (tcgetattr(serialFileDescriptor, &tty) != 0) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return -1;
    }
    //tcsetattr(serialFileDescriptor, TCSAFLUSH, &tty);
    sleep(2);
    //tcflush(serialFileDescriptor, TCIOFLUSH);
    /* Set Baud Rate */
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    /* flush the thingy */
    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB;  // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;  // no flow control
    tty.c_lflag = 0;          // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;          // no remapping, no delays
    tty.c_cc[VMIN] = 0;       // no min bytes read
    tty.c_cc[VTIME] = 20;      // deciseconds read timeout

    tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // turn off s/w flow ctrl
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    tty.c_oflag &= ~OPOST;                          // make raw
    //tty.c_oflag &= ~ONCLR;                          // make raw
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    /* Flush Port, then applies attributes */
    tcflush(serialFileDescriptor, TCIFLUSH);

    if (tcsetattr(serialFileDescriptor, TCSANOW, &tty) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return -1;
    }
    std::cout << "Termios set"<< std::endl;
    return 0;
}


void serialSpitFire::serialSpitFireSetup(struct termios tty)
{
    tcflush(serialFileDescriptor, TCIFLUSH);
    if (tcsetattr(serialFileDescriptor, TCSANOW, &tty) != 0)
    {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
}

int serialSpitFire::writeByte(uint8_t *byte)
{
    //cout << "Writing...." << std::endl;
    return write(serialFileDescriptor, byte, 1);
}

int serialSpitFire::readByte(uint8_t *byte)
{
    //cout << "Sleep to debug Read..." << std::endl;
    //sleep(2);
    return read(serialFileDescriptor, byte, 1);
}

bool serialSpitFire::available(int secondsTimeout, int uSecondsTimeout)
{

    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_ZERO(&except_fds);
    FD_SET(serialFileDescriptor, &read_fds);

    selectTimeout.tv_sec = secondsTimeout;
    selectTimeout.tv_usec = uSecondsTimeout;
    //usleep(1000);
    ///*
    // If there is data to be read
    if (select(serialFileDescriptor + 1, &read_fds, &write_fds, &except_fds, &selectTimeout) == 1)
    {
        return true;
    }
    return false;
}

