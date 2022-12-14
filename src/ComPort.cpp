/**
* @file ComPort.cpp 
* @brief Cpp file with code of functions for work with COM port. 
*/
#include <pthread.h>
#include <errno.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <stdint.h>
#include <stdexcept>
#include "ComPort.h"
#include <chrono>
#include <thread>
#include <QDebug>

CCommPort::CCommPort(): hCom(-1)  {
}

CCommPort::~CCommPort() {
    tcsetattr(hCom, TCSANOW, &old_dcb);
    close(hCom);
    hCom = -1;
}	

int CCommPort::Open(const std::string& port_path, int baud, int flags, char parity, int data_bit, int stop_bit) {
        speed_t speed;

        qDebug() << "Open " << QString::fromStdString(port_path);

        if (port_path.empty()) {
            return 0;  
        }

        std::string port = port_path;

        if ((hCom = open(port.data(),
                         (flags == 0)?kFlagsDefault:flags)) < 0) {
            return 0;  
        }

        tcflush(hCom, TCIOFLUSH);
        std::memset((void*)&dcb_struct, 0, (std::size_t)sizeof(struct termios));
        switch (baud) {
            case 110:
                speed = B110;
                break;
            case 300:
                speed = B300;
                break;
            case 600:
                speed = B600;
                break;
            case 1200:
                speed = B1200;
                break;
            case 2400:
                speed = B2400;
                break;
            case 4800:
                speed = B4800;
                break;
            case 9600:
                speed = B9600;
                break;
            case 19200:
                speed = B19200;
                break;
            case 38400:
                speed = B38400;
                break;
            case 57600:
                speed = B57600;
                break;
            case 115200:
                speed = B115200;
                break;
            default: {
                close(hCom);
                hCom = -1;
                return 0;  
                }
        }
    
        if ((cfsetispeed(&dcb_struct, speed) < 0) || (cfsetospeed(&dcb_struct, speed) < 0)) {
            close(hCom);
            hCom = -1;
            return 0;    
        }

        dcb_struct.c_cflag |= (CREAD | CLOCAL);
        dcb_struct.c_cflag &= ~CSIZE;

        switch (data_bit) {
            case 5:
                dcb_struct.c_cflag |= CS5;
                break;
            case 6:
                dcb_struct.c_cflag |= CS6;
                break;
            case 7:
                dcb_struct.c_cflag |= CS7;
                break;
            case 8:
            default:
                dcb_struct.c_cflag |= CS8;
                break;
        }           

        if (stop_bit == 1) {
                dcb_struct.c_cflag &=~ CSTOPB;
        } else {
                dcb_struct.c_cflag |= CSTOPB;
        }

        if (parity == 'N') {
            dcb_struct.c_cflag &=~ PARENB;
        } else if (parity == 'E') {
            dcb_struct.c_cflag |= PARENB;
            dcb_struct.c_cflag &=~ PARODD;
        } else {
             dcb_struct.c_cflag |= PARENB;
             dcb_struct.c_cflag |= PARODD;
        }

        dcb_struct.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        if (parity == 'N') {
            dcb_struct.c_iflag &= ~INPCK;
        } else {
            dcb_struct.c_iflag |= INPCK;
        }

        dcb_struct.c_iflag &= ~(IXON | IXOFF | IXANY);
        dcb_struct.c_oflag &=~ OPOST;
        dcb_struct.c_cc[VMIN] = 0;
        dcb_struct.c_cc[VTIME] = 0;

        if (tcsetattr(hCom, TCSANOW, &dcb_struct) < 0) {
            close(hCom);
            hCom = -1;
            return 0;
        }

        qDebug() << "Open success";


        return 1;
}

void CCommPort::Close()  {
    qDebug() << "Close serial";
    	if(hCom == -1) {
        } else {
            int DTR_flag;
            DTR_flag = TIOCM_DTR;
            fcntl(hCom, F_SETFL, 0);
            ioctl(hCom, TIOCMBIC, &DTR_flag);
            tcflush(hCom, TCIOFLUSH);
            close(hCom);
            hCom = -1;
            qDebug() << "Device close successful...";
        }
}

int CCommPort::ReadBlock(char* read_vect, size_t read_count)    {
    int ret = 0;
    auto time = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
    while (ret != read_count ) {
        if(read(hCom, &read_vect[ret], 1) == 1) {
           ret++;
        }
    
        if(time <  std::chrono::system_clock::now()) {
            printf("Read time out!\n");
            return false;
        }
    }
    return true;   
}

int CCommPort::WriteBlock(char* write_vect, size_t write_count)  {
    if (write(hCom, write_vect, write_count) != write_count) {
        return false;
    }
    return true;
}

