/**
* @file ComPort.h 
* @brief COM port header file
*/
#pragma once
#include <fcntl.h>
#include <string>
#include <vector>
#include <termios.h>

/**
* \~english A COM port class. Contains functions for opening and closing a port,
* writing and reading data, and a structure for saving port settings.
*/
class CCommPort  {
    public:
        //static const int kFlagsDefault = O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL;
        static const int kFlagsDefault = O_RDWR | O_NOCTTY  | O_EXCL;

        /**
        * \~english A public variable. Integer value used as a pointer to path for open a port(file).
        */
        int hCom;
        /** 
        * \~english A public variable. Integer value used to store settings for port.
        */
        struct termios dcb_struct;
        /**
        * \~english A constructor. Serves to set up a class and is called when an object of the class is created.
        */
        CCommPort();
        /**
        * \~english A destructor. A function that is called automatically when an object goes out of scope, or is explicitly destroyed by a call to delete.
        */
        virtual ~CCommPort();

        /**
        * \~english Open COM port for further interaction, taking 6 arguments as input and return one integer value.
        * @param    port_path an const std::string contains path to COM port.
        * @param    baud integer value of port speed.
        * @param    flags integer value of port flags for work.
        * @param    parity char value for parity case.
        * @param    data_bit integer value of data size.
        * @param    stop_bit intteger value of bit to stop.
        * @see      Close()
        * @return   Integer value that is 0 (fail) or 1 (success).
        * 
        */
        virtual int Open(const std::string& port_path, int baud, int flags, char parity, int data_bit, int stop_bit);

        /**
        * \~english Writes a block(vector) of data of a certain size to the port.
        * @param    write_vect char*, stores a block of data to write.
        * @param    write_count size_t value of size write_vect.
        * @see      ReadBlock()
        * @return   Integer value that is false(fail) or true(success).
        * 
        */
        virtual int WriteBlock(char* write_vect, size_t write_count);

        /**
        * \~english Reads a block(vector) of data of a certain size from the port.
        * @param    read_vect char*, saves a block of data received during a read operation.
        * @param    write_count size_t value of size read_vect.
        * @see      WriteBlock()
        * @return   Integer value that is false(fail) or true(success).
        * 
        */
        virtual int ReadBlock(char* read_vect, size_t read_count);

        /**
        * \~english Closes COM port.
        * @see      Open()
        * 
        */
	    virtual void Close();

    private:
        struct termios old_dcb;
};
