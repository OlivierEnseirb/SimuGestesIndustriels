/***************************************
* Author : Olivier Hartmann (dec. 2015)
* This object is used to get data from a sensor by a serial communication (UART)
***************************************/

#ifndef READSERIALCOM_H
#define READSERIALCOM_H

#include <ctime>
#include "IMU_maths.h"
#include "settings.h"
#include "uart_gestion.h"


using namespace std;

#define BUFF_SIZE           16
#define END_SAMPLE_CLUE     '\n' // character of the end of the protocol of communication
#define SEPARATING_CLUE     ',' // character seperating the data in the protocol of com
#define LIMIT_TIME_BETWEEN_TWO_RECEPTIONS   5 //seconds

class Multiple_Sensor_Reading; // class declared somewhere else and included in .cpp

class ReadSerialCom
{
    public:
        ReadSerialCom(){}
        ReadSerialCom(size_t _num_thread, size_t _num_port); //num_port is the port of the computer where is the arduino ; num_thread is the id of the communication in the program
        ~ReadSerialCom();

        void initSerialCommunication(size_t _num_com, size_t _num_port); //initialize the communication from the init file and its arguments
        bool openSerialCommunication(DATA_TYPE waiting_time); // open the serial communication, and try it for waiting_time seconds
        void closeSerialCommunication(); // close the serial port
        bool readSerialCommunication(); // read the serial port and wait for data
		bool launchSerialCommunication(Multiple_Sensor_Reading* msr, DATA_TYPE waiting_time); // main function reading and processing data from serial com
				//return false when communication is lost else true at the end of the function

        void appendNewDataToBuffer(); // adds "serial_buffer" to "big_buffer"
        bool waitEndDataClue(size_t& pos); // indicates if END_SAMPLE_CLUE caracter has been received and give the position in "big_buffer"
        void setNewSample(string& newSampleString); //set new_sample with the data given as a string
        void convertStringToSample(string& newSampleString, Sample& _sample); // convert data as string size_to a "Sample" object
		bool checkGoodCommunication(); // watch the time between two communications if it is not too long

        Sample new_sample = defaultSample; // variable where is stocked the new sample
        Sample previous_sample = defaultSample; // variable where is stocked the sample arrived before the new one

		size_t getNumThread() { return num_thread; }
		void setNumThread(size_t _nt) { num_thread = _nt; }
		size_t getNumPort() { return (size_t)serial_com->getComNumber(); }

		bool keep_processing = true; // indicates if the serial port has to be reading anymore

    private:
		UART_gestion* serial_com = new UART_gestion(); // object used to read the serial com


        char serial_buffer[BUFF_SIZE]; // buffer that gets data directly from the serial port
        string big_buffer; //buffer where are stocked new data from serial port, via "serial_buffer"

        int nb_bytes_read; // nbr of bytes read by the serial port
        bool isSynchronised = false; // indicates if the communication is synchronised on the communication protocol

		size_t num_thread = 0; // number of the thread given to identify itself when returning new samples
		time_t last_time_data_obtained; // indicates when the last data has been obtained
};

#endif // READSERIALCOM_H
