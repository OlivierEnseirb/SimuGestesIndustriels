#ifndef READSERIALCOM_H
#define READSERIALCOM_H

#include <ctime>
#include "IMU_maths.h"
#include "settings.h"
#include "uart_gestion.h"


using namespace std;

#define BUFF_SIZE           16
#define NEW_SAMPLE_CLUE     '\n'
#define SEPARATING_CLUE     ','

class Multiple_Sensor_Reading; // class declared somewhere else and included in .cpp

class ReadSerialCom
{
    public:
        ReadSerialCom(){}
        ReadSerialCom(int _num_thread, int _num_port); //num_port is the portof the computer where is the arduino
        ~ReadSerialCom();

        void initSerialCommunication(int _num_com, int _num_port);
        bool openSerialCommunication(double waiting_time);
        void closeSerialCommunication();
        bool readSerialCommunication();
		bool launchSerialCommunication(Multiple_Sensor_Reading* msr, double waiting_time);

        void appendNewDataToBuffer();
        bool waitNewDataClue(size_t& pos);
        void setNewSample(string& newSampleString);
        void convertStringToSample(string& newSampleString, Sample& _sample);
		bool checkGoodCommunication(double limit_time);

        Sample new_sample = defaultSample;
        Sample previous_sample = defaultSample;

		int getNumThread() { return num_thread; }
		void setNumThread(int _nt) { num_thread = _nt; }
		int getNumPort() { return set_data->getComNumber(); }

		bool keep_processing = true;

    private:
		Settings* set_data = new Settings();
		UART_gestion* serial_com = new UART_gestion();


        char buffer[BUFF_SIZE];//Déclaration d'un buffer qui reçoit les données
        string big_buffer;

        int nb_bytes_read;
        bool isSynchronised = false;

		int num_thread = 0;
		time_t last_time_data_obtained;
};

#endif // READSERIALCOM_H
