#ifndef READSERIALCOM_H
#define READSERIALCOM_H

#include "IMU_maths.h"
#include "settings.h"
#include "uart_gestion.h"


using namespace std;

#define BUFF_SIZE           16
#define NEW_SAMPLE_CLUE     '\n'
#define SEPARATING_CLUE     ','

class Multiple_Sensor_Reading;

class ReadSerialCom
{
    public:
        ReadSerialCom(){}
        ReadSerialCom(int _num_com, int _num_port); //num_port is the portof the computer where is the arduino
        ~ReadSerialCom();

        void initSerialCommunication(int _num_com, int _num_port);
        bool openSerialCommunication();
        void closeSerialCommunication();
        void readSerialCommunication();
		void launchSerialCom(Multiple_Sensor_Reading* msr);

        void appendNewDataToBuffer();
        bool waitNewDataClue(size_t& pos);
        void setNewSample(string& newSampleString);
        void convertStringToSample(string& newSampleString, Sample& _sample);

        Sample new_sample = defaultSample;
        Sample previous_sample = defaultSample;


		int getNumCom() { return num_com; }
		void setNumCom(int _nc) { num_com = _nc; }

		bool keep_processing = true;

    private:
		Settings* set_data = new Settings();
		UART_gestion* serial_com = new UART_gestion();


        char buffer[BUFF_SIZE];//Déclaration d'un buffer qui reçoit les données
        string big_buffer;

        int nb_bytes_read;
        bool isSynchronised = false;

		int num_com = 0;
};

#endif // READSERIALCOM_H
