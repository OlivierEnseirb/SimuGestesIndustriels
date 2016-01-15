#ifndef READSERIALCOM_H
#define READSERIALCOM_H
#include <OpenSim/OpenSim.h>
#include "IMU_maths.h"
#include "settings.h"
#include "uart_gestion.h"

#define BUFF_SIZE           16
#define NEW_SAMPLE_CLUE     '\n'
#define SEPARATING_CLUE     ','


class ReadSerialCom : public IMU_maths
{
    public:
        ReadSerialCom(){}
        ReadSerialCom(FILE* _file, DATA_FORMAT data_format, unsigned char _data_to_write, unsigned char _data_to_display, OpenSim::Model osimModel);
        ~ReadSerialCom();

        void initSerialCommunication(FILE* _file, DATA_FORMAT data_format, unsigned char _data_to_display, unsigned char _data_to_write);
        bool openSerialCommunication();
        void closeSerialCommunication();
        void readSerialCommunication();

        void appendNewDataToBuffer();
        bool waitNewDataClue(size_t& pos);
        void setNewSample(string& newSampleString);
        void convertStringToSample(string& newSampleString, Sample& _sample);

        Sample new_sample = getDefaultSample();
        Sample previous_sample = getDefaultSample();

        void setAssocietedFilePointer(FILE * _file){associatedFile = _file;}
        FILE* getAssocietedFilePointer(){return associatedFile;}
        void setDataFormat(DATA_FORMAT _data_format){data_format = _data_format;}
        DATA_FORMAT getDataFormat(){return data_format;}
		void setDataToDisplay(unsigned char _data_to_display) { data_to_display = _data_to_display; }
		unsigned char getDataToDisplay() { return data_to_display; }
		void setDataToWrite(unsigned char _data_to_write) { data_to_write = _data_to_write; }
		unsigned char getDataToWrite() { return data_to_write; }

    private:
        Settings* set_data = new Settings();
        UART_gestion* serial_com = new UART_gestion();

        FILE* associatedFile = NULL;
        DATA_FORMAT data_format = CSV;

        char buffer[BUFF_SIZE];//Déclaration d'un buffer qui reçoit les données
        string big_buffer;

        int nb_bytes_read;
        unsigned char data_to_display = 0;
		unsigned char data_to_write = 0;
        bool isSynchronised = false;
};

#endif // READSERIALCOM_H
