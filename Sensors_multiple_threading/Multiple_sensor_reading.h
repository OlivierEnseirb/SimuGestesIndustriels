#ifndef MULTIPLE_SENSOR_READING_H
#define MULTIPLE_SENSOR_READING_H

#include <stdio.h>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <mutex>
#include "IMU_maths.h"

class ReadSerialCom;

class Multiple_Sensor_Reading : public IMU_maths
{
	public:
		Multiple_Sensor_Reading() {}
		~Multiple_Sensor_Reading() {}
		Multiple_Sensor_Reading(FILE* _data_file, DATA_FORMAT _df, unsigned char data_to_write, unsigned char data_to_display);

		void addNewSample(Sample& _s, int num_com);
	
		void stopReading();

		void setAssocietedFilePointer(FILE * _file) { associatedFile = _file; }
		FILE* getAssocietedFilePointer() { return associatedFile; }
		void setDataFormat(DATA_FORMAT _data_format) { data_format = _data_format; }
		DATA_FORMAT getDataFormat() { return data_format; }
		void setDataToDisplay(unsigned char _data_to_display) { data_to_display = _data_to_display; }
		unsigned char getDataToDisplay() { return data_to_display; }
		void setDataToWrite(unsigned char _data_to_write) { data_to_write = _data_to_write; }
		unsigned char getDataToWrite() { return data_to_write; }

		vector<thread> sens_thr;
		vector<ReadSerialCom*> sens_com;
		vector<vector<string> > sens_joint_link;
		vector<vector<Sample> > sens_samples;

	private:

		FILE* associatedFile = NULL;
		DATA_FORMAT data_format = CSV;
		unsigned char data_to_display = 0;
		unsigned char data_to_write = 0;
};

#endif //MULTIPLE_SENSOR_READING_H