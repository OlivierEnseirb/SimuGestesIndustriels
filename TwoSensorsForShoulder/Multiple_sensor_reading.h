/***************************************
* Author : Olivier Hartmann (janv. 2015)
* This object is used to get data from several sensors
***************************************/

#ifndef MULTIPLE_SENSOR_READING_H
#define MULTIPLE_SENSOR_READING_H

#include <stdio.h>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <mutex>  // for access rights to a variable
#include "IMU_maths.h"

class ReadSerialCom; // class declared somewhere else and included in .cpp

class Multiple_Sensor_Reading : public IMU_maths
{
	public:
		Multiple_Sensor_Reading() {}
		~Multiple_Sensor_Reading() {}
		Multiple_Sensor_Reading(FILE* _data_file, DATA_FORMAT _df, unsigned char data_to_write, unsigned char data_to_display, DATA_TYPE waiting_time);

		void initializationFromFile(string& init_file_name, DATA_TYPE waiting_time); // initialize the class from an init file

		void addNewSample(Sample& _s, size_t num_thread); // add a new sample coming from the thread number num_thread. A mutex avoid variable access collisions
		void closeThread(size_t num_thread); // close the thread number num_thread
		void stopReading(); // stop all communications
		void addSerialCommunication(string& com_name, size_t _num_port, vector<string>& joint_link, DATA_TYPE waiting_time); //create a new serial communication on the hardware port "_num_port", linked with the joints "joint_link"
		
		bool findThreadByPortNumber(const size_t num_port, size_t& pos); // return true if the thread has been found by its port number, and set its position in "pos"
		bool findThreadByName(const string& name, size_t& pos); // return true if the thread has been found by its name, and set its position in "pos"

		void setAssocietedFilePointer(FILE * _file) { associatedFile = _file; }
		FILE* getAssocietedFilePointer() { return associatedFile; }
		void setDataFormat(DATA_FORMAT _data_format) { data_format = _data_format; }
		DATA_FORMAT getDataFormat() { return data_format; }
		void setDataToDisplay(unsigned char _data_to_display) { data_to_display = _data_to_display; }
		unsigned char getDataToDisplay() { return data_to_display; }
		void setDataToWrite(unsigned char _data_to_write) { data_to_write = _data_to_write; }
		unsigned char getDataToWrite() { return data_to_write; }


		/* TO DO
		 * faire un vecteur de structure pour regrouper ces 5 vecteurs.
		 * exemple : 
		 * struct Read_Sens_struct{
				thread sens_thr;
				string sens_name;
				ReadSerialCom* sens_com;
				vector<string> sens_joint_link;
				vector<Sample> sens_samples;};
			vector<Read_Sens_struct> vect_sensors; //et alors tout ce qui concerne un capteur sera regroupé dans une même structure et pas séparé en trois vecteurs
		 */
		vector<thread> sens_thr; // list of launched threads : 1 thread by sensor
		vector<string> sens_name; // name of the matching communication
		vector<ReadSerialCom*> sens_com; // Communication manager of the matching thread : 1 element by thread
		vector<vector<string> > sens_joint_link;// joints associated with the sensor : 1 articulation (3 DOF) by thread/sensor
		vector<vector<Sample> > sens_samples;// current samples of threads : a vector for each to not overwrite not yet processed Samples

	private:
		FILE* associatedFile = NULL; //associated file pointer where to write data
		DATA_FORMAT data_format = CSV; //format in which data will be written
		unsigned char data_to_display = 0; // data that will be displayed while reading
		unsigned char data_to_write = 0; // data that will be written in the file pointed with "associatedFile"
};

#endif //MULTIPLE_SENSOR_READING_H