#include "Multiple_sensor_reading.h"
#include "ReadSerialCom.h"
std::mutex mtx;

void launchSerialCom(Multiple_Sensor_Reading* msr, ReadSerialCom* rsc, double waiting_time)
{
	while (!(rsc->launchSerialCommunication(msr, waiting_time)))
	{
		cout << "Serial COM " << rsc->getNumPort() << " connection lost: try to reconnect it before "<< waiting_time<<" seconds." << endl;
		//rsc->closeSerialCommunication();
	}
	rsc->closeSerialCommunication();
}

Multiple_Sensor_Reading::Multiple_Sensor_Reading(FILE* _data_file, DATA_FORMAT _df, unsigned char data_to_write, unsigned char data_to_display, double waiting_time)
{
	setAssocietedFilePointer(_data_file);
	setDataFormat(_df);
	setDataToDisplay(data_to_display);
	setDataToWrite(data_to_write);

	// init com serie
	size_t nbr_sensors = 2;
	vector<string> temp;
	
	//for (size_t num_thread = 0; num_thread < nbr_sensors; num_thread++) {
		// a modifier en qqch prenant un fichier d'init pour relier un capteur à un marqueur/articulation
		temp.clear();
		//1ère centrale IMU
		sens_com.push_back(new ReadSerialCom(0, 3));

		temp.push_back("r_shoulder_xRotation");
		temp.push_back("r_shoulder_yRotation");
		temp.push_back("r_shoulder_zRotation");
		sens_joint_link.push_back(temp);

		sens_thr.push_back(thread(launchSerialCom, this, (sens_com.back()), waiting_time));

		// 2ème centrale IMU
		sens_com.push_back(new ReadSerialCom(1, 4));

		temp.clear();
		temp.push_back("r_helbow_xRotation");
		temp.push_back("r_helbow_yRotation");
		temp.push_back("r_helbow_zRotation");
		sens_joint_link.push_back(temp);

		sens_thr.push_back(thread(launchSerialCom, this, (sens_com.back()), waiting_time));
	//}

	sens_samples.resize(nbr_sensors);
	writeHeaderInFile(getAssocietedFilePointer(), getDataFormat(), getDataToWrite());
}

void Multiple_Sensor_Reading::addNewSample(Sample& _s, int num_thread)
{
	mtx.lock();
	sens_samples[num_thread].push_back(_s);

	// a changer si on récupère directement les Euler ou autre
	QuaternionToEuler(sens_samples[num_thread].back().quaternion, sens_samples[num_thread].back().euler); // euler angles calculation from quaternions

	sens_samples[num_thread].back().displaySample(getDataToDisplay());

	writeDataInFile(getAssocietedFilePointer(), getDataFormat(), sens_samples[num_thread].back(), getDataToWrite());
	mtx.unlock();
}

void Multiple_Sensor_Reading::closeThread(int num_thread)
{
	if (num_thread < sens_com.size())
	{
		sens_com[num_thread]->keep_processing = false;
		sens_thr[num_thread].join();
	}
	else
	{
		cerr << "Error in Multiple_Sensor_Reading::closeThread : num_thread higher than number of thread" << endl;
	}
}

void Multiple_Sensor_Reading::stopReading()
{
	for (size_t i = 0; i < sens_com.size(); i++)
	{
		sens_com[i]->keep_processing = false;
	}
	for (size_t i = 0; i < sens_thr.size(); i++)
	{
		sens_thr[i].join();
	}
}