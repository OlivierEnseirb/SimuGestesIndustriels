#include "Multiple_sensor_reading.h"
#include "ReadSerialCom.h"
std::mutex mtx;

void launchSerialCom(Multiple_Sensor_Reading* msr, ReadSerialCom* rsc)
{
	rsc->launchSerialCom(msr);
}


Multiple_Sensor_Reading::Multiple_Sensor_Reading(FILE* _data_file, DATA_FORMAT _df, unsigned char data_to_write, unsigned char data_to_display)
{
	setAssocietedFilePointer(_data_file);
	setDataFormat(_df);
	setDataToDisplay(data_to_display);
	setDataToWrite(data_to_write);

	// init com serie
	sens_com.push_back(new ReadSerialCom(0, 3));
	sens_thr.push_back(thread(launchSerialCom, this, (sens_com.back())));
	
	vector<string> temp;
	temp.push_back("r_shoulder_xRotation");
	temp.push_back("r_shoulder_yRotation");
	temp.push_back("r_shoulder_zRotation");
	sens_joint_link.push_back(temp);
	sens_samples.resize(1);

	writeHeaderInFile(getAssocietedFilePointer(), getDataFormat(), getDataToWrite());
}


void Multiple_Sensor_Reading::addNewSample(Sample& _s, int num_com)
{
	mtx.lock();
	sens_samples[num_com].push_back(_s);

	// a changer si on récupère directement les Euler ou autre
	QuaternionToEuler(sens_samples[num_com].back().quaternion, sens_samples[num_com].back().euler); // euler angles calculation from quaternions

	sens_samples[num_com].back().displaySample(getDataToDisplay());

	writeDataInFile(getAssocietedFilePointer(), getDataFormat(), sens_samples[num_com].back(), getDataToWrite());
	mtx.unlock();
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