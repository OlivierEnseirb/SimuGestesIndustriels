#include "Multiple_sensor_reading.h"
#include "ReadSerialCom.h"
std::mutex mtx;

void launchSerialCom(Multiple_Sensor_Reading* msr, ReadSerialCom* rsc, DATA_TYPE waiting_time)
{
	while (!(rsc->launchSerialCommunication(msr, waiting_time)))
	{
		cout << "Serial COM " << rsc->getNumPort() << " connection lost: try to reconnect it before "<< waiting_time<<" seconds." << endl;
		//rsc->closeSerialCommunication();
	}
	rsc->closeSerialCommunication();
}

Multiple_Sensor_Reading::Multiple_Sensor_Reading(FILE* _data_file, DATA_FORMAT _df, unsigned char data_to_write, unsigned char data_to_display, DATA_TYPE waiting_time)
{
	setAssocietedFilePointer(_data_file);
	setDataFormat(_df);
	setDataToDisplay(data_to_display);
	setDataToWrite(data_to_write);

	// init com serie
	size_t nbr_sensors = 2;
	vector<string> temp_link_joint;
	size_t port_number;
	string com_name;

	/* TO DO
	 * a modifier en qqch prenant un fichier d'init pour relier un capteur à un marqueur/articulation et un port
	 */
	//for (size_t num_thread = 0; num_thread < nbr_sensors; num_thread++) {

		com_name = "shoulder";
		port_number = 3;
		temp_link_joint.clear();
		temp_link_joint.push_back("r_shoulder_xRotation");
		temp_link_joint.push_back("r_shoulder_yRotation");
		temp_link_joint.push_back("r_shoulder_zRotation");
		addSerialCommunication(com_name, port_number, temp_link_joint, waiting_time);

		com_name = "back";
		port_number = 4;
		temp_link_joint.clear();
		temp_link_joint.push_back("r_helbow_xRotation");
		temp_link_joint.push_back("r_helbow_yRotation");
		temp_link_joint.push_back("r_helbow_zRotation");
		addSerialCommunication(com_name, port_number, temp_link_joint, waiting_time);
	//}

	sens_samples.resize(nbr_sensors);
	writeHeaderInFile(getAssocietedFilePointer(), getDataFormat(), getDataToWrite());
}

void Multiple_Sensor_Reading::addNewSample(Sample& _s, size_t num_thread)
{
	mtx.lock();
	sens_samples[num_thread].push_back(_s);

	// a changer si on récupère directement les Euler ou autre
	//QuaternionToEuler(sens_samples[num_thread].back().quaternion, sens_samples[num_thread].back().euler); // euler angles calculation from quaternions

	sens_samples[num_thread].back().displaySample(getDataToDisplay());

	writeDataInFile(getAssocietedFilePointer(), getDataFormat(), sens_samples[num_thread].back(), getDataToWrite());
	mtx.unlock();
}

void Multiple_Sensor_Reading::closeThread(size_t num_thread)
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

void Multiple_Sensor_Reading::addSerialCommunication(string& com_name, size_t _num_port, vector<string>& joint_link, DATA_TYPE waiting_time)
{
	sens_name.push_back(com_name);
	sens_joint_link.push_back(joint_link);
	sens_com.push_back(new ReadSerialCom(sens_com.size(), _num_port));
	sens_thr.push_back(thread(launchSerialCom, this, sens_com.back(), waiting_time));
}

bool Multiple_Sensor_Reading::findThreadByPortNumber(const size_t num_port, size_t& pos)
{
	for (size_t i = 0; i < sens_com.size(); i++)
	{
		if (sens_com[i]->getNumPort() == num_port)
		{
			pos = i;
			return true;
		}
	}
	return false;
}

bool Multiple_Sensor_Reading::findThreadByName(const string& name, size_t& pos)
{
	for (size_t i = 0; i < sens_name.size(); i++)
	{
		if (sens_name[i].compare(name) == 0)
		{
			pos = i;
			return true;
		}
	}
	return false;
}