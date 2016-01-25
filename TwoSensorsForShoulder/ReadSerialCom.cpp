#include "ReadSerialCom.h"
#include "Multiple_sensor_reading.h"

ReadSerialCom::ReadSerialCom(size_t _num_com, size_t _num_port)
{
    initSerialCommunication(_num_com, _num_port);
}

ReadSerialCom::~ReadSerialCom()
{
    printf("Closing...\n");
    closeSerialCommunication();
}

bool ReadSerialCom::launchSerialCommunication(Multiple_Sensor_Reading* msr, DATA_TYPE waiting_time)
{ 
	if (openSerialCommunication(waiting_time))
	{
		size_t pos = 0;

		while (keep_processing)
		{
			if (!readSerialCommunication())
				return false;

			if (!checkGoodCommunication()) {
				cout << "Time to long between two communications on Serial COM "<< getNumPort()<<"." << endl;
				return false;
			}

			appendNewDataToBuffer();
			if (waitEndDataClue(pos))
			{
				string sub_str = big_buffer.substr(0, pos + 1);
				big_buffer.erase(0, pos + 1);
				if (isSynchronised)
				{
					setNewSample(sub_str);
					msr->addNewSample(new_sample, getNumThread());
				}
				else
				{
					isSynchronised = true;
				}
			}
		}
		return true;
	}
	return true;
}

void ReadSerialCom::initSerialCommunication(size_t _num_thread, size_t _num_port)
{
	setNumThread(_num_thread);

    //lecture du fichier de config et récupération des données
	serial_com->getSettings("init.txt");
	serial_com->setComNumber((int)_num_port);

    //configuration de la strucutre concernant la com série en fonction des données du fichier de config
	serial_com->SetDcbStructure();// serial_com->getBaud(), serial_com->getNbBits(), serial_com->getBitsStop(), serial_com->getParity());
}

bool ReadSerialCom::openSerialCommunication(DATA_TYPE waiting_time)
{
    //ouverture du port com en fonction du numéro donné dans le fichier de config
    cout << "tentative d'ouverture du COM" << serial_com->getComNumber() << endl;

	time_t init_time, current_time;
	time(&init_time); //get time at the begining of the function

	string error_message;

	while (!(serial_com->OpenCOM(serial_com->getComNumber(), error_message)))
	{
		if (!keep_processing) // check if the thread has not been asked to be closed
			return false;

		time(&current_time);
		if (difftime(current_time, init_time) > waiting_time)
		{
			cout << "COM Serie " << serial_com->getComNumber() << " n'a pas pu etre ouverte apres " << waiting_time <<" senconds." << endl;
			cout << "Message d'erreur : " << error_message << endl;
			return false;
		}
	}
	time(&last_time_data_obtained);
	cout << "COM Serie "<< serial_com->getComNumber() << " ouverte et prete a l'emploi." << endl;
	return true;
}

void ReadSerialCom::closeSerialCommunication()
{
	cout << "COM Serie " << serial_com->getComNumber() << " fermee." << endl;
    serial_com->CloseCOM();
}

bool ReadSerialCom::readSerialCommunication()
{
    return serial_com->ReadCOM(serial_buffer, BUFF_SIZE, &nb_bytes_read);
}

void ReadSerialCom::setNewSample(string& newSampleString)
{
    previous_sample = new_sample;
    convertStringToSample(newSampleString, new_sample);
    new_sample.num_frame = previous_sample.num_frame + 1;
	/* TO DO
	 * ajouter timestamp au samples
	 * calculer diff_time_between_samples
	 * new_sample.timestamp = previous_sample.timestamp + diff_time_between_samples;
	 */
}

void ReadSerialCom::convertStringToSample(string& newSampleString, Sample& _sample)
{
	/* TO DO
	 * prendre en compte, éventuellement, le timestamp dans le protocole et le type de donnée en entrée (QUATERNION ou ACCELERATION ou EULER ...)
	 */
    size_t pos = 0;
    DATA_TYPE buf_realNumber;
    Vec4 data(0.0, 0.0, 0.0, 0.0);
    size_t data_indice = 0;

    newSampleString += SEPARATING_CLUE; // add this clue to take into account the last data

    while(pos != newSampleString.npos) // while a new data is found
    {
        data_indice++;
        pos = newSampleString.find_first_of(SEPARATING_CLUE);
        string sub_str = newSampleString.substr(0,pos+1);
        newSampleString.erase(0,pos+1);
        
		stringstream ss; 
        ss << sub_str;
        ss >> buf_realNumber; //convert string to real number of the same type than buf_realNumber

        switch(data_indice)
        {
        case 1:
            data.w = buf_realNumber;
            break;
        case 2:
            data.x = buf_realNumber;
            break;
        case 3:
            data.y = buf_realNumber;
            break;
        case 4:
            data.z = buf_realNumber;
            break;
        default:
            break;
        }
    }
    _sample.quaternion = data;
}

void ReadSerialCom::appendNewDataToBuffer()
{
    for(int i = 0; i<nb_bytes_read; i++)
    {
        big_buffer += serial_buffer[i];
    }
}

bool ReadSerialCom::waitEndDataClue(size_t& pos)
{
    pos = big_buffer.find_first_of(END_SAMPLE_CLUE);
    return pos != big_buffer.npos;
}

bool ReadSerialCom::checkGoodCommunication()
{
	time_t current_time;
	time(&current_time);
	bool temp = (difftime(current_time, last_time_data_obtained) < LIMIT_TIME_BETWEEN_TWO_RECEPTIONS);
	if(nb_bytes_read)
		last_time_data_obtained = current_time;
	return temp;
}