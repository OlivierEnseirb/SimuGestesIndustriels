#include "ReadSerialCom.h"

ReadSerialCom::ReadSerialCom(FILE* _file, DATA_FORMAT _data_format, unsigned char _data_to_write, unsigned char _data_to_display, OpenSim::Model osimModel)
{
    initSerialCommunication(_file, _data_format, _data_to_display, _data_to_write);
    if(openSerialCommunication())
    {
        size_t pos = 0;

		SimTK::State defaultState = osimModel.initSystem();
		osimModel.getVisualizer().show(defaultState);

        while(true)
        {
            if(_kbhit())
                break;
            readSerialCommunication();
            appendNewDataToBuffer();
            if(waitNewDataClue(pos))
            {
                string sub_str = big_buffer.substr(0,pos+1);
                big_buffer.erase(0,pos+1);
                if(isSynchronised)
                {
                    setNewSample(sub_str);
                    writeDataInFile(getAssocietedFilePointer(), getDataFormat(), new_sample, getDataToWrite());
                    new_sample.displaySample(getDataToDisplay());
                    QuaternionToEuler(new_sample.quaternion, new_sample.euler);

					osimModel.setStateVariable(defaultState, "r_helbow_xRotation", new_sample.euler.x);
					osimModel.setStateVariable(defaultState, "r_helbow_yRotation", new_sample.euler.y);
					osimModel.setStateVariable(defaultState, "r_helbow_zRotation", new_sample.euler.z);

					osimModel.getVisualizer().show(defaultState);
                }
                else
                {
                    isSynchronised = true;
                }
            }
        }
    }
}

ReadSerialCom::~ReadSerialCom()
{
    printf("Closing...\n");
    closeSerialCommunication();
}

void ReadSerialCom::initSerialCommunication(FILE* _file, DATA_FORMAT _data_format, unsigned char _data_to_display, unsigned char _data_to_write)
{
    setAssocietedFilePointer(_file);
    setDataFormat(_data_format);
    setDataToDisplay(_data_to_display);
	setDataToWrite(_data_to_write);

    writeHeaderInFile(getAssocietedFilePointer(), getDataFormat(), getDataToWrite());

    //lecture du fichier de config et récupération des données
    set_data->getSettings("init.txt");

    //configuration de la strucutre concernant la com série en fonction des données du fichier de config
    serial_com->SetDcbStructure(set_data->getBaud(), set_data->getNbBits(), set_data->getBitsStop(), set_data->getParity());
}

bool ReadSerialCom::openSerialCommunication()
{
    //ouverture du port com en fonction du numéro donné dans le fichier de config
    cout << "tentative d'ouverture du COM" << set_data->getComNumber() << endl;

    if( serial_com->OpenCOM(set_data->getComNumber())){
        //la communication série est ouverture
        cout << "COM Serie ouverte et prete a l'emploi" << endl;
        return true;
    }
    else
    {
        cout<<" La COM" << set_data->getComNumber() << " n'a pas pu être ouverte."<<endl;
        return false;
    }
}

void ReadSerialCom::closeSerialCommunication()
{
    cout << "fermeture de la communication" << endl;
    serial_com->CloseCOM();
}

void ReadSerialCom::readSerialCommunication()
{
    serial_com->ReadCOM(buffer, BUFF_SIZE, &nb_bytes_read);
}

void ReadSerialCom::setNewSample(string& newSampleString)
{
    previous_sample = new_sample;
    convertStringToSample(newSampleString, new_sample);
    new_sample.num_frame = previous_sample.num_frame + 1;
}

void ReadSerialCom::convertStringToSample(string& newSampleString, Sample& _sample)
{
    size_t pos = 0;
    float buf_float;
    Vec4 data(0.0, 0.0, 0.0, 0.0);
    size_t data_indice = 0;

    newSampleString += SEPARATING_CLUE;

    while(pos != newSampleString.npos)
    {
        data_indice++;
        pos = newSampleString.find_first_of(SEPARATING_CLUE);
        string sub_str = newSampleString.substr(0,pos+1);
        newSampleString.erase(0,pos+1);
        stringstream ss;
        ss << sub_str;
        ss >> buf_float;

        switch(data_indice)
        {
        case 1:
            data.w = buf_float;
            break;
        case 2:
            data.x = buf_float;
            break;
        case 3:
            data.y = buf_float;
            break;
        case 4:
            data.z = buf_float;
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
        big_buffer += buffer[i];
    }
}

bool ReadSerialCom::waitNewDataClue(size_t& pos)
{
    pos = big_buffer.find_first_of(NEW_SAMPLE_CLUE);
    return pos != big_buffer.npos;
}
