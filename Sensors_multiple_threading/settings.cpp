#include "settings.h"


Settings::~Settings(){
}

int Settings::getFosc(){
    return fosc;
}

int Settings::getBaud(){
    return baud;
}

int Settings::getNbBits(){
    return nb_bits;
}

int Settings::getBitsStop(){
    return bits_stop;
}

int Settings::getParity(){
    return parity;
}

int Settings::getComNumber(){
    return com;
}

void Settings::getSettings(const string &filename){
    string buf, cle, value;
    size_t pos;

    cout << "Start getting the settings for the serial communication" << endl;

    ifstream file(filename, ifstream::in);

    if(file.is_open()){
        while(!(file.eof())){
            getline(file, buf);

            pos = buf.find(":");
            cle = buf.substr(0, pos);
            value = buf.substr(pos+1, buf.size() - pos);

            if(cle.compare("FOSC") == 0)
                setFosc(atoi(value.c_str()));
            if(cle.compare("BAUD") == 0)
                setBaud(atoi(value.c_str()));
            if(cle.compare("DATA") == 0)
                setNbBits(atoi(value.c_str()));
            if(cle.compare("STOP") == 0)
                setBitsStop(atoi(value.c_str()));
            if(cle.compare("PARITY") == 0)
                setParity(atoi(value.c_str()));
            if(cle.compare("COM") == 0)
                setComNumber(atoi(value.c_str()));
        }
        file.close();
    }
    else
        cout << "error opening the settings file" << endl;

}

void Settings::setFosc(int _fosc){
    fosc = _fosc;
}

void Settings::setBaud(int _baud){
    baud = _baud;
}

void Settings::setNbBits(int _nb_bits){
    nb_bits = _nb_bits;
}

void Settings::setBitsStop(int _bits_stop){
    bits_stop = _bits_stop;
}

void Settings::setParity(int _parity){
    parity = _parity;
}

void Settings::setComNumber(int _com){
    com = _com;
}
