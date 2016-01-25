/***************************************
* Author : Amandine Rémont (Nov. 2015)
* Update janv. 2016 by Olivier Hartmann
* This object is used to configure a serial communication
***************************************/

#ifndef SETTINGS_H
#define SETTINGS_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>

using namespace std;

/*Pour l'écriture du fichier d'initialisation, les règles sont les suivantes (en accord avec la com série windows):
 * FOSC:correspond à l'horloge effective sur l'arduino
 * BAUD:BaudRate
 * DATA:nombre de bits de données
 * STOP:nombre de bits de stop (0 = 1bit, 1 = 1.5bits, 2 = 2bits, tout autre chiffre = 1bit)
 * PARITY:parité (0 = pas de parité, 1 = parité impaire, 2 = parité paire, 3 = "mark parity", 4 = "space parity", tout autre chiffre = pas de parité)
 * COM:numéro du port com*/


class Settings
{
private:
    int fosc;
    int baud;
    int nb_bits;
    int bits_stop;
    int parity;
    int com;

public:
    Settings(){}
    ~Settings();


    int getFosc(void);
    int getBaud(void);
    int getNbBits(void);
    int getBitsStop(void);
    int getParity(void);
    int getComNumber(void);
    void getSettings(const string& filename); // reads the given file and fills its attributs with what is written inside the file
    void setComNumber(int _com);

protected:
    void setFosc(int _fosc);
    void setBaud(int _baud);
    void setNbBits(int _nb_bits);
    void setBitsStop(int _bits_stop);
    void setParity(int _parity);
};

#endif // SETTINGS_H
