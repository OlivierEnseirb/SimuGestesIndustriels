#include <iostream>
#include <list>

#include "settings.h"
#include "uart_gestion.h"
#include "write_file.h"


#define BUFF_SIZE   16

#define MAIN
//#define TEST_VCD
//#define TEST_COM


char inverseByte(char byteIn);


using namespace std;

int main()
{

#ifdef MAIN

    Settings* set_data = new Settings();
    UART_gestion* serial_com = new UART_gestion();
    Write_file* out_vcd_file = new Write_file("vcd_file.vcd");

    //Déclaration d'un buffer qui reçoit les données
    char buffer[BUFF_SIZE];
    int nb_bytes_read;
    int nb_bytes_written;

    list<unsigned char> bufferList;

    //lecture du fichier de config et récupération des données
    set_data->getSettings("init.txt");

    //configuration de la strucutre concernant la com série en fonction des données du fichier de config
    serial_com->SetDcbStructure(set_data->getBaud(), set_data->getNbBits(), set_data->getBitsStop(), set_data->getParity());

    //ouverture du port com en fonction du numéro donné dans le fichier de config
    cout << "tentative d'ouverture du COM" << set_data->getComNumber() << endl;
    if( serial_com->OpenCOM(set_data->getComNumber())){
        //la communication série est ouverture
        cout << "COM Serie ouverte et prete a l'emploi" << endl;

        //envoi d'un caractere pour la synchronisation
        for(int i = 0; i < 0; i++){
            buffer[0] = 'a';
            serial_com->WriteCOM(buffer, 1, &nb_bytes_written);
        }
        int sum =  0;
       //opération de lecture sur la com série
        while(1){
            if(_kbhit())
                break;

            //cout << "appel de la fonction de lecture du port série" <<endl;
            serial_com->ReadCOM(buffer, 16, &nb_bytes_read);
           cout << "." << endl;
            sum += nb_bytes_read;
            if(nb_bytes_read != 0)
                for(int i = 0; i < nb_bytes_read; i++)
                    bufferList.push_back(buffer[i]);

        }
        cout << "lu " << sum << " octets" << endl;
        for(list<unsigned char>::iterator j = bufferList.begin(); j != bufferList.end(); j++)
            out_vcd_file->WriteByte(*j);

        //fermeture du port com
        cout << "fermeture de la communication" << endl;
        serial_com->CloseCOM();
    }


    delete set_data;
    delete serial_com;
    delete out_vcd_file;
    return 0;

#endif

#ifdef TEST_VCD

    Write_file* out_vcd_file = new Write_file("test_vcd_file.vcd");

    char buffer[] = {0x01,
                     0x00,
                     0x01,
                     0x00,
                     0x01,
                     0x00,
                     0x01,
                     0x00,
                     0x01,
                     0x02,
                     0x03,
                     0x00,
                     0x01,
                     0x02,
                     0x03,
                     0x00,
                     0x01,
                     0x02,
                     0x03,
                     0x02,
                     0x03,
                     0x02,
                     0x03};


    for(int i = 0; i < 23; i++)
        out_vcd_file->WriteByte(buffer+i);


    delete out_vcd_file;
    return 0;

#endif

#ifdef TEST_COM
    Settings* set_data = new Settings();
    UART_gestion* serial_com = new UART_gestion();

    //Déclaration d'un buffer qui reçoit les données
    char buffer[BUFF_SIZE];
    int nb_bytes_read;
    int nb_bytes_written;

    //lecture du fichier de config et récupération des données
    set_data->getSettings("init.txt");

    //configuration de la strucutre concernant la com série en fonction des données du fichier de config
    serial_com->SetDcbStructure(set_data->getBaud(), set_data->getNbBits(), set_data->getBitsStop(), set_data->getParity());

    //ouverture du port com en fonction du numéro donné dans le fichier de config
    cout << "tentative d'ouverture du COM" << set_data->getComNumber() << endl;
    if( serial_com->OpenCOM(set_data->getComNumber())){
        //la communication série est ouverture
        cout << "COM Serie ouverte et prete a l'emploi" << endl;

        //envoi d'un caractère
        for(int i = 0; i < 1; i++){
            buffer[0] = 'a';
            serial_com->WriteCOM(buffer, 1, &nb_bytes_written);
            cout << "nb octet envoye: " << nb_bytes_written << endl;
            cout << "caractere envoye: " << buffer[0] << endl;
        }
       //opération de lecture sur la com série
        while(1){
            if(_kbhit())
                break;

            cout << "appel de la fonction de lecture du port série" <<endl;
            serial_com->ReadCOM(buffer, 1, &nb_bytes_read);
            cout << "nombre de bits lus: " << nb_bytes_read << endl;
            if(nb_bytes_read != 0)
                cout << "caractere recu: " << buffer[0] << endl;
        }
        //fermeture du port com
        cout << "fermeture de la communication" << endl;
        serial_com->CloseCOM();
    }


    delete set_data;
    delete serial_com;

#endif
}

char inverseByte(char byteIn){
    char byteOut = 0;

    for(int i = 0; i < 8; i++){
        if((byteIn) >> i & 0x01){
            byteOut |= 1 << (7-i);
        }
    }

    return byteOut;
}

