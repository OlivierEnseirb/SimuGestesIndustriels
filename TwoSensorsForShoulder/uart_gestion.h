/***************************************
* Author : Amandine Rémont (Nov. 2015)
* Update janv. 2016 by Olivier Hartmann
* This object is used to read a serial port
***************************************/

#ifndef UART_GESTION_H
#define UART_GESTION_H

#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <string>
#include <sstream>
#include "settings.h"

using namespace std;

//Définition des constantes
#define RX_SIZE		4096 //taille du buffer de réception des données
#define TX_SIZE		4096 //taille du buffer de transmission des données
#define WAIT_READ	5000   //temps d'attente en ms pour la lecture


class UART_gestion : public Settings
{
    //déclaration des variables de fonctionnement du port série
private:
    //Délais d'attentente sur le port COM
    COMMTIMEOUTS g_cto =
    {
        WAIT_READ,  //ReadIntervalTimeOut
        0,          //ReadTotalTimeOutMultiplier
        WAIT_READ,  //ReadTotalTimeOutConstant
        0,          //WriteTotalTimeOutMultiplier
        0           //WriteTotalTimeOutConstant
    };


    //Configuration du port COM
    DCB g_dcb =
    {
        sizeof(DCB),            //DCBlength, taille de la structure
        CBR_300,               //BaudRate, peut aller jusqu'à 256000 bps
        TRUE,                   //fBinary, doit être à TRUE pour pouvoir lire une trame binaire (windows ne traite pas du binaire par défaut)
        FALSE,                  //fParity, pas de contrôle de parité
        FALSE,                  //fOutxCtsFlow, output flow control
        FALSE,                  //fOutxDsrFlow, idem
        DTR_CONTROL_DISABLE,    //fDtrControl
        FALSE,                  //fDsrSensitivity
        FALSE,                  //fTXContinueOnXoff
        FALSE,                  //fOutX
        FALSE,                  //fInX
        FALSE,                  //fErrorChar
        FALSE,                  //fNull
        RTS_CONTROL_DISABLE,    //fRtsControl
        FALSE,                  //fAbortOnError
        0,                      //fDummy2
        0,                      //wReserved
        0x100,                  //XonLim
        0x100,                  //XoffLim
        8,                      //ByteSize
        NOPARITY,               //Parity
        TWOSTOPBITS,            //StopBits
        0x11,                   //XonChar
        0x13,                   //XoffChar
        '?',                    //ErrorChar
        0x1A,                   //EofChar
        0x10,                   //EvtChar
        0                       //wReserved1
    };

    // Handle du port COM ouvert
    HANDLE g_hCOM = NULL;

    wstring s2ws(const string& s);

public:

    UART_gestion();
    ~UART_gestion();

    bool OpenCOM(int nId, string& error_message); //opening the serial port communication on the serial port number "nId"
    bool CloseCOM(void); // closing the running Serial Communication
    bool ReadCOM(void* buffer, int nBytesToRead, int* pBytesRead); // get data from the serial port
    bool WriteCOM(void* buffer, int nBytesToWrite, int* pBytesWritten); // write data on the serial port
    void SetDcbStructure(int _baud, int _nbBits, int _nbStop, int _parity); // initialize the serial port with parameters given in arguments
	void SetDcbStructure(); // initialize the serial port with parameters written in the "Settings" object 
};

#endif // UART_GESTION_H
