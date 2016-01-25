/***************************************
* Author : Amandine Rémont (Nov. 2015)
* Update janv. 2016 by Olivier Hartmann
* This object is used to read a serial port
***************************************/
#include "uart_gestion.h"

UART_gestion::UART_gestion()
{

}

UART_gestion::~UART_gestion()
{

}

//FONCTION DE CONVERSION STRING VERS TYPE LISIBLE PAR WINDOWS
wstring UART_gestion::s2ws(const std::string& s)
{
    int len;
    int slength = (int)s.length() + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    wchar_t* buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r;
}

//FONCTION D'OUVERTURE DU PORT COM ET DE SA CONFIGURATION
bool UART_gestion::OpenCOM(int nId, string& error_message){

    //construction du nom du port COM
    stringstream ss;
    ss << nId;
    const string myString = "COM" + ss.str();

//    string stemp = s2ws(myString);
    LPCSTR result = myString.c_str();

    g_hCOM = CreateFile(result, GENERIC_READ|GENERIC_WRITE,0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_SYSTEM, NULL);
    //les paramètres de la fonction CreateFile sont les suivants:
    // Nom du port à ouvrir
    // Type d'accès au fichier (read/write)
    // le mode de partage du fichier: 0 = le fichier est protégé
    // NULL = le handle retourné ne peut pas être hérité par une classe fille
    // ouverture du fichier uniquement si le device existe
    // attribut du fichier, le fichier est uniquement utilisé par un OS
    // valid handle to template, NULL = le paramètre est ignoré

    if(g_hCOM == INVALID_HANDLE_VALUE){
		error_message = "erreur d'ouverture de la com serie";
        //cout << "erreur d'ouverture de la com serie" << endl;

        if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			error_message = "The system cannot find the file specified";
			//cout <<"The system cannot find the file specified" << endl;  //error code 0x02
        }
        else if(GetLastError() == ERROR_INVALID_NAME) {
			error_message = "filename, directory name, or volume label syntax is incorrect";
			//cout << "filename, directory name, or volume label syntax is incorrect" << endl;  //error code 0x7B
        }
        else
        {
			error_message = "Handle creation error code" + GetLastError();
			//cout << "Handle creation error code" << GetLastError() << endl;
        }


        return false;
    }

    //affectation de la taille des buffers d'émission et de réception
    SetupComm(g_hCOM, RX_SIZE, TX_SIZE);

    //configuration du port COM avec les paramètres de la strucutre
    if(!SetCommTimeouts(g_hCOM, &g_cto) || !SetCommState(g_hCOM, &g_dcb)){
        cout << "erreur lors de la configuration du port" << endl;
        CloseHandle(g_hCOM);
        return false;
    }

    //on vide les tampons d'émission et de réception et on est prêt à utiliser la liaison série
    PurgeComm(g_hCOM, PURGE_TXCLEAR|PURGE_RXCLEAR|PURGE_TXABORT|PURGE_RXABORT);
    EscapeCommFunction(g_hCOM, SETDTR);
    return TRUE;
}

//FONCTION DE FERMETURE DU PORT COM
bool UART_gestion::CloseCOM(){
    //fermeture de la communication série
    CloseHandle(g_hCOM);
    return true;
}

//FONCTION DE LECTURE SUR LE PORT COM
bool UART_gestion::ReadCOM(void *buffer, int nBytesToRead, int *pBytesRead){
    return ReadFile(g_hCOM, buffer, nBytesToRead, (LPDWORD)pBytesRead, NULL);
    /*Les paramètres de la fonction ReadFile sont les suivants:
     * handle qui sert pour la COM
     * pointeur vers un buffer qui va recevoir les données lues
     * nombre d'octets max à lire
     * pointeur vers le buffer qui reçoit le nombre d'octets lus
     * paramètre non utile dans notre cas
	 
	 * Return value
	 *	If the function succeeds, the return value is nonzero (TRUE).
	 *	If the function fails, or is completing asynchronously, the return value is zero (FALSE). To get extended error information, call the GetLastError function.
	 */
}

//FONCTION D'ECRITURE SUR LE PORT COM
bool UART_gestion::WriteCOM(void *buffer, int nBytesToWrite, int *pBytesWritten){
    return WriteFile(g_hCOM, buffer, nBytesToWrite, (LPDWORD)pBytesWritten, NULL);
}

//FONCTION DE REMPLISSAGE DE LA STRUCTURE DE CONFIGURATION DU PORT COM AVEC DONNEES DE LA COM
void UART_gestion::SetDcbStructure(int _baud, int _nbBits, int _nbStop, int _parity){
    //configuration du BaudRate
    g_dcb.BaudRate = _baud;
    //configuration du nombre de bits de données
    g_dcb.ByteSize = _nbBits;
    //configuration du nombre de bits de Stop
    switch (_nbStop) {
    case 0:
        //1 bit de stop
        g_dcb.StopBits = ONESTOPBIT;
        break;
    case 1:
        //1.5 bits de stop
        g_dcb.StopBits = ONE5STOPBITS;
        break;
    case 2:
        //2 bits de stop
        g_dcb.StopBits = TWOSTOPBITS;
        break;
    default:
        //1 bit de stop par défaut
        g_dcb.StopBits = ONESTOPBIT;
        break;
    }
    //Configuration de la parité
    switch (_parity) {
    case 0:
        //pas de parité
        g_dcb.Parity = NOPARITY;
        break;
    case 1:
        //parité impaire
        g_dcb.Parity = ODDPARITY;
        break;
    case 2:
        //parité parire
        g_dcb.Parity = EVENPARITY;
        break;
    case 3:
        //mark parity
        g_dcb.Parity = MARKPARITY;
        break;
    case 4:
        //space parity
        g_dcb.Parity = SPACEPARITY;
        break;
    default:
        //pas de parité
        g_dcb.Parity = NOPARITY;
        break;
    }
}

//FONCTION DE REMPLISSAGE DE LA STRUCTURE DE CONFIGURATION DU PORT COM AVEC DONNEES DE LA COM
void UART_gestion::SetDcbStructure() {
	//configuration du BaudRate
	g_dcb.BaudRate = getBaud();
	//configuration du nombre de bits de données
	g_dcb.ByteSize = getNbBits();
	//configuration du nombre de bits de Stop
	switch (getBitsStop()) {
	case 0:
		//1 bit de stop
		g_dcb.StopBits = ONESTOPBIT;
		break;
	case 1:
		//1.5 bits de stop
		g_dcb.StopBits = ONE5STOPBITS;
		break;
	case 2:
		//2 bits de stop
		g_dcb.StopBits = TWOSTOPBITS;
		break;
	default:
		//1 bit de stop par défaut
		g_dcb.StopBits = ONESTOPBIT;
		break;
	}
	//Configuration de la parité
	switch (getParity()) {
	case 0:
		//pas de parité
		g_dcb.Parity = NOPARITY;
		break;
	case 1:
		//parité impaire
		g_dcb.Parity = ODDPARITY;
		break;
	case 2:
		//parité parire
		g_dcb.Parity = EVENPARITY;
		break;
	case 3:
		//mark parity
		g_dcb.Parity = MARKPARITY;
		break;
	case 4:
		//space parity
		g_dcb.Parity = SPACEPARITY;
		break;
	default:
		//pas de parité
		g_dcb.Parity = NOPARITY;
		break;
	}
}
