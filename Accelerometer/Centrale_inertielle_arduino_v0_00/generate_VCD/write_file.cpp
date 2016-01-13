#include "write_file.h"

Write_file::Write_file(const string &s)
{
    filename = s;
    cout << "ouverture du fichier de sortie" << endl;
    file.open(filename, fstream::out);
    if(file.is_open())
        cout << "fichier ouvert" << endl;
    else
        exit(EXIT_FAILURE);
    //initialisation du tableau contenant les caractères d'identification des variables pour le VCD
    varIdentifiers.push_back(BIT_0_PORT);
    varIdentifiers.push_back(BIT_1_PORT);
    varIdentifiers.push_back(BIT_2_PORT);
    varIdentifiers.push_back(BIT_3_PORT);
    varIdentifiers.push_back(BIT_4_PORT);
    varIdentifiers.push_back(BIT_5_PORT);
    varIdentifiers.push_back(BIT_6_PORT);
    varIdentifiers.push_back(BIT_7_PORT);

    //écriture de l'entete du fichier VCD
    writeHeadFile();

    //initialisation des variables
    initializeFileVariables();
}

Write_file::~Write_file()
{
    cout << "fermeture du fichier de sortie" << endl;
    file.close();
}

void Write_file::writeHeadFile(void){

    //ecriture de l'entete du fichier VCD

    //commentaire de début de fichier
    file << VCD_COMMENT_SYMBOLE << endl << "Generation des valeurs sortant du clavier" << endl << VCD_END_SYMBOLE << endl;

    //définition de l'unite du timestamp
    file << VCD_TIMESCALE_SYMBOLE << " 1" << VCD_us << VCD_END_SYMBOLE << endl;

    //ligne pour definir le scope ?
    file << VCD_SCOPE_SYMBOLE << " module logic" << VCD_END_SYMBOLE << endl;

    //definition des variables, un bus de données séparé en 8 bits
    for(unsigned int i = 0; i < varIdentifiers.size(); i++)
        file << VCD_VAR_SYMBOLE << " wire 1 " << varIdentifiers[i] << " PB" << i << " " << VCD_END_SYMBOLE << endl;

    //fin de l'entete
    file << "$upscope " << VCD_END_SYMBOLE << endl;
    file << VCD_ENDDEFINITION_SYMBOLE << " " << VCD_END_SYMBOLE << endl;
}

void Write_file::initializeFileVariables(void){
    //Initialisations des variables
    file << VCD_DUMPVARS_SYMBOLE << endl;

    //toutes les variables sont initialisées à un état inconnu correspondant au temps -1
    for(unsigned int i = 0; i < varIdentifiers.size(); i ++)
        file << VCD_VALUE_UNKNOW << varIdentifiers[i] << endl;

    //fin de l'initialisation
    file << VCD_END_SYMBOLE <<endl;
}

void Write_file::WriteByte(unsigned char buff){

    //on récupère les 8 bits de données
    unsigned char byte = buff;

    //ecriture d'un octet dans le fichier, on n'écrit que les bits qui changent

    //on détermine si c'est le premier octet que l'on écrit
    if(isFirstByte == true){
        //on démarre le timer pour le timestamp
        StartTimer();

        //ecriture du timestamp en us
        file << VCD_TIMESTAMP_SYMBOLE << (int)timestamp << endl;

        //ecriture de l'octet dans son intégralité
        for (unsigned int i = 0; i < 8; i++){
            if((byte>>i & 0x01) == 0)
                file << VCD_VALUE_LOW << varIdentifiers[i] << endl;
            else
                file << VCD_VALUE_HIGH << varIdentifiers[i] << endl;
        }

        isFirstByte = false;
        previousByte = byte;
    }
    else{
        //on récupère le timestamp
        timestamp = GetTimer();

        //ecriture du timestamp en ns
        //file << VCD_TIMESTAMP_SYMBOLE << (int)timestamp << endl;

        //on écrit uniquement les données qui ont changées
        for(unsigned int i = 0; i < 8; i++){

            if((byte>>i & 0x01) != (previousByte>>i & 0x01)){
                //alors on écrit
                //ecriture du timestamp en ns
                file << VCD_TIMESTAMP_SYMBOLE << (int)timestamp << endl;

                if((byte >>i & 0x01) == 0)
                    file << VCD_VALUE_LOW << varIdentifiers[i] << endl;
                else
                    file << VCD_VALUE_HIGH << varIdentifiers[i] << endl;
            }

        }

        previousByte = byte;
    }

}

