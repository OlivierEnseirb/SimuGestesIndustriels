#ifndef WRITE_FILE_H
#define WRITE_FILE_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <vector>

#include "VCD_definitions.h"
#include "calculateTimestamp.h"


#define BIT_0_PORT              '#'
#define BIT_1_PORT              '%'
#define BIT_2_PORT              '&'
#define BIT_3_PORT              '*'
#define BIT_4_PORT              '('
#define BIT_5_PORT              ')'
#define BIT_6_PORT              '['
#define BIT_7_PORT              ']'

using namespace std;

class Write_file
{
private:
    string filename;
    fstream file;
    double timestamp = 0;
    bool isFirstByte = true;
    vector<char> varIdentifiers;
    unsigned char previousByte;

public:
    Write_file(const string& s);
    ~Write_file();


    void writeHeadFile(void);
    void initializeFileVariables(void);
    void WriteByte(unsigned char buff);
};

#endif // WRITE_FILE_H
