#include <iostream>
#include <string>

#include "ReadSerialCom.h"

using namespace std;

int main()
{
    FILE* file = fopen("spatial_data_inertial.csv","w");
	printf("Reading.....\n");//read spatial event data
    ReadSerialCom rsc(file, CSV, DISPLAY_QUATERNION|DISPLAY_EULER);


	printf("Enter any key then tap enter to stop the program.");
    cin.get();
	fclose(file);

    return 1;
}
