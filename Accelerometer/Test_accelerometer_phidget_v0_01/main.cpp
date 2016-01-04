#include <stdio.h>
#include "ReadSpatialPhidget.h"

//main entry point to the program
int main(int argc, char* argv[])
{
	//all done, exit
    FILE* file = fopen("spatial_data_arm.trc","w");

    ReadSpatialPhidget accelerometer(file, 16, TRC);

	//read spatial event data
	printf("Reading.....\n");

	printf("Enter any key then tap enter to stop the program.");
	getchar();

	fclose(file);
	return 0;
}

