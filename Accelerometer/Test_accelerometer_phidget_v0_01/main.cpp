#include <stdio.h>
#include "ReadSpatialPhidget.h"

//main entry point to the program
int main(int argc, char* argv[])
{
	//all done, exit
    FILE* file = fopen("spatial_data.csv","w");

    ReadSpatialPhidget accelerometer(file, 16, CSV);

	//read spatial event data
	printf("Reading.....\n");

	printf("Enter any key then tap enter to stop the program.");
	getchar();

	fclose(file);
	return 0;
}

