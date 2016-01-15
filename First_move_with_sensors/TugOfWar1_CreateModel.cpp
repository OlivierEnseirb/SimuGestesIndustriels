#include <OpenSim/OpenSim.h>
#include <string>
#include <conio.h>

#include "ReadSerialCom.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77


int main()
{
	/* open the model and enable visualizer */
	Model osimModel("rightArmModel.osim");
	osimModel.setUseVisualizer(true);

	/* create destination file of data*/
	FILE* file = fopen("spatial_data_inertial.csv", "w");

	printf("Reading.....\n");//read spatial event data
	printf("Enter any key then tap enter to stop the program.\n");
	ReadSerialCom rsc(file, CSV, ENABLE_EULER, ENABLE_NOTHING, osimModel);

	cin.get();
	fclose(file);
	return EXIT_SUCCESS;
}