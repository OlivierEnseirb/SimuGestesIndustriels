#include <OpenSim/OpenSim.h>
#include <string>
#include <conio.h>
#include "Multiple_sensor_reading.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;


void updateVisualizer(Multiple_Sensor_Reading& msk, Model& osimModel, State& currentState)
{
	while (true)
	{
		if (_kbhit())
			break;
		for (size_t num_thread = 0; num_thread < msk.sens_samples.size(); num_thread++)
		{
			if (msk.sens_samples[num_thread].size()>0 && msk.sens_joint_link[num_thread].size() == 3)
			{
				osimModel.setStateVariable(currentState, msk.sens_joint_link[num_thread][0], msk.sens_samples[num_thread][0].euler.x);
				osimModel.setStateVariable(currentState, msk.sens_joint_link[num_thread][1], msk.sens_samples[num_thread][0].euler.y);
				osimModel.setStateVariable(currentState, msk.sens_joint_link[num_thread][2], msk.sens_samples[num_thread][0].euler.z);
				msk.sens_samples[num_thread].erase(msk.sens_samples[num_thread].begin());
			}
		}
		osimModel.getVisualizer().show(currentState);
	}
}

int main()
{
	try {
		Model my_osimModel("rightArmModel.osim"); //load the opensim model
		my_osimModel.setUseVisualizer(true);	// enable the visualizer
		State currentState = my_osimModel.initSystem(); // initialize the model and get the default state of the model
		my_osimModel.getVisualizer().show(currentState); // display the state
		
		FILE* data_file = fopen("spatial_data_inertial.csv", "w"); // open the output file to write data

		unsigned char data_to_write = ENABLE_EULER; // data to write into the file : EULER, ACCELERATION, VELOCITY... cf Sample.h
		unsigned char data_to_display = ENABLE_NOTHING; // data to display each time a sample is got

		Multiple_Sensor_Reading msr(data_file, CSV, data_to_write, data_to_display); // launch communication with sensors in parallel threads

		printf("Reading.....\n");
		printf("Enter any key then tap enter to stop the program.\n");

		updateVisualizer(msr, my_osimModel, currentState); // loop that update the visualizer

		msr.stopReading(); //stop communications

		fclose(data_file); // close the file
	}
	catch (OpenSim::Exception e)
	{
		cout << e.getMessage() << endl;
	}

	cin.get(); // wait enter to kill program
	return EXIT_SUCCESS;
}