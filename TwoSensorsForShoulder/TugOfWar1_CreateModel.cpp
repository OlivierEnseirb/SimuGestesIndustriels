#include <OpenSim/OpenSim.h>
#include <string>
#include <conio.h>
#include "Multiple_sensor_reading.h"


using namespace std;


void updateVisualizer(Multiple_Sensor_Reading& msr, OpenSim::Model& osimModel, SimTK::State& currentState)
{
	while (true)
	{
		if (_kbhit())
			break;
		for (size_t num_thread = 0; num_thread < msr.sens_samples.size(); num_thread++)
		{
			if (msr.sens_samples[num_thread].size()>0 && msr.sens_joint_link[num_thread].size() == 3)
			{
				osimModel.setStateVariable(currentState, msr.sens_joint_link[num_thread][0], msr.sens_samples[num_thread][0].euler.x);
				osimModel.setStateVariable(currentState, msr.sens_joint_link[num_thread][1], msr.sens_samples[num_thread][0].euler.y);
				osimModel.setStateVariable(currentState, msr.sens_joint_link[num_thread][2], msr.sens_samples[num_thread][0].euler.z);
				msr.sens_samples[num_thread].erase(msr.sens_samples[num_thread].begin());
			}
		}
		osimModel.getVisualizer().show(currentState);
	}
}

void updateVisualizer2(Multiple_Sensor_Reading& msr, OpenSim::Model& osimModel, SimTK::State& currentState)
{
	DATA_TYPE M_identity[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_capteurDos_Terre[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_capteurBras_Terre[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_isbBras_capteurBras[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_isbDos_capteurDos[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_isbBras_isbDos[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_temp0[MAT_SIZE][MAT_SIZE];
	DATA_TYPE M_temp1[MAT_SIZE][MAT_SIZE];
	DATA_TYPE M_temp2[MAT_SIZE][MAT_SIZE];

	while (true)
	{
		if (_kbhit())
			break;
		if (msr.sens_samples[NUM_THREAD_SHOULDER].size()>0 && msr.sens_samples[NUM_THREAD_BACK].size()>0)
		{
			msr.QuaternionToOrthogonalMatrix(msr.sens_samples[NUM_THREAD_SHOULDER][0].quaternion, M_capteurBras_Terre);
			msr.QuaternionToOrthogonalMatrix(msr.sens_samples[NUM_THREAD_BACK][0].quaternion, M_capteurDos_Terre);


			msr.mulMatrix(M_isbBras_capteurBras, M_capteurBras_Terre, M_temp0);

			msr.traMatrix(M_isbDos_capteurDos, M_temp1);
			msr.copMatrix(M_temp1, M_isbDos_capteurDos);
			msr.traMatrix(M_capteurDos_Terre, M_temp1);
			msr.copMatrix(M_temp1, M_capteurDos_Terre);
			msr.mulMatrix(M_capteurDos_Terre, M_isbDos_capteurDos, M_temp1);

			msr.mulMatrix(M_temp1, M_temp0, M_isbBras_isbDos);

			Vec4 q = Vec4();
			Vec3 e = Vec3();
			msr.OrthogonalMatrixToQuaternion(M_isbBras_isbDos, q);
			msr.QuaternionToEuler(q, e);

			osimModel.setStateVariable(currentState, "r_shoulder_xRotation", e.x);
			osimModel.setStateVariable(currentState, "r_shoulder_yRotation", e.y);
			osimModel.setStateVariable(currentState, "r_shoulder_zRotation", e.z);
			
			msr.sens_samples[NUM_THREAD_SHOULDER].erase(msr.sens_samples[NUM_THREAD_SHOULDER].begin());
			msr.sens_samples[NUM_THREAD_BACK].erase(msr.sens_samples[NUM_THREAD_BACK].begin());
		}
		osimModel.getVisualizer().show(currentState);
	}
}

int main()
{/*
	Multiple_Sensor_Reading msr = Multiple_Sensor_Reading(); // launch communication with sensors in parallel threads

	DATA_TYPE M_identity[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_capteurDos_Terre[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_capteurBras_Terre[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_isbBras_capteurBras[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_isbDos_capteurDos[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	DATA_TYPE M_isbBras_isbDos[MAT_SIZE][MAT_SIZE] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };

	msr.displayMatrix(M_identity);
	msr.addMatrix(M_capteurBras_Terre, M_isbDos_capteurDos, M_isbBras_capteurBras);
	msr.mulMatrix(M_isbBras_capteurBras, M_isbBras_capteurBras, M_isbDos_capteurDos);
	msr.mulMatrix(M_isbDos_capteurDos, 3, M_capteurBras_Terre);

	msr.displayMatrix(M_capteurBras_Terre);
	msr.displayMatrix(M_isbDos_capteurDos);
	msr.displayMatrix(M_isbBras_capteurBras);


	cin.get();
	return 1;
}*/

	cout<<"Enter the file name of the OpenSim Model to display : ";
	string model_file_name;
	cin >> model_file_name;
	
	try {
		
		OpenSim::Model my_osimModel(model_file_name); //load the opensim model
		//Model my_osimModel("C:\\Users\\Olivier\\Desktop\\ExampleMain\\build\\Release\\bodyModel.osim");
		my_osimModel.setUseVisualizer(true);	// enable the visualizer

		SimTK::State currentState = my_osimModel.initSystem(); // initialize the model and get the default state of the model
		my_osimModel.getVisualizer().show(currentState); // display the state

		FILE* data_file = fopen("spatial_data_inertial.csv", "w"); // open the output file to write data
		
		unsigned char data_to_write = ENABLE_EULER; // data to write into the file : EULER, ACCELERATION, VELOCITY... cf Sample.h
		unsigned char data_to_display = ENABLE_NOTHING; // data to display each time a sample is got
		double waiting_time = 10; //seconds
		Multiple_Sensor_Reading msr(data_file, CSV, data_to_write, data_to_display, waiting_time); // launch communication with sensors in parallel threads

		printf("Reading.....\n");
		printf("Enter any key then tap enter to stop the program.\n");

		updateVisualizer2(msr, my_osimModel, currentState); // loop that update the visualizer

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