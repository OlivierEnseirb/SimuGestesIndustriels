/***************************************
* Author : Olivier Hartmann (Janv. 2015)
* This is the main file of the project that open communications with sensors and display in real time the opensim model
***************************************/

#include <OpenSim/OpenSim.h>
#include <string>
#include <conio.h>
#include "Multiple_sensor_reading.h"


using namespace std;
//using namespace OpenSim; // ne pas ajouter ces namespaces car sinon il y a conflit entre des classes du même (Vec3 et Vec4) alors créé par moi et semblant exister dans OpenSim.
//using namespace SimTK;

void updateVisualizer(Multiple_Sensor_Reading& msr, OpenSim::Model& osimModel, SimTK::State& currentState) // set sensor data directly into the joints value
{
	while (true)
	{
		if (_kbhit())
			break;
		for (size_t num_thread = 0; num_thread < msr.sens_samples.size(); num_thread++)
		{
			if (msr.sens_samples[num_thread].size()>0 && msr.sens_joint_link[num_thread].size() == 3)
			{
				//sets the euler angles of the new sample into the matching joint
				osimModel.setStateVariable(currentState, msr.sens_joint_link[num_thread][0], msr.sens_samples[num_thread][0].euler.x);
				osimModel.setStateVariable(currentState, msr.sens_joint_link[num_thread][1], msr.sens_samples[num_thread][0].euler.y);
				osimModel.setStateVariable(currentState, msr.sens_joint_link[num_thread][2], msr.sens_samples[num_thread][0].euler.z);
				
				// then erases it
				msr.sens_samples[num_thread].erase(msr.sens_samples[num_thread].begin());
			}
		}
		// update the visualizer
		osimModel.getVisualizer().show(currentState);
	}
}

void updateVisualizer_shoulder(Multiple_Sensor_Reading& msr, OpenSim::Model& osimModel, SimTK::State& currentState) // inverse kinematic only on the shoulder
{
	// matrix for the inverse kinematic
	/* TO DO
	 * make a real function that make the inverse kinematic
	 */
	//RotMat M_identity = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	RotMat M_capteurDos_Terre = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	RotMat M_capteurBras_Terre = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	RotMat M_isbBras_capteurBras = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	RotMat M_isbDos_capteurDos = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	RotMat M_isbBras_isbDos = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	RotMat M_temp0;
	RotMat M_temp1;

	// get the position of the sensors in the threads
	size_t num_thread_shoulder, num_thread_back; 
	msr.findThreadByName(string("shoulder"), num_thread_shoulder);
	msr.findThreadByName(string("back"), num_thread_back);


	while (true)
	{
		if (_kbhit()) // take a look at the keyboard buffer and say if a key was entered or not
			break;
		if (msr.sens_samples[num_thread_shoulder].size()>0 && msr.sens_samples[num_thread_back].size()>0)
		{ // there are new data from the two sensors

			// calculate Rotation matrix of the two new quaternions
			msr.QuaternionToOrthogonalMatrix(msr.sens_samples[num_thread_shoulder][0].quaternion, M_capteurBras_Terre);
			msr.QuaternionToOrthogonalMatrix(msr.sens_samples[num_thread_back][0].quaternion, M_capteurDos_Terre);

			msr.mulMatrix(M_isbBras_capteurBras, M_capteurBras_Terre, M_temp0); // M_temp0 = M_isbBras_capteurBras * M_capteurBras_Terre

			msr.traMatrix(M_isbDos_capteurDos, M_temp1);
			msr.copMatrix(M_temp1, M_isbDos_capteurDos); // M_isbDos_capteurDos = transpose(M_isbDos_capteurDos)

			msr.traMatrix(M_capteurDos_Terre, M_temp1);
			msr.copMatrix(M_temp1, M_capteurDos_Terre); // M_capteurDos_Terre = transpose(M_capteurDos_Terre)

			msr.mulMatrix(M_capteurDos_Terre, M_isbDos_capteurDos, M_temp1); // M_temp1 = transpose(M_capteurDos_Terre) * transpose(M_isbDos_capteurDos)

			msr.mulMatrix(M_temp1, M_temp0, M_isbBras_isbDos); // M_isbBras_isbDos = M_isbBras_capteurBras * M_capteurBras_Terre * transpose(M_capteurDos_Terre) * transpose(M_isbDos_capteurDos)

			Vec4 q;
			Vec3 e;
			msr.OrthogonalMatrixToQuaternion(M_isbBras_isbDos, q);
			msr.QuaternionToEuler(q, e); // euler angles of the rotation between the arm and the back

			//set the data into the shoulder articulation
			osimModel.setStateVariable(currentState, "r_shoulder_xRotation", e.x);
			osimModel.setStateVariable(currentState, "r_shoulder_yRotation", e.y);
			osimModel.setStateVariable(currentState, "r_shoulder_zRotation", e.z);
			//printf("(%3f, %3f, %3f)\n", e.x*180/ MATHS_PI, e.y * 180 / MATHS_PI, e.z * 180 / MATHS_PI);
			
			// erase the two samples just use to update the model
			msr.sens_samples[num_thread_shoulder].erase(msr.sens_samples[num_thread_shoulder].begin());
			msr.sens_samples[num_thread_back].erase(msr.sens_samples[num_thread_back].begin());

			//update the model in the visualizer
			osimModel.getVisualizer().show(currentState);
		}
	}
}

int main()
{
	cout<<"Enter the file name of the OpenSim Model to display : ";
	string model_file_name;
	cin >> model_file_name;
	
	try {
		OpenSim::Model my_osimModel(model_file_name); //load the opensim model
		my_osimModel.setUseVisualizer(true);	// enable the visualizer

		SimTK::State currentState = my_osimModel.initSystem(); // initialize the model and get the default state of the model
		my_osimModel.getVisualizer().show(currentState); // display the state

		FILE* data_file = fopen("spatial_data_inertial.csv", "w"); // open the output file to write data
		
		unsigned char data_to_write = ENABLE_EULER; // data to write into the file : EULER, ACCELERATION, VELOCITY... cf Sample.h
		unsigned char data_to_display = ENABLE_NOTHING; // data to display each time a sample is got
		DATA_TYPE waiting_time = 10; //seconds
		Multiple_Sensor_Reading msr(data_file, CSV, data_to_write, data_to_display, waiting_time); // launch communication with sensors in parallel threads

		printf("Reading.....\n");
		printf("Enter any key to stop reading sensors.\n");

		/* TO DO 
		 * intialisation du modele et des matrices
		 * il serait bon de créer une classe s'occupant de la cinématique inverse et de la gestion de l'affichage en même temps
		 * elle serait donc appellée au début de cette fonction, créant alors le modèle OpenSim et contiendrait une fonction d'initialisation des matrices et la fonction updateVisualizer_shoulder pour tout le corps
		 */
		updateVisualizer_shoulder(msr, my_osimModel, currentState); // loop that update the visualizer

		msr.stopReading(); //stop communications

		fclose(data_file); // close the file
	}
	catch (OpenSim::Exception e)
	{
		cout << e.getMessage() << endl;
	}

	printf("Enter any key to close the program.\n");
	while (!_kbhit()) // take a look at the keyboard buffer and say if a key was entered or not
	{

	}// wait enter to kill program
	return EXIT_SUCCESS;
}