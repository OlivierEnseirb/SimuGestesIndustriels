/* -------------------------------------------------------------------------- *
 *                    OpenSim:  CreateModel.cpp                     *
 * -------------------------------------------------------------------------- *
 * Premier programme permettant de se faire à OpenSim                         *
 * Permet également de bouger certaines parties du cors à l'aide des touches: *
 *                   * HAUT * BAS * DROITE * GAUCHE *                         *
 * -------------------------------------------------------------------------- */

// Author:  Olivier Hartmann, Léa Pillette

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include <string>
//Used by 'getch' function
#include <conio.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//Used for the switch case with the different key pressed
#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77

int main()
{
	//Creation of the model which will be composed of all the elements
	Model osimModel;
	osimModel.setName("rightArmModel");
	//Allow real time visualization of the model
	osimModel.setUseVisualizer(true);

	// Get a reference to the model's ground body
	OpenSim::Body& ground = osimModel.getGroundBody();

	// Add display geometry to the ground (BODY TORSO) to visualize in the GUI
	vector<std::string> groundElem{ "jaw", "r_clavicle", "r_scapula", "ribs", "skull", "spine" };
	std::string name;
	for (int i = 0; i < groundElem.size(); i++) {
		name = "ground_" + groundElem[i] + ".vtp";
		ground.addDisplayGeometry(name);
	}

	// Specify properties of a 20 kg, 0.1 m^3 block body
	double upperArmMass = 20.0, upperArmLength = 0.1;
	Vec3 upperArmCenter(0);
	Inertia upperArmInertia = upperArmMass*Inertia::brick(upperArmLength, upperArmLength, upperArmLength);

	// Create a new block body (UPPER ARM) with specified properties
	OpenSim::Body *upperArm = new OpenSim::Body("upperArm", upperArmMass, upperArmCenter, upperArmInertia);

	// Add display geometry of the upper part of the arm
	vector<std::string> upperArmElem{ "humerus" };
	for (int i = 0; i < upperArmElem.size(); i++) {
		name = "arm_r_" + upperArmElem[i] + ".vtp";
		upperArm->addDisplayGeometry(name);
	}

	//Create a new free JOINT with 6 degrees - of - freedom(coordinates) between the upperArm and ground body
	Vec3 locationInParent(-0.017545, -0.007, 0.17), orientationInParent(0), locationInBody(0), orientationInBody(0);
	FreeJoint *r_shoulder = new FreeJoint("r_shoulder", ground, locationInParent, orientationInParent, *upperArm, locationInBody, orientationInBody);

	// Get a reference to the coordinate set (6 degrees-of-freedom) between the block and ground bodies
	CoordinateSet& jointCoordinateSet = r_shoulder->upd_CoordinateSet();

	// Set the angle and position ranges for the coordinate set (SimTK:: prefix not actually needed here)
	double angleRangeShoulderX[2] = { -SimTK::Pi, 0 };
	double angleRangeShoulderY[2] = { -SimTK::Pi / 4, SimTK::Pi + SimTK::Pi / 4 };
	double angleRangeShoulderZ[2] = { -SimTK::Pi / 4 , SimTK::Pi };
	double angleRange[2] = { -SimTK::Pi / 2, SimTK::Pi / 2 };
	double positionRange[2] = { -0.01, 0.01 };

	jointCoordinateSet[0].setRange(angleRangeShoulderX);
	jointCoordinateSet[1].setRange(angleRangeShoulderY);
	jointCoordinateSet[2].setRange(angleRangeShoulderZ);
	jointCoordinateSet[3].setRange(positionRange);
	jointCoordinateSet[4].setRange(positionRange);
	jointCoordinateSet[5].setRange(positionRange);

	// Specify properties of a 20 kg, 0.1 m^3 block body
	double lowerArmMass = 20.0, lowerArmLength = 0.1;
	Vec3 lowerArmCenter(0);
	Inertia lowerArmInertia = lowerArmMass*Inertia::brick(lowerArmLength, lowerArmLength, lowerArmLength);

	// Create a new block body (LOWER ARM) with specified properties
	OpenSim::Body *lowerArm = new OpenSim::Body("lowerArm", lowerArmMass, lowerArmCenter, lowerArmInertia);

	// Add display geometry of the lower part of the arm
	vector<std::string> lowerArmElem{ "1mc", "2distph", "2mc", "2midph", "2proxph", "3distph", "3mc", "3midph", "3proxph", "4distph", "4mc", "4midph", "4proxph", "5distph", "5mc", "5midph", "5proxph", "capitate", "hamate", "lunate", "pisiform", "radius", "scaphoid", "thumbdist", "thumbprox", "trapezium", "trapezoid", "triquetrum", "ulna" };
	for (int i = 0; i < lowerArmElem.size(); i++) {
		name = "arm_r_" + lowerArmElem[i] + ".vtp";
		lowerArm->addDisplayGeometry(name);
	}

	//Create a new free JOINT with 6 degrees - of - freedom(coordinates) between the lowerArm and ground bodies
	Vec3 locationInParenth(0.0061, -0.2904, -0.0123), orientationInParenth(0), locationInBodyh(0), orientationInBodyh(0);
	FreeJoint *r_helbow = new FreeJoint("r_helbow", *upperArm, locationInParenth, orientationInParenth, *lowerArm, locationInBodyh, orientationInBodyh);

	// Get a reference to the coordinate set (6 degrees-of-freedom) between the block and ground bodies
	CoordinateSet& jointCoordinateSetH = r_helbow->upd_CoordinateSet();

	// Set the angle and position ranges for the coordinate set (SimTK:: prefix not actually needed here)
	double angleRangeHelbowX[2] = { 0, 0 };
	double angleRangeHelbowY[2] = { 0, 0 };
	double angleRangeHelbowZ[2] = { 0, 5 * SimTK::Pi / 6 };

	// Set the angle and position ranges for the coordinate set (SimTK:: prefix not actually needed here)
	jointCoordinateSetH[0].setRange(angleRangeHelbowX);
	jointCoordinateSetH[1].setRange(angleRangeHelbowY);
	jointCoordinateSetH[2].setRange(angleRangeHelbowZ);
	jointCoordinateSetH[3].setRange(positionRange);
	jointCoordinateSetH[4].setRange(positionRange);
	jointCoordinateSetH[5].setRange(positionRange);

	// Add a STL file 
	OpenSim::Body *stlPart = new OpenSim::Body("stlPart", upperArmMass, upperArmCenter, upperArmInertia);
	stlPart->addDisplayGeometry("drillPart.STL");
	Vec3 drillLocationInParent(-0.017545, -0.007, 0.17);
	PinJoint *stlT = new PinJoint("drillJoint", ground, drillLocationInParent, orientationInParent, *stlPart, locationInBody, orientationInBody);

	// Add the blocks lowerArm and upperArm to the model
	osimModel.addBody(upperArm);
	osimModel.addBody(lowerArm);
	osimModel.addBody(stlPart);

	// Define the acceleration of gravity
	osimModel.setGravity(Vec3(0, -9.80665, 0));

	// Save the model to a file .OSIM
	osimModel.print("rightArmModel.osim");

	// Part of the program used to modify orientation of the lowerArm
	State defaultState = osimModel.initSystem();
	// Get the initial state of the body
	osimModel.getVisualizer().show(defaultState);

	cout << "PLEASE TAP ON THE ORIENTATION KEYS -> UP, DOWN, LEFT or RIGHT (Tap 'q' to quit)" << endl;
	bool var = true;
	double value_x = 0;
	double value_y = 0;
	while (var) {
		//Get the key pressed by the user
		switch (getch()) {
		//key up
		case KEY_UP:
			cout << endl << "Up" << endl;
			value_x += SimTK::Pi / 4;
			break;
		// key down
		case KEY_DOWN:
			cout << endl << "Down" << endl;   
			value_x -= SimTK::Pi / 4;
			break;
		// key left
		case KEY_LEFT:
			cout << endl << "Left" << endl;  
			value_y -= SimTK::Pi / 4;
			break;
		// key right
		case KEY_RIGHT:
			cout << endl << "Right" << endl;  
			value_y += SimTK::Pi / 4;
			break;
		// 'q' to quit
		case 'q':
			var = false;
			break;
		default:
			break;
		}
		// Change the value of the helbow joint's rotations in the current state
		osimModel.setStateVariable(defaultState, "r_helbow_yRotation", value_x);
		osimModel.setStateVariable(defaultState, "r_helbow_zRotation", value_y);

		// Set the state for the vizualiseur
		osimModel.getVisualizer().show(defaultState);
		// Get the number of rotations and translations that can be set in the current state
		//const size_t sizeG = osimModel.getNumStateVariables();
	}
	// Display all the rotations and translations of the current state
	/*OpenSim::Array<string> tableau = osimModel.getStateVariableNames();
	cout << sizeG << endl;
	for (size_t i = 0; i < sizeG; ++i)
	{
	cout << tableau[i] << endl;
	}*/

	std::cout << "OpenSim example completed successfully.\n";
	std::cin.get();

	return 0;
}