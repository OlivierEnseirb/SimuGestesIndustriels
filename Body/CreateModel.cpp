/* -------------------------------------------------------------------------- *
*                    OpenSim:  CreateModel.cpp                     *
* -------------------------------------------------------------------------- *
* Fichier plus complet permettant de créer le model d'un corps entier        *
* -------------------------------------------------------------------------- */

// Author:  Olivier Hartmann, Léa Pillette

//==============================================================================
//==============================================================================

#include <OpenSim/OpenSim.h>createBodyPropMap
#include <string>
#include <map>
#include <exception>
#include <iostream>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//Map associating body part name and the bones that are in this body part
void createBodyNamesMap(std::map< string, vector<string> > &bodyNames, std::map<string, string> &suffix) {
	//Main
	bodyNames["torso"] = { "hat_jaw", "hat_spine", "hat_skull", "hat_ribs_scap" };
	bodyNames["pelvis"] = { "sacrum", "pelvis", "l_pelvis" };

	//Right leg
	bodyNames["femur_r"] = { "femur_r" };
	bodyNames["tibia_r"] = { "tibia_r", "fibula_r" };
	bodyNames["talus_r"] = { "talus_rv" };
	bodyNames["calcn_r"] = { "foot" };
	bodyNames["toes_r"] = { "bofoot" };

	//Left leg
	bodyNames["femur_l"] = { "femur_l" };
	bodyNames["tibia_l"] = { "tibia_l", "fibula_l" };
	bodyNames["talus_l"] = { "talus_lv" };
	bodyNames["calcn_l"] = { "l_foot" };
	bodyNames["toes_l"] = { "l_bofoot" };

	//Right arm
	bodyNames["humerus_r"] = { "humerus_rv" };
	bodyNames["ulna_r"] = { "ulna_rv" };
	bodyNames["radius_r"] = { "radius_rv" };
	bodyNames["hand_r"] = { "pisiform", "lunate", "scaphoid", "triquetrum", "hamate", "capitate", "trapezoid", "trapezium", "metacarpal2", "index_proximal", "index_medial", "index_distal", "metacarpal3", "middle_proximal", "middle_medial", "middle_distal", "metacarpal4", "ring_proximal", "ring_medial", "ring_distal", "metacarpal5", "little_proximal", "little_medial", "little_distal", "metacarpal1", "thumb_proximal", "thumb_distal" };
	suffix["hand_r"] = "_rvs";

	//Left arm
	bodyNames["humerus_l"] = { "humerus_lv" };
	bodyNames["ulna_l"] = { "ulna_lv" };
	bodyNames["radius_l"] = { "radius_lv" };
	bodyNames["hand_l"] = { "pisiform", "lunate", "scaphoid", "triquetrum", "hamate", "capitate", "trapezoid", "trapezium", "metacarpal2", "index_proximal", "index_medial", "index_distal", "metacarpal3", "middle_proximal", "middle_medial", "middle_distal", "metacarpal4", "ring_proximal", "ring_medial", "ring_distal", "metacarpal5", "little_proximal", "little_medial", "little_distal", "metacarpal1", "thumb_proximal", "thumb_distal" };
	suffix["hand_l"] = "_lvs";
}

//Properties of the body part (can be skiped)
void createBodyPropMap(std::map <string, vector<double> > &bodyProp) {
	bodyProp["torso"] = { 26.82, 0.1 };
	bodyProp["humerus_r"] = { 0, 0 };
	bodyProp["lowerArm"] = { 20.0, 0.1 };
}

//Map associating of the joints names and the name of the bodies they link
void createJointNamesMap(std::map< string, vector<string> > &jointNames) {
	jointNames["back"] = { "pelvis", "torso" };

	//Legs
	jointNames["hip_r"] = { "pelvis", "femur_r" };
	jointNames["hip_l"] = { "pelvis", "femur_l" };
	jointNames["knee_r"] = { "femur_r", "tibia_r" };
	jointNames["knee_l"] = { "femur_l", "tibia_l" };
	jointNames["ankle_r"] = { "tibia_r","talus_r" };
	jointNames["ankle_l"] = { "tibia_l","talus_l" };
	jointNames["subtalar_r"] = { "talus_r", "calcn_r" };
	jointNames["subtalar_l"] = { "talus_l", "calcn_l" };
	jointNames["mtp_r"] = { "calcn_r", "toes_r" };
	jointNames["mtp_l"] = { "calcn_l", "toes_l" };

	//Arms
	jointNames["acromial_r"] = { "torso", "humerus_r" };
	jointNames["acromial_l"] = { "torso", "humerus_l" };
	jointNames["elbow_r"] = { "humerus_r", "ulna_r" };
	jointNames["elbow_l"] = { "humerus_l", "ulna_l" };
	jointNames["radioulnar_r"] = { "ulna_r", "radius_r" };
	jointNames["radioulnar_l"] = { "ulna_l", "radius_l" };
	jointNames["radius_hand_r"] = { "radius_r", "hand_r" };
	jointNames["radius_hand_l"] = { "radius_l", "hand_l" };
}

//Properties of the joints (can be skiped)
void createJointPropMap(std::map< string, Vec3 > &jointProp) {
	jointProp["back"] = Vec3(-0.1007, 0.0815, 0);

	//Legs
	jointProp["hip_r"] = Vec3(-0.0707, -0.0661, 0.0835);
	jointProp["knee_r"] = Vec3(0.0, -0.39, 0.0);
	jointProp["ankle_r"] = Vec3(0, -0.43, 0);
	jointProp["subtalar_r"] = Vec3(-0.04877, -0.04195, 0.00792);
	jointProp["mtp_r"] = Vec3(0.1788, -0.002, 0.00108);

	jointProp["hip_l"] = Vec3(-0.0707, -0.0661, -0.0835);
	jointProp["knee_l"] = Vec3(0.0, -0.39, 0.0);
	jointProp["ankle_l"] = Vec3(0, -0.43, 0);
	jointProp["subtalar_l"] = Vec3(-0.04877, -0.04195, -0.00792);
	jointProp["mtp_l"] = Vec3(0.1788, -0.002, -0.00108);

	//Arms
	jointProp["acromial_r"] = Vec3(0.003155, 0.3715, 0.17);
	jointProp["elbow_r"] = Vec3(0.013144, -0.286273, -0.009595);
	jointProp["radioulnar_r"] = Vec3(-0.006727, -0.013007, 0.026083);
	jointProp["radius_hand_r"] = Vec3(-0.008797, -0.235841, 0.01361);

	jointProp["acromial_l"] = Vec3(0.003155, 0.3715, -0.17);
	jointProp["elbow_l"] = Vec3(0.013144, -0.286273, 0.009595);
	jointProp["radioulnar_l"] = Vec3(-0.006727, -0.013007, -0.026083);
	jointProp["radius_hand_l"] = Vec3(-0.008797, -0.235841, -0.01361);
}

//Map associating muscles and their pathpoints
void createMuscleMap(std::map< string, vector<pair<string, Vec3> >> &muscles) {
	//Right Upper Arm
	muscles["TriLongR"] = { { "torso", Vec3(-0.01, 0.36, 0.14723) },{ "humerus_r", Vec3(-0.02714, -0.11441, -0.00664) },{ "humerus_r", Vec3(-0.03184, -0.22637, -0.01217) },{ "humerus_r", Vec3(-0.01743, -0.26757, -0.01208) },{ "radius_r", Vec3(-0.0219, 0.01046, -0.00078) } };
	muscles["TriLatR"] = { { "humerus_r", Vec3(-0.00599, -0.12646, 0.00428) },{ "humerus_r", Vec3(-0.02344, -0.14528, 0.00928) },{ "humerus_r", Vec3(-0.03184, -0.22637, -0.01217) },{ "humerus_r", Vec3(-0.01743, -0.26757, -0.01208) },{ "radius_r", Vec3(-0.0219, 0.01046, -0.00078) } };
	muscles["TriMedR"] = { { "humerus_r", Vec3(-0.00838, -0.13695, -0.00906) },{ "humerus_r", Vec3(-0.02601, -0.15139, -0.0108) },{ "humerus_r", Vec3(-0.03184, -0.22637, -0.01217) },{ "humerus_r", Vec3(-0.01743, -0.26757, -0.01208) },{ "radius_r", Vec3(-0.0219, 0.01046, -0.00078) } };
	muscles["BicLongR"] = { {"torso", Vec3(-0.039235, 0.36, 0.14795)}, {"torso", Vec3(-0.028945, 0.36, 0.15639)}, {"humerus_r", Vec3(0.02131, 0.01793, 0.01028)}, {"humerus_r", Vec3(0.02378, - 0.00511, 0.01201)},{ "humerus_r", Vec3(0.02, - 0.02827, 0.00136) },{ "humerus_r", Vec3(0.02, - 0.07736, - 0.00165) },{ "humerus_r", Vec3(0.01703, - 0.12125, 0.00024) },{ "humerus_r", Vec3(0.0228, - 0.1754, - 0.0063) }, {"radius_r", Vec3(0.00751, 0.024, 0.0)} };
	muscles["BicShortR"] = { { "torso", Vec3(0.004675, 0.36 , 0.15) },{ "torso", Vec3(-0.007075, 0.36, 0.15) },{ "humerus_r", Vec3(0.01117, -0.07576, -0.01101) },{ "humerus_r", Vec3(0.01703, -0.12125, -0.01079) },{ "humerus_r", Vec3(0.0228, -0.1754, -0.0063) },{ "radius_r", Vec3(0.00751, 0.024, 0.0) } };
	muscles["BraR"] = { { "humerus_r", Vec3(0.009, - 0.1739, - 0.0036) },{ "radius_r", Vec3(0.023, - 0.0239, 0.001) } };

	//Left Upper Arm
	muscles["TriLongL"] = { { "torso", Vec3(-0.01, 0.36, -0.14723) },{ "humerus_l", Vec3(-0.02714, -0.11441, 0.00664) },{ "humerus_l", Vec3(-0.03184, -0.22637, 0.01217) },{ "humerus_l", Vec3(-0.01743, -0.26757, 0.01208) },{ "radius_l", Vec3(-0.0219, 0.01046, 0.00078) } };
	muscles["TriLatL"] = { { "humerus_l", Vec3(-0.00599, -0.12646, -0.00428) },{ "humerus_l", Vec3(-0.02344, -0.14528, -0.00928) },{ "humerus_l", Vec3(-0.03184, -0.22637, 0.01217) },{ "humerus_l", Vec3(-0.01743, -0.26757, 0.01208) },{ "radius_l", Vec3(-0.0219, 0.01046, 0.00078) } };
	muscles["TriMedL"] = { { "humerus_l", Vec3(-0.00838, -0.13695, 0.00906) },{ "humerus_l", Vec3(-0.02601, -0.15139, 0.0108) },{ "humerus_l", Vec3(-0.03184, -0.22637, 0.01217) },{ "humerus_l", Vec3(-0.01743, -0.26757, 0.01208) },{ "radius_l", Vec3(-0.0219, 0.01046, 0.00078) } };
	muscles["BicLongL"] = { { "torso", Vec3(-0.039235, 0.36, -0.14795) },{ "torso", Vec3(-0.028945, 0.36, -0.15639) },{ "humerus_l", Vec3(0.02131, 0.01793, -0.01028) },{ "humerus_l", Vec3(0.02378, -0.00511, -0.01201) },{ "humerus_l", Vec3(0.02, -0.02827, -0.00136) },{ "humerus_l", Vec3(0.02, -0.07736, 0.00165) },{ "humerus_l", Vec3(0.01703, -0.12125, -0.00024) },{ "humerus_l", Vec3(0.0228, -0.1754, 0.0063) },{ "radius_l", Vec3(0.00751, 0.024, 0.0) } };
	muscles["BicShortL"] = { { "torso", Vec3(0.004675, 0.36 , -0.15) },{ "torso", Vec3(-0.007075, 0.36, -0.15) },{ "humerus_l", Vec3(0.01117, -0.07576, 0.01101) },{ "humerus_l", Vec3(0.01703, -0.12125, 0.01079) },{ "humerus_l", Vec3(0.0228, -0.1754, 0.0063) },{ "radius_l", Vec3(0.00751, 0.024, 0.0) } };
	muscles["BraL"] = { { "humerus_l", Vec3(0.009, -0.1739, 0.0036) },{ "radius_l", Vec3(0.023, -0.0239, -0.001) } };

	//Right Lower Arm
	muscles["EcrbR"] = { { "humerus_r", Vec3(0.02254, -0.26813, 0.00679) },{ "radius_r", Vec3(-0.00384, -0.1248, 0.00482) },{ "radius_r", Vec3(-0.01733, -0.22, 0.0004) } };
	muscles["EcrlR"] = { { "humerus_r", Vec3(0.01337, - 0.26111, 0.00069) },{ "humerus_r", Vec3(0.02239, - 0.26834, 0.00679) },{ "radius_r", Vec3(-0.00136, - 0.12953, 0.00532) },{ "radius_r", Vec3(-0.0083, - 0.21052, 0.0052) } };
	muscles["FcuR"] = { { "humerus_r", Vec3(0.00463, -0.27768, 0.01318) },{ "ulna_r", Vec3(-0.00465, -0.22771, 0.05589) },{ "ulna_r", Vec3(-0.01368, -0.22899, 0.05805) } };
	muscles["FcrR"] = { { "humerus_r", Vec3(0.00639, - 0.27761, 0.01313) },{ "radius_r", Vec3(0.00953, - 0.21667, 0.035) } };

	//Left Lower Arm
	muscles["EcrbL"] = { { "humerus_l", Vec3(0.02254, -0.26813, -0.00679) },{ "radius_l", Vec3(-0.00384, -0.1248, -0.00482) },{ "radius_l", Vec3(-0.01733, -0.22, -0.0004) } };
	muscles["EcrlL"] = { { "humerus_l", Vec3(0.01337, -0.26111, -0.00069) },{ "humerus_l", Vec3(0.02239, -0.26834, -0.00679) },{ "radius_l", Vec3(-0.00136, -0.12953, -0.00532) },{ "radius_l", Vec3(-0.0083, -0.21052, -0.0052) } };
	muscles["FcuL"] = { { "humerus_l", Vec3(0.00463, -0.27768, -0.01318) },{ "ulna_l", Vec3(-0.00465, -0.22771, -0.05589) },{ "ulna_l", Vec3(-0.01368, -0.22899, -0.05805) } };
	muscles["FcrL"] = { { "humerus_l", Vec3(0.00639, -0.27761, -0.01313) },{ "radius_l", Vec3(0.00953, -0.21667, -0.035) } };

	//Right Torso
	muscles["ErcSpnR"] = { { "pelvis", Vec3(-0.14000000, 0.04390000, 0.04360000) },{ "torso", Vec3(-0.05500000, 0.11000000, 0.02410000) } };
	muscles["IntOblR"] = { { "pelvis", Vec3(-0.14000000, 0.07000000, 0.11570000) },{ "torso", Vec3(0.07000000, 0.16000000, 0.01500000) } };
	muscles["ExtOblR"] = { { "pelvis", Vec3(-0.03000000, - 0.06360000, 0.01000000) },{ "torso", Vec3(0.06500000, 0.11000000, 0.11000000) } };

	//Left Torso
	muscles["ErcSpnL"] = { { "pelvis", Vec3(-0.14000000, 0.04390000, -0.04360000) },{ "torso", Vec3(-0.05500000, 0.11000000, -0.02410000) } };
	muscles["IntOblL"] = { { "pelvis", Vec3(-0.14000000, 0.07000000, -0.11570000) },{ "torso", Vec3(0.07000000, 0.16000000, -0.01500000) } };
	muscles["ExtOblL"] = { { "pelvis", Vec3(-0.03000000, -0.06360000, -0.01000000) },{ "torso", Vec3(0.06500000, 0.11000000, -0.11000000) } };

}

int main()
{
	//Creation of the model which will be composed of all the elements
	Model osimModel;
	osimModel.setName("bodyModel");
	//Allow real time visualization of the model
	//osimModel.setUseVisualizer(true);

	// Get a reference to the model's ground body
	OpenSim::Body& ground = osimModel.getGroundBody();
	
	//Map associating body part name and the bones that are in this body part
	std::map< string, vector<string> > bodyNames;

	//Map associating the body part name and the suffix necessary to acces the .VTP bone file
	std::map<string, string> suffix;

	//Creates the bodyNames map and the suffix map
	createBodyNamesMap(bodyNames, suffix);
	
	//Properties of the body part (can be skiped)
	std::map< string, vector<double> > bodyProp;
	createBodyPropMap(bodyProp);
	
	Vec3 center(0);
	std::string name, suf;
	Inertia inertia;
	double mass, length;
	//Contains all the bodies accessible by their body names
	std::map< string, OpenSim::Body* > bodies;
	//Creates all the bodies (= body parts)
	for (std::map< string, vector<string> >::const_iterator it = bodyNames.begin(); it != bodyNames.end(); ++it) {
		//Initialize variables linked to the body with the bodyProp map 
		mass = (bodyProp.find(it->first) != bodyProp.end()) ? bodyProp[it->first][0] : 0.;
		length = (bodyProp.find(it->first) != bodyProp.end()) ? bodyProp[it->first][1] : 0.;
		inertia = mass*Inertia::brick(length, length, length);
		suf = (suffix.find(it->first) != suffix.end()) ? suffix[it->first] : "";
		// Create a new block body with specified properties
		OpenSim::Body *b = new OpenSim::Body(it->first, mass, center, inertia);
		// Associate the bones to the body
		for (int i = 0; i < it->second.size(); i++) {
			name = it->second[i] + suf + ".vtp";
			b->addDisplayGeometry(name);
		}
		// Store the bodies in the map
		bodies[it->first] = b;
	}

	//Map associating of the joints names and the name of the bodies they link
	std::map< string, vector<string> > jointNames;
	createJointNamesMap(jointNames);

	//Properties of the joints (can be skiped)
	std::map< string, Vec3 > jointProp;
	createJointPropMap(jointProp);

	//Create a new pin joint between the ground and the pelvis
	Vec3 locationInParent(0), orientationInParent(0), locationInBody(0), orientationInBody(0);
	PinJoint *ground_pelvis = new PinJoint("ground_pelvis", ground, locationInParent, orientationInParent, *bodies["pelvis"], locationInBody, orientationInBody);
	
	//Contains all the joints accessible by their joints names
	std::map< string, OpenSim::FreeJoint* > joints;
	double positionRange[2] = { -0.01, 0.01 };
	//Creates all the joints using the list of joint names contained in the map named jointNames
	for (std::map< string, vector<string> >::const_iterator it = jointNames.begin(); it != jointNames.end(); ++it) {
		//Get property of the joint if it exists
		locationInParent = (jointProp.find(it->first) != jointProp.end()) ? jointProp[it->first] : Vec3(0);
		//Create the new joint
		FreeJoint *j = new FreeJoint(it->first, *bodies[it->second[0]], locationInParent, orientationInParent, *bodies[it->second[1]], locationInBody, orientationInBody);
		// Store the bodies in the map
		joints[it->first] = j;
		// Get a reference to the coordinate set (6 degrees-of-freedom) between the block and ground bodies
		CoordinateSet& jointCoordinateSetH = j->upd_CoordinateSet();
		// Set the angle and position ranges for the coordinate set (SimTK:: prefix not actually needed here)
		double angleRangeHelbowX[2] = { 0, 0 };
		double angleRangeHelbowY[2] = { 0, 0 };
		double angleRangeHelbowZ[2] = { -5 * SimTK::Pi / 6, 5 * SimTK::Pi / 6 };
		// Set the angle and position ranges for the coordinate set (SimTK:: prefix not actually needed here)
		jointCoordinateSetH[0].setRange(angleRangeHelbowX);
		jointCoordinateSetH[1].setRange(angleRangeHelbowY);
		jointCoordinateSetH[2].setRange(angleRangeHelbowZ);
		jointCoordinateSetH[3].setRange(positionRange);
		jointCoordinateSetH[4].setRange(positionRange);
		jointCoordinateSetH[5].setRange(positionRange);
	}
	
	// Add every body to the model
	std::map< string, OpenSim::Body* > bodies_copy(bodies);
	int loop = 0, number = 0;
	enum AddedState { NOT_ADDED, ADDED, NOT_FOUND };
	AddedState addState = NOT_FOUND;
	while (bodies_copy.size()>0)
	{
		//Go thought all the bodies in the map bodies
		for (std::map< string, OpenSim::Body* >::const_iterator it = bodies_copy.begin(); it != bodies_copy.end(); ++it)
		{
			string body_name = it->first;
			//cout << "Body Name : "<<body_name << endl;
			addState = NOT_FOUND;
			for (std::map< string, vector<string> >::const_iterator it_joint = jointNames.begin(); it_joint != jointNames.end(); ++it_joint)
			{
				string child_name = it_joint->second[1];
				//cout << "child_name : " << child_name << endl;
				if (child_name.compare(body_name) != 0)
					continue;

				string parent_name = it_joint->second[0];
				//cout << "parent_name : " << parent_name << endl;
				if (bodies_copy.find(parent_name) == bodies_copy.end())
				{// parent has already been added in Model
					addState = ADDED;
					//cout << "Added"<<endl;
				}
				else
				{// the parent is still in the list
				 //cout << "No added" << endl;
					addState = NOT_ADDED;
				}
				break;
			}
			if (addState == ADDED || addState == NOT_FOUND)
			{
				osimModel.addBody(it->second);
				bodies_copy.erase(it);
				break;
			}
		}
	}

	//Add muscles to the body
	std::map< string, vector<pair<string, Vec3> >> muscleNames;
	createMuscleMap(muscleNames);
	double maxIsometricForce = 1000.0, optimalFiberLength = 0.1, tendonSlackLength = 0.2, pennationAngle = 0.0, activation = 0.0001, deactivation = 1.0;
	
	//Contains all the muscles accessible by their names
	std::map< string, OpenSim::Thelen2003Muscle > muscles;
	//Creates and add the muscles to the model
	for (std::map< string, vector<pair<string, Vec3> >>::const_iterator it = muscleNames.begin(); it != muscleNames.end(); ++it) {
		// Create a new muscle using the Shutte 1993 muscle model
		OpenSim::Thelen2003Muscle *muscle = new Thelen2003Muscle(it->first, maxIsometricForce, optimalFiberLength, tendonSlackLength, pennationAngle);
		
		muscle->setActivationTimeConstant(activation);
		muscle->setDeactivationTimeConstant(deactivation);

		// Specify the paths for the muscle
		vector<pair<string, Vec3>> pathPoint = it->second;
		string name;
		
		//Create the path of the muscle
		for (int i = 0; i < pathPoint.size(); i++) {
			name = it->first + to_string(i + 1);
			muscle->addNewPathPoint(name, *bodies[pathPoint[i].first], pathPoint[i].second);
		}
		muscles[it->first] = *muscle;

		// Add the two muscles (as forces) to the model
		osimModel.addForce(muscle);
	}
	
	//Create the markers
	OpenSim::Mark *M = new OpenSim::Marker();
	M->

	// Define the acceleration of gravity
	osimModel.setGravity(Vec3(0, -9.80665, 0));

	// Save the model to a file
	osimModel.print("bodyModel.osim");
	//osimModel.initSystem();

	std::cout << "OpenSim example completed successfully.\n";
	std::cin.get();

	return 0;
}