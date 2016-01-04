#include <OpenSim/OpenSim.h>
#include <string>
#include <map>
#include <exception>
#include <iostream>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main()
{
	cout << "1" << endl;
	Model osimModel;
	osimModel.setName("bodyModel");
	//osimModel.setUseVisualizer(true);
	
	// Get a reference to the model's ground body
	OpenSim::Body& ground = osimModel.getGroundBody();

	//Map of body properties
	std::map< string, vector<string> > bodyNames;
	std::map<string, string> suffix;
	bodyNames["torso"] = { "hat_jaw", "hat_spine", "hat_skull", "hat_ribs_scap" };
	bodyNames["pelvis"] = { "sacrum", "pelvis", "l_pelvis" };

	bodyNames["femur_r"] = { "femur_r" };
	bodyNames["tibia_r"] = { "tibia_r", "fibula_r" };
	bodyNames["talus_r"] = { "talus_rv" };
	bodyNames["calcn_r"] = { "foot" };
	bodyNames["toes_r"] = { "bofoot" };

	bodyNames["femur_l"] = { "femur_l" };
	bodyNames["tibia_l"] = { "tibia_l", "fibula_l" };
	bodyNames["talus_l"] = { "talus_lv" };
	bodyNames["calcn_l"] = { "l_foot" };
	bodyNames["toes_l"] = { "l_bofoot" };

	bodyNames["humerus_r"] = { "humerus_rv" };
	bodyNames["ulna_r"] = { "ulna_rv" };
	bodyNames["radius_r"] = { "radius_rv" };
	bodyNames["hand_r"] = { "pisiform", "lunate", "scaphoid", "triquetrum", "hamate", "capitate", "trapezoid", "trapezium", "metacarpal2", "index_proximal", "index_medial", "index_distal", "metacarpal3", "middle_proximal", "middle_medial", "middle_distal", "metacarpal4", "ring_proximal", "ring_medial", "ring_distal", "metacarpal5", "little_proximal", "little_medial", "little_distal", "metacarpal1", "thumb_proximal", "thumb_distal" };
	suffix["hand_r"] = "_rsv";

	bodyNames["humerus_l"] = { "humerus_lv" };
	bodyNames["ulna_l"] = { "ulna_lv" };
	bodyNames["radius_l"] = { "radius_lv" };
	bodyNames["hand_l"] = { "pisiform", "lunate", "scaphoid", "triquetrum", "hamate", "capitate", "trapezoid", "trapezium", "metacarpal2", "index_proximal", "index_medial", "index_distal", "metacarpal3", "middle_proximal", "middle_medial", "middle_distal", "metacarpal4", "ring_proximal", "ring_medial", "ring_distal", "metacarpal5", "little_proximal", "little_medial", "little_distal", "metacarpal1", "thumb_proximal", "thumb_distal" };
	suffix["hand_l"] = "_lsv";

	std::map< string, vector<double> > bodyProp;
	bodyProp["torso"] = { 26.82, 0.1 };
	bodyProp["humerus_r"] = {0, 0};
	bodyProp["lowerArm"] = {20.0, 0.1};
	Vec3 center(0);

	std::string name, suf;
	Inertia inertia;
	double mass, length;
	std::map< string, OpenSim::Body* > bodies;
	//Creates all the bodies
	for (std::map< string, vector<string> >::const_iterator it = bodyNames.begin(); it != bodyNames.end(); ++it) {
		//Initialize variables
		mass = (bodyProp.find(it->first) != bodyProp.end()) ? bodyProp[it->first][0] : 0.;
		length = (bodyProp.find(it->first) != bodyProp.end()) ? bodyProp[it->first][1] : 0.;
		inertia = mass*Inertia::brick(length, length, length);
		suf = (suffix.find(it->first) != suffix.end()) ? suffix[it->first] : "";
		// Create a new block body with specified properties
		OpenSim::Body *b = new OpenSim::Body(it->first, mass, center, inertia);
		for (int i = 0; i < it->second.size(); i++) {
			name = it->second[i] + suf + ".vtp";
			b->addDisplayGeometry(name);
		}
		bodies[it->first] = b;
		//osimModel.addBody(b);
	} 
	cout << "2" << endl;
	//Map of the joints properties
	std::map< string, vector<string> > jointsNames;
	jointsNames["back"] = { "pelvis", "torso" };
	jointsNames["hip_r"] = {"pelvis", "femur_r"};
	jointsNames["hip_l"] = { "pelvis", "femur_l" };
	jointsNames["knee_r"] = { "femur_r", "tibia_r" };
	jointsNames["knee_l"] = { "femur_l", "tibia_l" };
	jointsNames["ankle_r"] = { "tibia_r","talus_r" };
	jointsNames["ankle_l"] = { "tibia_l","talus_l" };
	jointsNames["subtalar_r"] = { "talus_r", "calcn_r" };
	jointsNames["subtalar_l"] = { "talus_l", "calcn_l" };
	jointsNames["mtp_r"] = { "calcn_r", "toes_r" };
	jointsNames["mtp_l"] = { "calcn_l", "toes_l" };

	jointsNames["acromial_r"] = { "torso", "humerus_r" };
	jointsNames["acromial_l"] = { "torso", "humerus_l" };
	jointsNames["elbow_r"] = { "humerus_r", "ulna_r" };
	jointsNames["elbow_l"] = { "humerus_l", "ulna_l" };
	jointsNames["radioulnar_r"] = { "ulna_r", "radius_r" };
	jointsNames["radioulnar_l"] = { "ulna_l", "radius_l" };
	jointsNames["radius_hand_r"] = { "radius_r", "hand_r" };
	jointsNames["radius_hand_l"] = { "radius_l", "hand_l" };

	std::map< string, Vec3 > jointProp;
	jointProp["hip_r"] = Vec3 (-0.0707, - 0.0661, 0.0835);
	jointProp["ankle_r"] = Vec3 (0, -0.43, 0);
	jointProp["subtalar_r"] = Vec3 (-0.04877, - 0.04195, 0.00792);
	jointProp["mtp_r"] = Vec3(0.1788, - 0.002, 0.00108);
	jointProp["hip_l"] = Vec3(-0.0707, -0.0661, -0.0835);
	jointProp["ankle_l"] = Vec3(0, -0.43, 0);
	jointProp["subtalar_l"] = Vec3(-0.04877, -0.04195, -0.00792);
	jointProp["mtp_l"] = Vec3(0.1788, -0.002, -0.00108);
	jointProp["back"] = Vec3(-0.1007, 0.0815, 0);
	jointProp["acromial_r"] = Vec3(0.003155, 0.3715, 0.17);
	jointProp["elbow_r"] = Vec3(0.013144, - 0.286273, - 0.009595);
	jointProp["radioulnar_r"] = Vec3(-0.006727, - 0.013007, 0.026083);
	jointProp["radius_hand_r"] = Vec3(-0.008797, - 0.235841, 0.01361);
	jointProp["acromial_r"] = Vec3(0.003155, 0.3715, -0.17);
	jointProp["elbow_r"] = Vec3(0.013144, -0.286273, 0.009595);
	jointProp["radioulnar_r"] = Vec3(-0.006727, -0.013007, -0.026083);
	jointProp["radius_hand_r"] = Vec3(-0.008797, -0.235841, -0.01361);

	//Creates all the joints
	//Create a new pin joint between the ground and the pelvis
	Vec3 locationInParent(0), orientationInParent(0), locationInBody(0), orientationInBody(0);
	PinJoint *ground_pelvis = new PinJoint("ground_pelvis", ground, locationInParent, orientationInParent, *bodies["pelvis"], locationInBody, orientationInBody);
	
	std::map< string, OpenSim::FreeJoint* > joints;
	double positionRange[2] = { -0.01, 0.01 };
	for (std::map< string, vector<string> >::const_iterator it = jointsNames.begin(); it != jointsNames.end(); ++it) {
		locationInParent = (jointProp.find(it->first) != jointProp.end()) ? jointProp[it->first] : Vec3 (0);
		FreeJoint *j = new FreeJoint(it->first, *bodies[it->second[0]], locationInParent, orientationInParent, *bodies[it->second[1]], locationInBody, orientationInBody);
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
	cout << "3" << endl;
	// Add every body to the model
	for (std::map< string, OpenSim::Body* >::const_iterator it = bodies.begin(); it != bodies.end(); ++it) {
		cout << bodies[it->first]->getName() << endl;
		OpenSim::Body *b(it->second);
		osimModel.addBody(b);
		//cout << "hello3.5" << endl;
	}
	cout << "hello" << endl;
	for (std::map< string, OpenSim::FreeJoint* >::const_iterator it = joints.begin(); it != joints.end(); ++it) {
		cout << joints[it->first]->getName() << endl;

		//osimModel.addBody((OpenSim::Body*)(it->second));
		//cout << "hello3.5" << endl;
	}
	
	cout << "4" << endl;
	// Define the acceleration of gravity
	osimModel.setGravity(Vec3(0, -9.80665, 0));

	// Save the model to a file
	osimModel.print("bodyModel.osim");
	//osimModel.initSystem();

	std::cout << "OpenSim example completed successfully  .\n";
	std::cin.get();
	
	return 0;
}