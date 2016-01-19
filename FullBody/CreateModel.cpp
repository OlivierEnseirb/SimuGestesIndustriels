/* -------------------------------------------------------------------------- *
*                    OpenSim:  CreateModel.cpp                     *
* -------------------------------------------------------------------------- *
* Fichier complet permettant d'ouvrir le model d'un corps entier avec        *
* muscles, marqueur															 *
* -------------------------------------------------------------------------- */

// Author:  Olivier Hartmann, Léa Pillette

//==============================================================================
//==============================================================================
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
	//Creation of the model which will be composed of all the elements

	try{
		Model osimCreated("bodyModel.osim");
		cout << "hello" << endl;
		osimCreated.setUseVisualizer(true);
		// Part of the program used to modify orientation of the lowerArm
		State defaultState = osimCreated.initSystem();
		// Set the state for the vizualiseur
		osimCreated.getVisualizer().show(defaultState);

	}
	catch(const OpenSim::Exception e){
		cerr << e.getMessage();
		cout << "hi" << endl;
	}
	std::cin.get();
}