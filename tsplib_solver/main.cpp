#include "CuttingPlaneCrossing.h"
#include "CuttingPlaneSubtour.h"
#include "CuttingPlaneFlow.h"
#include "Data.h"
#include "FileParser.h"
#include "MTZ.h"


#include <iostream>

using namespace std;

int main(int argc, const char* argv[]) {
#ifndef _DEBUG	
	try { 
#endif
		if (argc != 4) {
			cout << "There are one or more parameters missing" << endl;
			cout << "Correct usage is: program [formulation] [singleThreaded] [filePath]" << endl;
			cout << "where [formulation] is one of: {'MTZ', 'CuttingPlaneCrossing', 'CuttingPlaneSubtour', 'CuttingPlaneFlow'}" << endl;
			cout << "[singleThreaded] is one of: {'true' or 'false'}" << endl;
			cout << "and [filePath] is a path to a tsplib file" << endl;
			return 0;
		}
		const char* param1 = argv[1];
		const bool param2 = !strcmp(argv[2], "true") ? true : false;
		const char* param3 = argv[3];
		cout << "Loading file " << param3 << "..." << endl;

		Data data;
		FileParser fileParser(data);
		fileParser.parse(param3);

		if(!strcmp(param1, "MTZ")) MTZ(data, param2).solve();
		else if (!strcmp(param1, "CuttingPlaneCrossing")) CuttingPlaneCrossing(data, param2).solve();
		else if (!strcmp(param1, "CuttingPlaneSubtour")) CuttingPlaneSubtour(data, param2).solve();
		else if (!strcmp(param1, "CuttingPlaneFlow")) CuttingPlaneFlow(data, param2).solve();
		else cout << "Invalid formulation parameter!" << endl;
#ifndef _DEBUG 
	}
	catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (const string& e) {
		cerr << "Exception caught: " << e << endl;
	}
	catch (exception& e) {
		cerr << "Exception caught: " << e.what() << endl;
	}
	catch (...) {
		cerr << "Unknown exception caught." << endl;
	}
#endif
	return 0;
}