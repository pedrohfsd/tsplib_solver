#include <iostream>

#include "CuttingPlane_MIP.h"
#include "CuttingPlane_MIP_Callback.h"
#include "CuttingPlane_LP.h"
#include "CuttingPlane_LP_Callback.h"
#include "Data.h"
#include "FileParser.h"
#include "MTZ.h"

using namespace std;

int main(int argc, const char* argv[]) {
	//try {
		if (argc != 3) {
			cout << "There are one or more parameters missing" << endl;
			cout << "Correct usage is: program [formulation] [filePath]" << endl;
			cout << "where [formulation] is one of: {'MTZ', 'CuttingPlane_MIP_1', 'CuttingPlane_MIP_2', 'CuttingPlane_LP'}" << endl;
			cout << "and [filePath] is a path to a tsplib file" << endl;
			return 0;
		}
		const char* param1 = argv[1];
		const char* param2 = argv[2];
		cout << "Loading file " << param2 << "..." << endl;

		Data data;
		FileParser parser;
		//parser.createTestData(data);
		parser.read(param2, data);

		if(!strcmp(param1, "MTZ")) MTZ().run(data);
		else if (!strcmp(param1, "CuttingPlane_MIP_1")) CuttingPlane_MIP().run(data, true);
		else if (!strcmp(param1, "CuttingPlane_MIP_2")) CuttingPlane_MIP().run(data, false);
		else if (!strcmp(param1, "CuttingPlane_MIP_Callback")) CuttingPlane_MIP_Callback().run(data, true);
		else if (!strcmp(param1, "CuttingPlane_LP")) CuttingPlane_LP().run(data);
		else if (!strcmp(param1, "CuttingPlane_LP_Callback")) CuttingPlane_LP_Callback().run(data);
	//}
	//catch (IloException& e) {
	//	cerr << "Concert exception caught: " << e << endl;
	//}
	//catch (const string& e) {
	//	cerr << "Exception caught: " << e << endl;
	//}
	//catch (exception& e) {
	//	cerr << "Exception caught: " << e.what() << endl;
	//}
	//catch (...) {
	//	cerr << "Unknown exception caught." << endl;
	//}
	return 0;
}