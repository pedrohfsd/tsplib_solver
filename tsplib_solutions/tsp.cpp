#include "tsp_common.h"
#include "tsp_mip_mtz.cpp"
//#include "tsplib.cpp"

int main(int argc, const char* argv[]){
	try {
		if (argc != 2) {
			printf("Please specify a valid tsplib filepath as program argument");
			throw(-1);
		}

		//return TSP_MIP_MTZ().run(*createTestData());
		TSP_MIP_MTZ().run(*read(argv[1]));
	}
	catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "Unknown exception caught" << endl;
	}

	return 0;
}