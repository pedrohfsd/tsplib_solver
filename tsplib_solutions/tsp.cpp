#include "tsp_common.h"
#include "tsp_mip_mtz.h"

using namespace std;

int main(int argc, const char* argv[]) {
	try {
		if (argc != 2) {
			cerr << "Please specify a valid tsplib filepath as program argument!" << endl;
			throw(-1);
		}
		cout << "Loading file " << argv[1] << "..." << endl;

		//TSP_MIP_MTZ().run(*read("E:\\pedro\\projects\\tsplib_solutions\\res\\gr17.tsp"));
		TSP_MIP_MTZ().run(*read(argv[1]));
	}
	catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (const string& e) {
		cerr << "Exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "Unknown exception caught." << endl;
	}

	return 0;
}