#include <functional>
#include <iostream>
#include <ilcplex/ilocplex.h>
#include <unordered_map>
#include <vector>

#include "Data.h"
#include "Edge.h"
#include "FileParser.h"
#include "MTZ.h"
#include "Vertex.h"

using namespace std;

int main(int argc, const char* argv[]) {
	try {
		if (argc != 2) {
			throw("Please specify a valid tsplib filepath as program argument!");
		}
		Data data;
		FileParser parser;

		cout << "Loading file " << argv[1] << "..." << endl;
		parser.read(argv[1], data);

		MTZ().run(data);
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