#include <fstream>
#include "tsp_common.h"

using namespace std;

int attDistance(double*, double*);
Data* buildDataFromNodes(double**, int, int(*f)(double*, double*));
Data* buildDataFromEdges(double** nodes, int n);
Data* createTestData();
int euclidianDistance2D(double*, double*);
int geoDistance(double*, double*);
int nint(double);
double** readLowDiagEdges2D(ifstream&, const int);
double** readNodes2D(ifstream&, const int);
unordered_map<string, string>* readSpecification(ifstream&);

Data* read(const string& filepath) {
	ifstream infile(filepath);
	unordered_map<string, string>* spec = readSpecification(infile);
	int n = stoi((*spec)["DIMENSION"]);

	int(*distance_function)(double*, double*) = NULL;
	if ((*spec)["EDGE_WEIGHT_TYPE"] == "EUC_2D") distance_function = euclidianDistance2D;
	else if ((*spec)["EDGE_WEIGHT_TYPE"] == "ATT") distance_function = attDistance;
	else if ((*spec)["EDGE_WEIGHT_TYPE"] == "GEO") distance_function = geoDistance;
	else if ((*spec)["EDGE_WEIGHT_TYPE"] == "EXPLICIT") distance_function = NULL;
	else throw("Unsupported distance function");

	Data* data = NULL;
	if ((*spec)["NODE_COORD_SECTION"] == "true") data = buildDataFromNodes(readNodes2D(infile, n), n, distance_function);
	else if ((*spec)["EDGE_WEIGHT_FORMAT"] == "LOWER_DIAG_ROW") data = buildDataFromEdges(readLowDiagEdges2D(infile, n), n);

	return data;
};

unordered_map<string, string>* readSpecification(ifstream& infile){
	unordered_map<string, string>* spec = new unordered_map<string, string>();
	string line;
	while (getline(infile, line)) {
		size_t pos = line.find(":");
		if (pos == string::npos) break;
		replace(line.begin(), line.end(), ':', ' ');
		istringstream iss(line);
		string key; string value;
		if (!(iss >> key >> value)) { break; }
		(*spec)[key] = value;
	}
	(*spec)[line] = "true";
	return spec;
}

double** readNodes2D(ifstream& infile, const int dimension){
	string line;
	double** nodes = new double*[dimension];
	while (getline(infile, line)) {
		istringstream iss(line);
		int id; double x, y;
		if (!(iss >> id >> x >> y)) break;
		nodes[id-1] = new double[2];
		nodes[id-1][0] = x;
		nodes[id-1][1] = y;
	}
	return nodes;
}

double** readLowDiagEdges2D(ifstream& infile, const int dimension) {
	double** edges = new double*[dimension];
	for (int i = 0; i < dimension; i++) edges[i] = new double[dimension];

	int pos = 0;
	for (int i = 0; i < dimension; i++)
	{
		for (int j = 0; j <= pos; j++)
		{
			infile >> edges[i][j];
			edges[j][i] = edges[i][j];
		}
		pos++;
	}
	return edges;
}

Data* buildDataFromNodes(double** nodes, int n, int(*distance_function)(double*, double*)){
	Data* data = new Data();
	data->n = n;
	data->vertices = new Vertice[n];
	for (int i = 0; i < n; i++)	{
		Vertice& vertice = data->vertices[i];
		vertice.m = n;
		vertice.edges = new Edge[n];
		for (int j = 0; j < n; j++)	{
			vertice.edges[j].cost = (*distance_function)(nodes[i], nodes[j]);
			vertice.edges[j].id = n * i + j;
		}
	}
	return data;
}

Data* buildDataFromEdges(double** nodes, int n) {
	Data* data = new Data();
	data->n = n;
	data->vertices = new Vertice[n];
	for (int i = 0; i < n; i++) {
		Vertice& vertice = data->vertices[i];
		vertice.m = n;
		vertice.edges = new Edge[n];
		for (int j = 0; j < n; j++) {
			vertice.edges[j].cost = nodes[i][j];
			vertice.edges[j].id = n * i + j;
		}
	}
	return data;
}

int euclidianDistance2D(double* node1, double* node2){
	double x[2] = { node1[0], node2[0] };
	double y[2] = { node1[1], node2[1] };
	double xd = x[0] - x[1];
	double yd = y[0] - y[1];
	return nint(sqrt(xd*xd + yd*yd));
}

int attDistance(double* node1, double* node2) {
	double x[2] = { node1[0], node2[0] };
	double y[2] = { node1[1], node2[1] };
	double xd = x[0] - x[1];
	double yd = y[0] - y[1];
	double rij = sqrt((xd*xd + yd*yd) / 10.0);
	int tij = nint(rij);
	if (tij < rij) return tij + 1;
	return tij;
}

int geoDistance(double* node1, double* node2) {
	double x[2] = { node1[0], node2[0] };
	double y[2] = { node1[1], node2[1] };
	double PI = 3.141592;
	double latitude[2];
	double longitude[2];

	int deg = nint(x[0]);
	double min = x[0] - deg;
	latitude[0] = PI * (deg + 5.0 * min / 3.0) / 180.0;
	deg = nint(y[0]);
	min = y[0] - deg;
	longitude[0] = PI * (deg + 5.0 * min / 3.0) / 180.0;

	deg = nint(x[1]);
	min = x[1] - deg;
	latitude[1] = PI * (deg + 5.0 * min / 3.0) / 180.0;
	deg = nint(y[1]);
	min = y[1] - deg;
	longitude[1] = PI * (deg + 5.0 * min / 3.0) / 180.0;

	double RRR = 6378.388;
	double q1 = cos(longitude[0] - longitude[1]);
	double q2 = cos(latitude[0] - latitude[1]);
	double q3 = cos(latitude[0] + latitude[1]);
	return (int)(RRR * acos(0.5*((1.0 + q1)*q2 - (1.0 - q1)*q3)) + 1.0);
}

int nint(double x) {
	return (int)(x + 0.5);
}

Data* createTestData() {
	int c0[4] = { 5,8,2,1 };
	int c1[4] = { 5,1,5,3 };
	int c2[4] = { 6,3,4,2 };
	int c3[4] = { 6,3,6,3 };
	int* c[4] = { c0,c1,c2,c3 };

	Data* data = new Data();
	data->n = 4;
	data->vertices = new Vertice[4];
	for (int i = 0; i < data->n; i++) {
		Vertice& vertice = data->vertices[i];
		vertice.m = 4;
		vertice.edges = new Edge[4];
		for (int j = 0; j < data->n; j++) {
			vertice.edges[j].cost = c[i][j];
			vertice.edges[j].id = 4 * i + j;
		}
	}
	return data;
}