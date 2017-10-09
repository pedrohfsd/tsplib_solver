#include "tsp_common.h"
#include <fstream>

Data* buildData(double** nodes, int n, int m);
unordered_map<string, string>* readSpecification(ifstream& infile);
double** readData_EUC_2D(ifstream& infile, const int dimension);
int euclidian_distance_2d(double* node1, double* node2);
Data* createTestData();

Data* read(const string& filepath){
	ifstream infile(filepath);
	unordered_map<string, string>* spec = readSpecification(infile);
	int n = stoi((*spec)["DIMENSION"]);
	double** nodes = readData_EUC_2D(infile, n);
	Data* data = buildData(nodes, n, n*n);
	return data;
};

Data* buildData(double** nodes, int n, int m){
	Data* data = new Data();
	data->n = n;
	data->vertices = new Vertice[n];
	for (int i = 0; i < n; i++)	{
		Vertice& vertice = data->vertices[i];
		vertice.m = n;
		vertice.edges = new Edge[n];
		for (int j = 0; j < n; j++)	{
			vertice.edges[j].cost = euclidian_distance_2d(nodes[i], nodes[j]);
			vertice.edges[j].id = n * i + j;
		}
	}
	return data;
}

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

double** readData_EUC_2D(ifstream& infile, const int dimension){
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

int euclidian_distance_2d(double* node1, double* node2){
	double x[2] = { node1[0], node2[0] };
	double y[2] = { node1[1], node2[1] };
	double xd = x[0] - x[1];
	double yd = y[0] - y[1];
	return (int)(sqrt(xd*xd + yd*yd)+0.5);
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