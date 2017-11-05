#include "FileParser.h"

#include <sstream>

using namespace std;

FileParser::FileParser(Data& data):data(data) {}

void FileParser::parse(const string& filepath) {
	ifstream infile(filepath);
	if (!infile.is_open()) throw exception(("Filepath " + filepath + " does not exists").c_str());
	unordered_map<string, string> spec;
	readSpecification(infile, spec);
	int n = stoi(spec["DIMENSION"]);
	data.init(n);

	function<int(double[], double[])> dist = NULL;
	if (spec["EDGE_WEIGHT_TYPE"] == "EUC_2D") dist = [this](double f[2], double t[2]) {return euclidianDistance2D(f, t); };
	else if (spec["EDGE_WEIGHT_TYPE"] == "ATT") dist = [this](double f[2], double t[2]) {return attDistance(f, t); };
	else if (spec["EDGE_WEIGHT_TYPE"] == "GEO") dist = [this](double f[2], double t[2]) {return geoDistance(f, t); };
	else if (spec["EDGE_WEIGHT_TYPE"] == "EXPLICIT") dist = NULL;
	else throw("Unsupported distance function");

	if (spec["NODE_COORD_SECTION"] == "true") {
		vector<vector<double>> nodes;
		for (int i = 0; i < data.vertices.size(); i++) nodes.push_back(vector<double>(n));
		readNodes2D(infile, nodes);
		buildDataFromNodes(nodes, dist);
	}
	else if (spec["EDGE_WEIGHT_FORMAT"] == "LOWER_DIAG_ROW") {
		vector<vector<double>> edges;
		for (int i = 0; i < data.vertices.size(); i++) edges.push_back(vector<double>(n));
		readLowDiagEdges2D(infile, edges);
		buildDataFromEdges(edges);
	}
};

void FileParser::createTestData() {
	/*int c0[4] = { 5,8,2,1 };
	int c1[4] = { 5,1,5,3 };
	int c2[4] = { 6,3,4,2 };
	int c3[4] = { 6,3,6,3 };
	int* c[4] = { c0,c1,c2,c3 };
	int n = 4;*/

	/*int c0[6] = { 1, 153, 510, 706, 966, 581 };
	int c1[6] = { 153, 1, 422, 664, 997, 598 };
	int c2[6] = { 510, 422, 1, 289, 744, 390 };
	int c3[6] = { 706, 664, 289, 1, 491, 265 };
	int c4[6] = { 966, 997, 744, 491, 1, 400 };
	int c5[6] = { 581, 598, 390, 265, 400, 1 };
	int* c[6] = { c0,c1,c2,c3,c4,c5 };
	int n = 6;*/

	//flow = 28
	int c0[8] = { 0, 10, 5, 15, 0, 0, 0, 0 };
	int c1[8] = { 0, 0, 4, 0, 9, 15, 0, 0 };
	int c2[8] = { 0, 0, 0, 4, 0, 8, 0, 0 };
	int c3[8] = { 0, 0, 0, 0, 0, 0, 30, 0 };
	int c4[8] = { 0, 0, 0, 0, 0, 15, 0, 10 };
	int c5[8] = { 0, 0, 0, 0, 0, 0, 15, 10 };
	int c6[8] = { 0, 0, 6, 0, 0, 0, 0, 10 };
	int c7[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	int* c[8] = { c0,c1,c2,c3,c4,c5,c6,c7 };
	int n = 8;

	data.init(n);
	for (int i = 0; i < n; i++) {
		Vertex& vertex = data.vertices[i];
		for (int j = 0; j < n; j++) {
			vertex.edges[j].cost = c[i][j];
			vertex.edges[j].id = Data::indexToId(n, i, j);
		}
	}

	vector<vector<double>> capacities(n);
	vector<vector<double>> flow(n);
	vector<vector<double>> residual(n);
	for (int i = 0; i < n; i++) {
		flow[i].resize(n);
		residual[i].resize(n);
		capacities[i].resize(n);
	}
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			capacities[i][j] = data.vertices[i].edges[j].cost;
	//double max = data.maxFlow(0, 7, capacities, flow, residual);

	vector<int> minCut;
	data.findMinCut(0,7,capacities,minCut);
};

int FileParser::attDistance(double node1[2], double node2[2]) {
	double x[2] = { node1[0], node2[0] };
	double y[2] = { node1[1], node2[1] };
	double xd = x[0] - x[1];
	double yd = y[0] - y[1];
	double rij = sqrt((xd*xd + yd*yd) / 10.0);
	int tij = nint(rij);
	if (tij < rij) return tij + 1;
	return tij;
};

void FileParser::buildDataFromNodes(vector<vector<double>>& nodes, function<int(double[], double[])> dist) {
	int n = (int)data.vertices.size();
	for (int i = 0; i < n; i++) {
		Vertex& vertex = data.vertices[i];
		for (int j = 0; j < n; j++) {
			double from[2] = { nodes[i][0], nodes[i][1] };
			double to[2] = { nodes[j][0], nodes[j][1] };
			vertex.edges[j].cost = dist(from, to);
			vertex.edges[j].id = Data::indexToId(n, i, j);
		}
	}
};

void FileParser::buildDataFromEdges(vector<vector<double>>& edges) {
	int n = (int)data.vertices.size();
	for (int i = 0; i < n; i++) {
		Vertex& vertex = data.vertices[i];
		for (int j = 0; j < n; j++) {
			vertex.edges[j].cost = (int)edges[i][j];
			vertex.edges[j].id = Data::indexToId(n, i, j);
		}
	}
};

int FileParser::euclidianDistance2D(double node1[2], double node2[2]) {
	double x[2] = { node1[0], node2[0] };
	double y[2] = { node1[1], node2[1] };
	double xd = x[0] - x[1];
	double yd = y[0] - y[1];
	return nint(sqrt(xd*xd + yd*yd));
};

int FileParser::geoDistance(double node1[2], double node2[2]) {
	double x[2] = { node1[0], node2[0] };
	double y[2] = { node1[1], node2[1] };
	double PI = 3.141592;
	double latitude[2];
	double longitude[2];

	int deg = (int)x[0];
	double min = x[0] - deg;
	latitude[0] = PI * (deg + 5.0 * min / 3.0) / 180.0;
	deg = (int)y[0];
	min = y[0] - deg;
	longitude[0] = PI * (deg + 5.0 * min / 3.0) / 180.0;

	deg = (int)x[1];
	min = x[1] - deg;
	latitude[1] = PI * (deg + 5.0 * min / 3.0) / 180.0;
	deg = (int)y[1];
	min = y[1] - deg;
	longitude[1] = PI * (deg + 5.0 * min / 3.0) / 180.0;

	double RRR = 6378.388;
	double q1 = cos(longitude[0] - longitude[1]);
	double q2 = cos(latitude[0] - latitude[1]);
	double q3 = cos(latitude[0] + latitude[1]);
	return (int)(RRR * acos(0.5*((1.0 + q1)*q2 - (1.0 - q1)*q3)) + 1.0);
};

int FileParser::nint(double x) {
	return (int)(x + 0.5);
};

void FileParser::readLowDiagEdges2D(ifstream& infile, vector<vector<double>>& edges) {
	int pos = 0;
	for (int i = 0; i < edges.size(); i++)
	{
		for (int j = 0; j <= pos; j++)
		{
			infile >> edges[i][j];
			edges[j][i] = edges[i][j];
		}
		pos++;
	}
};

void FileParser::readNodes2D(ifstream& infile, vector<vector<double>>& nodes) {
	string line;
	while (getline(infile, line)) {
		istringstream iss(line);
		int id; double x, y;
		if (!(iss >> id >> x >> y)) break;
		nodes[id - 1][0] = x;
		nodes[id - 1][1] = y;
	}
};

void FileParser::readSpecification(ifstream& infile, unordered_map<string, string>& spec) {
	string line;
	while (getline(infile, line)) {
		size_t pos = line.find(":");
		if (pos == string::npos) break;
		line.replace(pos, 1, " ");
		istringstream iss(line);
		string key; string value;
		if (!(iss >> key >> value)) { break; }
		spec[key] = value;
	}
	spec[line] = "true";
};