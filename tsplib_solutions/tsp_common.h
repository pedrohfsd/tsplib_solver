#ifndef TSP_COMMON_H_
#define TSP_COMMON_H_

#include <ilcplex/ilocplex.h>

#include <algorithm>
#include <sstream>
#include <unordered_map>

class Edge{
public:
	int cost;
	int id;
};

class Vertice{
public:
	Edge* edges;
	int m;
};

class Data{
public:
	Vertice* vertices;
	int n;
};

Data* read(const std::string& filepath);

#endif  // TSP_COMMON_H_