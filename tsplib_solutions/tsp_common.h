#ifndef TSP_COMMON_H_
#define TSP_COMMON_H_

#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <sstream>
#include <algorithm>
#include <unordered_map>
ILOSTLBEGIN


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

Data* read(const string& filepath);

#endif  // TSP_COMMON_H_