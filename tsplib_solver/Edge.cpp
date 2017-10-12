#include "Edge.h"

using namespace std;

Edge::Edge() {
	init(0, 0);
};

Edge::Edge(int id, int cost) {
	init(id, cost);
};

void Edge::init(int id, int cost) {
	this->id = id;
	this->cost = cost;
};