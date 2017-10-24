#ifndef VERTEX_H_
#define VERTEX_H_

#include "Edge.h"

class Vertex {
public:
	std::vector<Edge> edges;
	Vertex();
	Vertex(int m);
	void init(int m);
};

#endif  // VERTEX_H_