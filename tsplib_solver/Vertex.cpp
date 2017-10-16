#include <memory>
#include <vector>

#include "Edge.h"
#include "Vertex.h"

using namespace std;

Vertex::Vertex() {
	init(0);
};

Vertex::Vertex(int m) {
	init(m);
};

void Vertex::init(int m) {
	edges.clear();
	if (m > 0) {
		for (int i = 0; i < m; i++) edges.push_back(Edge());
	}
};