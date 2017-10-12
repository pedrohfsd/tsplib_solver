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
	if (m > 0) {
		edges = vector<Edge>(m);
	}
};