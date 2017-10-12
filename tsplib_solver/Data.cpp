#include <memory>
#include <vector>

#include "Data.h"
#include "Vertex.h"
#include "Edge.h"

using namespace std;

Data::Data() {
	init(0, 0);
};

Data::Data(int n) {
	init(n, 0);
};

void Data::init(int n, int m) {
	vertices = vector<Vertex>(n);
	if (n > 0) {
		//for (int i = 0; i < n; i++) vertices.push_back(unique_ptr<Vertex>());
		if (m > 0) {
			for (int i = 0; i < n; i++) vertices[i].init(m);
			//for each (unique_ptr<Vertex> v in vertices.data) v.get()->init(m);
		}
	}
};
