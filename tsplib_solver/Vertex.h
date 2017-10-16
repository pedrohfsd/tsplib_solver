#ifndef Vertex_H_
#define Vertex_H_

class Edge;

class Vertex {
public:
	std::vector<Edge> edges;
	Vertex();
	Vertex(int m);
	void init(int m);
};

#endif  // Vertex_H_