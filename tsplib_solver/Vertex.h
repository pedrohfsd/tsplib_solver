#ifndef Vertex_H_
#define Vertex_H_

class Edge;

class Vertex {
public:
	std::vector<Edge> edges;
	Vertex();
	Vertex(int);
	void init(int);
};

#endif  // Vertex_H_