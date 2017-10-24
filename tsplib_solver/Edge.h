#ifndef EDGE_H_
#define EDGE_H_

class Edge {
public:
	int id;
	int cost;
	Edge();
	Edge(int id, int cost);
private:
	void init(int id, int cost);
};

#endif  // EDGE_H_
