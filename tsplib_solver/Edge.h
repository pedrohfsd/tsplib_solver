#ifndef Edge_H_
#define Edge_H_

class Edge {
public:
	int id;
	int cost;
	Edge();
	Edge(int id, int cost);
private:
	void init(int id, int cost);
};

#endif  // Edge_H_
