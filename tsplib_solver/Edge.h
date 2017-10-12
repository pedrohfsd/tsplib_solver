#ifndef Edge_H_
#define Edge_H_

class Edge {
public:
	int id;
	int cost;
	Edge();
	Edge(int, int);
private:
	void init(int, int);
};

#endif  // Edge_H_
