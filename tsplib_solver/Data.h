#ifndef Data_H_
#define Data_H_

class Vertex;

class Data {
public:
	std::vector<Vertex> vertices;
	Data();
	Data(int);
	void init(int, int);
};

#endif  // Data_H_