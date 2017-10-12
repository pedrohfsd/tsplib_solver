#ifndef FileParser_H_
#define FileParser_H_

class Data;

//typedef function<int(FileReader&, double[], double[])> dist distance_function;

class FileParser {

public:
	void createTestData(Data&);
	void read(const std::string&, Data&);
	FileParser();

private:
	int attDistance(double[2], double[2]);
	void buildDataFromNodes(std::vector<std::vector<double>>&, std::function<int(double[], double[])>, Data&);
	void buildDataFromEdges(std::vector<std::vector<double>>&, Data&);
	int euclidianDistance2D(double[2], double[2]);
	int geoDistance(double[2], double[2]);
	int nint(double);
	void readLowDiagEdges2D(std::ifstream&, std::vector<std::vector<double>>&);
	void readNodes2D(std::ifstream&, std::vector<std::vector<double>>&);
	void readSpecification(std::ifstream&, std::unordered_map<std::string, std::string>&);
};

#endif  // FileParser_H_
