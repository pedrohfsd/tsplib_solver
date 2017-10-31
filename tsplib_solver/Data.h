#ifndef Data_H_
#define Data_H_

#include <vector>

#include "Vertex.h"

class Data {
public:
	std::vector<Vertex> vertices;
	Data();
	Data(int n);
	void init(int n);
	void findConnectedComponent(const std::vector<std::vector<double>>& edgeValues, std::vector<int>& connectComponent);
	void findConnectedComponent(int root, const std::vector<std::vector<double>>& edgeValues, std::vector<int>& connectComponent);
	double findMinCut(int source, int sink, const std::vector<std::vector<double>>& capacities, std::vector<int>& minCut);
	void findCutSet(std::vector<int>& cut, std::vector<Edge>& cutset);
	double maxFlow(int source, int sink, const std::vector<std::vector<double>>& edgeCapacities, std::vector<std::vector<double>>& flow, std::vector<std::vector<double>>& residual);

	static int edgeIdToRow(int id, int rows);
	static int edgeIdToColumn(int id, int rows);
	static void edgeIdToIndex(int id, int rows, int(&index)[2]);
	static int indexToId(int, int i, int j);
	static void toMatrix(const std::vector<double>& edgeValues, int expectedRows, std::vector<std::vector<double>>& valueMatrix);

private:
	void Data::findAugmentingPath(int source, int sink, std::vector<int>& augmentingPath, const std::vector<std::vector<double>>& edgeCapacities, const std::vector<std::vector<double>>& flow, const std::vector<std::vector<double>>& residual);
	double augment(std::vector<int>& augmentingPath, const std::vector<std::vector<double>>& capacities, std::vector<std::vector<double>>& flow, std::vector<std::vector<double>>& residual);
};

#endif  // Data_H_