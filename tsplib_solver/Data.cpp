#include "Data.h"

#include <limits>
#include <queue>
#include <set>
#include <stack>

#define EPS 1e-5

using namespace std;

Data::Data() {
	init(0);
};

Data::Data(int n) {
	init(n);
};

void Data::init(int n) {
	vertices.clear();
	if (n > 0) {
		for (int i = 0; i < n; i++) vertices.push_back(Vertex());
		for (int i = 0; i < n; i++) vertices[i].init(n);
	}
};

void Data::findConnectedComponent(const vector<vector<double>>& edgeValues, vector<int>& connectedComponent) {
	findConnectedComponent(0, edgeValues, connectedComponent);
};

void Data::findConnectedComponent(int root, const vector<vector<double>>& edgeValues, vector<int>& connectedComponent) {
	queue<int> q;
	int n = (int)vertices.size();
	vector<bool> visited(n, false);

	q.push(root);
	visited[root] = true;
	while (!q.empty()) {
		int node = q.front();
		q.pop();
		connectedComponent.push_back(node);
		for (int i = 0; i < n; i++) {
			if (!visited[i] && fabs(edgeValues[node][i]) > EPS) {
				q.push(i);
				visited[i] = true;
			}
		}
	}
};

void Data::findCutSet(vector<int>& cut, vector<Edge>& cutset) {
	set<int> s(cut.begin(), cut.end());
	for(int& v : cut) {
		for (int i = 0; i < vertices.size(); i++) {
			Edge& e = vertices[v].edges[i];
			if (s.find(i) == s.end() && e.cost != 0) cutset.push_back(e);
		}
	}
}

double Data::findMinCut(int source, int sink, const vector<vector<double>>& capacities, vector<int>& minCut) {
	size_t n = vertices.size();
	vector<vector<double>> flow(n);
	vector<vector<double>> residual(n);
	for (size_t i = 0; i < n; i++){
		flow[i].resize(n, 0);
		residual[i].resize(n, 0);
	}
	double max_flow = maxFlow(source, sink, capacities, flow, residual);

	vector<vector<double>> remainingFlow(n);
	for (size_t i = 0; i < n; i++) {
		remainingFlow[i].resize(n, 0);
		for (size_t j = 0; j < n; j++) {
			remainingFlow[i][j] = (capacities[i][j] - flow[i][j]) + residual[i][j];
		}
	}
	findConnectedComponent(source, remainingFlow, minCut);
	return max_flow;
};

double Data::maxFlow(int source, int sink, const vector<vector<double>>& edgeCapacities, vector<vector<double>>& flow, vector<vector<double>>& residual){
	vector<int> augmentingPath;
	findAugmentingPath(source, sink, augmentingPath, edgeCapacities, flow, residual);
	double max = 0;
	while (!augmentingPath.empty()) {
		max += augment(augmentingPath, edgeCapacities, flow, residual);
		augmentingPath.clear();
		findAugmentingPath(source, sink, augmentingPath, edgeCapacities, flow, residual);
	}
	return max;
};

void Data::findAugmentingPath(int source, int sink, vector<int>& augmentingPath, const vector<vector<double>>&capacities, const vector<vector<double>>& flow, const vector<vector<double>>& residual) {
	queue<int> q;
	size_t n = (int)vertices.size();
	vector<bool> visited(n, false);
	vector<int> prev(n, -1);

	q.push(source);
	prev[source] = source;
	visited[source] = true;
	while (!q.empty()) {
		int node = q.front();
		q.pop();
		if (node == sink) break;
		for (int i = 0; i < n; i++) {
			if (visited[i]) continue;
			if ((fabs(flow[node][i] - capacities[node][i]) <= EPS) && (fabs(residual[node][i]) <= EPS)) continue;
			q.push(i);
			prev[i] = node;
			visited[i] = true;
		}
	}

	if (prev[sink] == -1) return;
	int current = sink;
	augmentingPath.push_back(sink);
	while (prev[current] != current) {
		current = prev[current];
		augmentingPath.push_back(current);
	}
	reverse(augmentingPath.begin(), augmentingPath.end());
};

double Data::augment(vector<int>& augmentingPath, const vector<vector<double>>&capacities, vector<vector<double>>& flow, vector<vector<double>>& residual) {
	int n = (int)vertices.size();
	if (augmentingPath.size() < 2) throw(exception("Can't augment path from node to itself"));
	double min = numeric_limits<double>::max();
	for (size_t i = 1; i < augmentingPath.size(); i++) {
		int from = augmentingPath[i-1];
		int to = augmentingPath[i];
		double f = flow[from][to] - capacities[from][to] < -EPS ? capacities[from][to] - flow[from][to] : residual[from][to];
		if (f < min) min = f;
	}

	for (size_t i = 1; i < augmentingPath.size(); i++) {
		int from = augmentingPath[i-1];
		int to = augmentingPath[i];
		int edgeid = Data::indexToId(n, augmentingPath[i - 1], augmentingPath[i]);
		if (flow[from][to] - capacities[from][to] < -EPS) {
			flow[from][to] += min;
			residual[to][from] += min;
		} else residual[from][to] -= min;
	}
	return min;
};

void Data::toMatrix(const vector<double>& edgeValues, int expectedRows, vector<vector<double>>& valueMatrix) {
	for (int i = 0; i < edgeValues.size(); i++) valueMatrix[Data::edgeIdToRow(i, expectedRows)][Data::edgeIdToColumn(i, expectedRows)] = edgeValues[i];
};

int Data::edgeIdToRow(int id, int rows) {
	return id / rows;
};

int Data::edgeIdToColumn(int id, int rows) {
	return id % rows;
};

void Data::edgeIdToIndex(int id, int rows, int(&index)[2]) {
	index[0] = id / rows;
	index[1] = id % rows;
};

int Data::indexToId(int rows, int i, int j) {
	return rows*i + j;
};