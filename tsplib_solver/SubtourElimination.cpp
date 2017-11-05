#include "SubtourElimination.h"
#include <bitset>
#include <vector>
#include <set>

#define EPS 1e-5
using namespace std;

IloRange SubtourElimination::createInsideConstraint(const IloEnv &env, const IloNumVarArray &vars, const IloNumArray &vals, Data &data) {
	vector<int> subtour;
	findSubtour(env, vars, vals, data, subtour);
	return createInsideConstraint(env, vars, vals, data, subtour);
}

IloRange SubtourElimination::createInsideConstraint(const IloEnv &env, const IloNumVarArray &vars, const IloNumArray &vals, Data &data, vector<int> &subtour) {
	IloRangeArray cons(env);
	IloExpr expr(env);
	int n = (int)data.vertices.size();

	if (subtour.size() == n) return NULL;
	for (int i = 0; i < subtour.size(); i++) {
		for (int j = i + 1; j < subtour.size(); j++) {
			expr += vars[data.vertices[subtour[i]].edges[subtour[j]].id];
			expr += vars[data.vertices[subtour[j]].edges[subtour[i]].id];
		}
	}

	IloRange range(expr <= (int)subtour.size() - 1);
	expr.end();
	return range;
}

IloRange SubtourElimination::createCrossingConstraint(const IloEnv &env, const IloNumVarArray &vars, const IloNumArray &vals, Data &data) {
	vector<int> subtour;
	findSubtour(env, vars, vals, data, subtour);
	return createCrossingConstraint(env, vars, vals, data, subtour);
}

IloRange SubtourElimination::createCrossingConstraint(const IloEnv &env, const IloNumVarArray &vars, const IloNumArray &vals, Data &data, vector<int> &s) {
	IloRangeArray cons(env);
	IloExpr expr(env);
	int n = (int)data.vertices.size();

	if (s.size() == data.vertices.size()) return NULL;

	vector<int> t;
	for (int i = 0; i < n; i++) {
		if (find(s.begin(), s.end(), i) == s.end()) t.push_back(i);
	}

	for (int i = 0; i < s.size(); i++) {
		for (int j = 0; j < t.size(); j++) {
			expr += vars[data.vertices[s[i]].edges[t[j]].id];
			expr += vars[data.vertices[t[j]].edges[s[i]].id];
		}
	}
	IloRange range(expr >= 1);
	expr.end();
	return range;
}

IloRangeArray SubtourElimination::createFlowConstraint(const IloEnv &env, const IloNumVarArray &vars, const IloNumArray &vals, Data &data, int maxLimit, bool skipRedundants) {
	int n = (int)data.vertices.size();
	vector<double> edges(vars.getSize());
	vector<vector<double>> costMatrix(n);
	for (int i = 0; i < vars.getSize(); i++) edges[i] = vals[i];
	for (int i = 0; i < n; i++) costMatrix[i].resize(n);
	Data::toMatrix(edges, n, costMatrix);

	vector<bool> outsideCut(n, false);
	IloRangeArray cons(env);
	for (int i = 1; i < n; i++) {
		if (skipRedundants && outsideCut[i]) continue;
		vector<int> minCut;
		double maxFlow = data.findMinCut(0, i, costMatrix, minCut);
		if (maxFlow - 1 >= -EPS) continue; // if maxFLow >= 1

		vector<bool> insideCut(n, false);
		for (int j = 0; j < minCut.size(); j++) insideCut[minCut[j]] = true;
		for (int j = 0; j < minCut.size(); j++) if (!insideCut[j]) outsideCut[j] = true;
		cons.add(createInsideConstraint(env, vars, vals, data, minCut));
		if (cons.getSize() >= maxLimit) break;
	}
	return cons;
}

void SubtourElimination::findSubtour(const IloEnv &env, const IloNumVarArray &vars, const IloNumArray &vals, Data &data, vector<int> &connectedComponent) {
	IloRangeArray cons(env);
	IloExpr expr(env);
	int n = (int)data.vertices.size();

	vector<double> edges(vars.getSize());
	vector<vector<double>> valueMatrix(n);
	for (int i = 0; i < vars.getSize(); i++) edges[i] = vals[i];
	for (int i = 0; i < n; i++) valueMatrix[i].resize(n);
	Data::toMatrix(edges, n, valueMatrix);

	data.findConnectedComponent(valueMatrix, connectedComponent);
}