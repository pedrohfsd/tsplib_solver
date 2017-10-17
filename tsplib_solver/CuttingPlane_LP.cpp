#include <ilcplex/ilocplex.h>
#include <vector>
#include <stack>
#include <cmath>

#include "Data.h"
#include "Edge.h"
#include "CuttingPlane_LP.h"
#include "Vertex.h"

#define EPS 1e-5

using namespace std;

CuttingPlane_LP::CuttingPlane_LP() {
};

void CuttingPlane_LP::run(Data& data) {
	IloEnv   env;
	IloModel model(env, PROBLEM.c_str());
	IloNumVarArray vars(env);
	size_t n = data.vertices.size();

	addDecisionVariables(model, vars, data);
	addObjectiveFunction(model, vars, data);
	addDegreeConstraints(model, vars, data);

	IloCplex cplex(model);
	cplex.setParam(IloCplex::Threads, 1);
	cplex.setOut(env.getNullStream());
	cplex.setWarning(env.getNullStream());
	long current = 0;
	double limit = 100000;
	while (current++ < limit) {
		cplex.exportModel((PROBLEM + ".lp").c_str());
		if (!cplex.solve())	break;
		cout << "Obj = " << cplex.getObjValue() << endl;
		if (!addSubtourConnectionConstraint(cplex, vars, data)) break;
		//if (!addSubtourEliminationConstraint(cplex, vars, data)) break;
		cout << current << " cut(s) added in total" << endl;
	}
	if (current == limit) throw("Number of constraints exceeded 2^n");

	print(cplex, vars);
};

void CuttingPlane_LP::addDecisionVariables(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	size_t n = data.vertices.size();
	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < n; j++) {
			stringstream name; name << "x_" << i << "_" << j;
			vars.add(IloNumVar(env, 0.0, 1.0, name.str().c_str()));
			if (i == j) vars[vars.getSize() - 1].setBounds(0, 0);
		}
	}
};

void CuttingPlane_LP::addDegreeConstraints(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	IloRangeArray cons(env);

	size_t n = data.vertices.size();
	for (size_t i = 0; i < n; i++) {
		IloExpr expr(env);
		for (size_t j = 0; j < n; j++)
			expr += vars[data.vertices[i].edges[j].id];
		cons.add(IloRange(expr == 1));
		expr.end();

		expr = IloExpr(env);
		for (int j = 0; j < n; j++)
			expr += vars[data.vertices[j].edges[i].id];
		cons.add(IloRange(expr == 1));
		expr.end();
	}
	model.add(cons);
};

void CuttingPlane_LP::addObjectiveFunction(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	IloExpr expr(env);
	size_t n = data.vertices.size();
	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < n; j++) {
			Edge edge = data.vertices[i].edges[j];
			expr += edge.cost * vars[edge.id];
		}
	}
	model.add(IloMinimize(env, expr));
	expr.end();
};

bool CuttingPlane_LP::addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray vars, Data& data) {
	size_t n = data.vertices.size();
	vector<double> edges;
	vector<vector<double>> costMatrix;
	for (int i = 0; i < vars.getSize(); i++) edges.push_back(cplex.getValue(vars[i]));
	for (int i = 0; i < vars.getSize(); i++) costMatrix.push_back(vector<double>(n));
	Data::toMatrix(edges, n, costMatrix);

	for (size_t j = 1; j < n; j++) {
		vector<int> s;
		vector<int> t;
		double maxFlow = data.findMinCut(0, j, costMatrix, s);
		if (maxFlow - 1 > -EPS) continue;
		addSubtourConstraints(cplex, vars, s, t, data);
		return true;
	}
	return false;
};

void CuttingPlane_LP::addSubtourConstraints(IloCplex cplex, IloNumVarArray vars, const vector<int>& s, const vector<int>& t, Data& data) {
	IloModel model = cplex.getModel();
	IloRangeArray cons(cplex.getEnv());
	int n = data.vertices.size();

	/*IloExpr expr(cplex.getEnv());
	for (int i = 0; i < s.size(); i++) {
		for (int j = 0; j < t.size(); j++) {
			expr += vars[data.vertices[s[i]].edges[t[j]].id];
			expr += vars[data.vertices[t[j]].edges[s[i]].id];
		}
	}
	cons.add(IloRange(expr >= 2));
	expr.end();
	model.add(cons);*/

	IloExpr expr(cplex.getEnv());
	for (int i = 0; i < s.size(); i++) {
		for (int j = i; j < s.size(); j++) {
			expr += vars[data.vertices[s[i]].edges[s[j]].id];
			if (i != j)
				expr += vars[data.vertices[s[j]].edges[s[i]].id];
		}
	}
	cons.add(IloRange(expr <= (int)s.size() - 1));
	expr.end();
	model.add(cons);
};

void CuttingPlane_LP::print(IloCplex cplex, IloNumVarArray vars) {
	IloEnv env = cplex.getEnv();
	IloNumArray vals(env);
	env.out() << "Solution status = " << cplex.getStatus() << endl;
	env.out() << "Solution value  = " << cplex.getObjValue() << endl;
	cplex.getValues(vals, vars);
	env.out() << "Values        = " << vals << endl;
	/*cplex.getSlacks(vals, con);
	env.out() << "Slacks        = " << vals << endl;*/
	/*cplex.getDuals(vals, con);
	env.out() << "Duals         = " << vals << endl;
	cplex.getReducedCosts(vals, var);
	env.out() << "Reduced Costs = " << vals << endl;*/
};