#include <ilcplex/ilocplex.h>
#include <vector>
#include <stack>
#include <cmath>

#include "Data.h"
#include "Edge.h"
#include "CuttingPlane_MIP.h"
#include "Vertex.h"

using namespace std;

CuttingPlane_MIP::CuttingPlane_MIP() {
};

void CuttingPlane_MIP::run(Data& data, bool option1) {
	IloEnv   env;
	IloModel model(env, PROBLEM.c_str());
	IloNumVarArray vars(env);
	int n = data.vertices.size();

	addDecisionVariables(model, vars, data);
	addObjectiveFunction(model, vars, data);
	addDegreeConstraints(model, vars, data);

	IloCplex cplex(env);
	cplex.setParam(IloCplex::Threads, 1);
	cplex.setOut(env.getNullStream());
	long current = 0;
	long limit = pow(2, n);
	while (current++ < limit) {
		cplex.extract(model);
		if (!cplex.solve())	break;
		if (option1 && !addSubtourConnectionConstraint(cplex, vars, data)) break;
		if (!option1 && !addSubtourEliminationConstraint(cplex, vars, data)) break;
		cout << current << " cut(s) added in total" << endl;
	}
	if(current == limit) throw("Number of constraints exceeded 2^n");
	cplex.exportModel((PROBLEM + ".lp").c_str());

	print(cplex, vars);
};

void CuttingPlane_MIP::addDegreeConstraints(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	IloRangeArray cons(env);

	int n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		IloExpr expr(env);
		for (int j = 0; j < n; j++)
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

void CuttingPlane_MIP::addDecisionVariables(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	int n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			stringstream name; name << "x_" << i << "_" << j;
			vars.add(IloBoolVar(env, name.str().c_str()));
		}
	}
};

void CuttingPlane_MIP::addObjectiveFunction(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	IloExpr expr(env);
	int n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			Edge edge = data.vertices[i].edges[j];
			expr += edge.cost * vars[edge.id];
		}
	}
	model.add(IloMinimize(env, expr));
	expr.end();
};

bool CuttingPlane_MIP::addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray vars, Data& data) {
	IloModel model = cplex.getModel();
	IloRangeArray cons(cplex.getEnv());
	IloExpr expr(cplex.getEnv());
	int n = data.vertices.size();

	vector<double> edges;
	vector<vector<double>> valueMatrix;
	for (int i = 0; i < vars.getSize(); i++) edges.push_back(cplex.getValue(vars[i]));
	for (int i = 0; i < vars.getSize(); i++) valueMatrix.push_back(vector<double>(n));
	Data::toMatrix(edges, n, valueMatrix);

	vector<int> s;
	data.findConnectedComponent(valueMatrix, s);
	if (s.size() == data.vertices.size()) return false;

	vector<int> t;
	for (int i = 0; i < n; i++) if(find(s.begin(), s.end(), i) == s.end()) t.push_back(i);

	for (int i = 0; i < s.size(); i++) {
		for (int j = 0; j < t.size(); j++) {
			expr += vars[data.vertices[s[i]].edges[t[j]].id];
			if (i != j) // works without this if on MIP but not in LP. Left it only for didatic purposes
				expr += vars[data.vertices[t[j]].edges[s[i]].id];
		}
	}
	cons.add(IloRange(expr >= 1));
	expr.end();
	model.add(cons);
	return true;
};

bool CuttingPlane_MIP::addSubtourEliminationConstraint(IloCplex cplex, IloNumVarArray vars, Data& data) {
	IloModel model = cplex.getModel();
	IloRangeArray cons(cplex.getEnv());
	IloExpr expr(cplex.getEnv());
	int n = data.vertices.size();

	vector<double> edges;
	vector<vector<double>> valueMatrix;
	for (int i = 0; i < vars.getSize(); i++) edges.push_back(cplex.getValue(vars[i]));
	for (int i = 0; i < vars.getSize(); i++) valueMatrix.push_back(vector<double>(n));
	Data::toMatrix(edges, n, valueMatrix);

	vector<int> s;
	data.findConnectedComponent(valueMatrix, s);
	if (s.size() == data.vertices.size()) return false;

	for (int i = 0; i < s.size(); i++) {
		for (int j = i; j < s.size(); j++) {
			expr += vars[data.vertices[s[i]].edges[s[j]].id];
			if( i!= j) // works without this if on MIP but not in LP. Left it only for didatic purposes
				expr += vars[data.vertices[s[j]].edges[s[i]].id];
		}
	}
	cons.add(IloRange(expr <= (int)s.size()-1));
	expr.end();
	model.add(cons);
	return true;
};

void CuttingPlane_MIP::print(IloCplex cplex, IloNumVarArray vars) {
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