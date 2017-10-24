/*
Solution for the TSP using the MTZ (Miller-Tucker-Zemlin, n^2 squared constraints):
2<=ui<=n, i!=1
ui-uj+1 <= (n-1)(1-xij) , i!=1, j!=1

This formulation removes subtours by numbering each node and imposing a sequential order with the given restrictions:
- there's a node u where number[u] = 1
- for every edge Xij=(u,v) if Xij is part of the optimal solution (has value 1 in the objective function) then number[v]= number[u]-1
- for every node u, number[u] is distinct
If we can number all the nodes from 1 to n in the above way, then this must be a feasible tour.
*/

#include "MTZ.h"

#include <vector>

using namespace std;

MTZ::MTZ() {
};

void MTZ::run(Data& data) {
	IloEnv   env;
	IloModel model(env, PROBLEM.c_str());
	IloNumVarArray vars(env);
	IloRangeArray cons(env);

	addDecisionVariables(model, vars, cons, data);
	addObjectiveFunction(model, vars, cons, data);
	addDegreeConstraints(model, vars, cons, data);
	addSubtourEliminationConstraint(model, vars, cons, data);

	model.add(cons);
	IloCplex cplex(model);
	cplex.exportModel((PROBLEM + ".lp").c_str());
	cplex.setParam(IloCplex::Threads, 1);

	if (!cplex.solve()) {
		env.error() << "Failed to optimize LP" << endl;
		throw(-1);
	}

	print(cplex, vars, cons);
};

void MTZ::addDecisionVariables(IloModel model, IloNumVarArray var, IloRangeArray con, Data& data) {
	IloEnv env = model.getEnv();
	size_t n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			stringstream name; name << "x_" << i << "_" << j;
			var.add(IloBoolVar(env, name.str().c_str()));
		}
	}
};

void MTZ::addDegreeConstraints(IloModel model, IloNumVarArray var, IloRangeArray con, Data& data) {
	IloEnv env = model.getEnv();
	size_t n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		IloExpr expr(env);
		for (int j = 0; j < n; j++) 
			expr += var[data.vertices[i].edges[j].id];
		con.add(IloRange(expr == 1));
		expr.end();

		expr = IloExpr(env);
		for (int j = 0; j < n; j++)
			expr += var[data.vertices[j].edges[i].id];
		con.add(IloRange(expr == 1));
		expr.end();
	}
};

void MTZ::addObjectiveFunction(IloModel model, IloNumVarArray var, IloRangeArray con, Data& data) {
	IloEnv env = model.getEnv();
	IloExpr expr(env);
	size_t n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			Edge edge = data.vertices[i].edges[j];
			expr += edge.cost * var[edge.id];
		}
	}
	model.add(IloMinimize(env, expr));
	expr.end();
};

void MTZ::addSubtourEliminationConstraint(IloModel model, IloNumVarArray var, IloRangeArray con, Data& data)	{
	IloEnv env = model.getEnv();
	int n = (int)data.vertices.size();
	IloNumVarArray u(env, n);
	for (int v = 1; v < n; v++) {
		stringstream name;
		name << "u_" << v;
		var.add(u[v] = IloIntVar(env, 2, n, name.str().c_str()));
	}
	for (int i = 1; i < n; i++) {
		for (int j = 1; j < n; j++) {
			IloExpr expr(env);
			expr += u[i]- u[j] + 1;
			expr -= (n - 1) * (1 - var[data.vertices[i].edges[j].id]);
			con.add(IloRange(expr <= 0));
			expr.end();
		}
	}
};

void MTZ::print(IloCplex cplex, IloNumVarArray var, IloRangeArray con) {
	IloEnv env = cplex.getEnv();
	IloNumArray vals(env);
	env.out() << "Solution status = " << cplex.getStatus() << endl;
	env.out() << "Solution value  = " << cplex.getObjValue() << endl;
	cplex.getValues(vals, var);
	env.out() << "Values        = " << vals << endl;
	cplex.getSlacks(vals, con);
	env.out() << "Slacks        = " << vals << endl;
	/*cplex.getDuals(vals, con);
	env.out() << "Duals         = " << vals << endl;
	cplex.getReducedCosts(vals, var);
	env.out() << "Reduced Costs = " << vals << endl;*/
};