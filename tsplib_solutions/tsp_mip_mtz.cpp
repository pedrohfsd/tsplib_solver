/*
Solution for the TSP using the MTZ (Miller-Tucker-Zemlin, n^2 squared constraints):
2<=ui<=n, i!=1
ui-uj+1 <= (n-1)(1-xij) , i!=1, j!=1
*/

#include "tsp_mip_mtz.h"

using namespace std;

void TSP_MIP_MTZ::run(Data& data) {
	IloEnv   env;
	IloModel model(env, PROBLEM.c_str());
	IloNumVarArray var(env);
	IloRangeArray con(env);

	addDecisionVariables(model, var, con, data);
	addObjectiveFunction(model, var, con, data);
	addDegreeConstraints(model, var, con, data);
	addSubtourEliminationConstraint(model, var, con, data);

	model.add(con);
	IloCplex cplex(model);
	cplex.exportModel((PROBLEM + ".lp").c_str());
	cplex.setParam(IloCplex::Threads, 1);

	if (!cplex.solve()) {
		env.error() << "Failed to optimize LP" << endl;
		throw(-1);
	}

	print(cplex, var, con);
}

void TSP_MIP_MTZ::addDecisionVariables(IloModel model, IloNumVarArray var, IloRangeArray con, Data& data) {
	IloEnv env = model.getEnv();
	for (int i = 0; i < data.n; i++) {
		for (int j = 0; j < data.n; j++) {
			stringstream name; name << "x_" << i << "_" << j;
			var.add(IloBoolVar(env, name.str().c_str()));
		}
	}
}

void TSP_MIP_MTZ::addObjectiveFunction(IloModel model, IloNumVarArray var, IloRangeArray con, Data& data) {
	IloEnv env = model.getEnv();
	IloExpr expr(env);
	for (int i = 0; i < data.n; i++) {
		for (int j = 0; j < data.n; j++) {
			Edge edge = data.vertices[i].edges[j];
			expr += edge.cost * var[edge.id];
		}
	}
	model.add(IloMinimize(env, expr));
	expr.end();
}

void TSP_MIP_MTZ::addDegreeConstraints(IloModel model, IloNumVarArray var, IloRangeArray con, Data& data) {
	IloEnv env = model.getEnv();
	for (int i = 0; i < data.n; i++) {
		IloExpr expr(env);
		for (int j = 0; j < data.n; j++) 
			expr += var[data.vertices[i].edges[j].id];
		con.add(IloRange(expr == 1));
		expr.end();

		expr = IloExpr(env);
		for (int j = 0; j < data.n; j++)
			expr += var[data.vertices[j].edges[i].id];
		con.add(IloRange(expr == 1));
		expr.end();
	}
}

void TSP_MIP_MTZ::addSubtourEliminationConstraint(IloModel model, IloNumVarArray var, IloRangeArray con, Data& data)	{
	IloEnv env = model.getEnv();
	IloNumVarArray u(env, data.n);
	for (int v = 1; v < data.n; v++) {
		stringstream name;
		name << "u_" << v;
		var.add(u[v] = IloIntVar(env, 2, data.n, name.str().c_str()));
	}
	for (int i = 1; i < data.n; i++) {
		for (int j = 1; j < data.n; j++) {
			IloExpr expr(env);
			expr += u[i]- u[j] + 1;
			expr -= (data.n - 1) * (1 - var[data.vertices[i].edges[j].id]);
			con.add(IloRange(expr <= 0));
			expr.end();
		}
	}
}

void TSP_MIP_MTZ::print(IloCplex cplex, IloNumVarArray var, IloRangeArray con) {
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
}
