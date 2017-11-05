#include "TSP.h"
#include <string>

using namespace std;

TSP::TSP(Data& data, string formulationName, bool singleThreaded) :
cons(IloRangeArray(env))
, data(data)
, formulationName(formulationName)
, model(IloModel(env))
, singleThreaded(singleThreaded)
, vars(IloNumVarArray(env)) {}


void TSP::addDecisionVariables(bool useIntegerTypes) {
	size_t n = data.vertices.size();
	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < n; j++) {
			if(useIntegerTypes) vars.add(IloBoolVar(env, ("x_" + to_string(i) + "_" + to_string(j)).c_str()));
			else vars.add(IloNumVar(env, 0, 1, ("x_" + to_string(i) + "_" + to_string(j)).c_str()));
		}
	}
}

void TSP::addIncomingEdgeConstraints() {
	size_t n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		IloExpr expr(env);
		for (int j = 0; j < n; j++) {
			expr += vars[data.vertices[j].edges[i].id];
		}
		cons.add(IloRange(expr == 1));
		expr.end();
	}
}

void TSP::addObjectiveFunction() {
	IloExpr expr(env);
	size_t n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			Edge edge = data.vertices[i].edges[j];
			expr += edge.cost * vars[edge.id];
		}
	}
	model.add(IloMinimize(env, expr));
	expr.end();
}

void TSP::addOutgoingEdgeConstraints() {
	size_t n = data.vertices.size();
	for (int i = 0; i < n; i++) {
		IloExpr expr(env);
		for (int j = 0; j < n; j++) {
			expr += vars[data.vertices[i].edges[j].id];
		}
		cons.add(IloRange(expr == 1));
		expr.end();
	}
}

void TSP::addSelfEdgeConstraints() {
	int n = (int)data.vertices.size();
	for (int i = 0; i < n; i++) {
		vars[Data::indexToId(n, i, i)].setBounds(0, 0);
	}
}

IloNumArray TSP::getFeasibleTour() {
	int n = (int)data.vertices.size();
	IloNumArray vals(env, vars.getSize());
	for (int i = 1; i < n; i++) {
		vals[data.indexToId(n, i - 1, i)] = 1;
	}
	vals[data.indexToId(n, n - 1, 0)] = IloNum(1);
	return vals;
}

void TSP::print(IloCplex cplex) {
	IloNumArray vals(env);

	env.out() << "Solution status = " << cplex.getStatus() << endl;
	env.out() << "Solution value  = " << cplex.getObjValue() << endl;
	cplex.getValues(vals, vars);
	env.out() << "Values        = " << vals << endl;
	cplex.getSlacks(vals, cons);
	env.out() << "Slacks        = " << vals << endl;
	/*cplex.getDuals(vals, con);
	env.out() << "Duals         = " << vals << endl;
	cplex.getReducedCosts(vals, var);
	env.out() << "Reduced Costs = " << vals << endl;*/
}