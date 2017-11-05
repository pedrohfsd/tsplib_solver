/*
Solution for the TSP using the MTZ (Miller-Tucker-Zemlin, n^2 squared constraints) formulation:

//TSP Formulation:
Min sum(Cij*Xij) | i{1,...,n} and j={1,...,n} --> choose edges atempting to minimize total cost
sum(Xij) = 1 | j={1,...,n}					  --> every node uses exactly 1 outgoing edge
sum(Xij) = 1 | 1={1,...,n}					  --> every node uses exactly 1 incoming edge
Xii = 0 | i={1,...,n}						  --> use an edge to itself is not allowed
Xij = {0,1} | i{1,...,n} and j={1,...,n}	  --> we want an integer result (every edge is used or not) 
// Additional MTZ constraints
2<=ui<=n | i!=1								  --> every node is associated to a number ranging from 2 to n
ui-uj+1 <= (n-1)(1-xij) | i!=1, j!=1		  --> this association must be sequential, ordered
u0 = 1 --> 

This formulation removes subtours by numbering each node and imposing a sequential order with the given restrictions:
- there's a node u where number[u] = 1
- for every edge Xij=(u,v) if Xij is part of the optimal solution (has value 1 in the objective function) then number[v]= number[u]-1
- for every node u, number[u] is distinct
If we can number all the nodes from 1 to n in the above way, then this must be a feasible tour.
*/

#include "MTZ.h"

#include <vector>

using namespace std;

MTZ::MTZ(Data& data, bool singleThreaded):TSP(data, "MTZ", singleThreaded)
, u(IloNumVarArray(env, (int)data.vertices.size())) {}

void MTZ::solve() {

	addDecisionVariables(true);
	addObjectiveFunction();
	addIncomingEdgeConstraints();
	addOutgoingEdgeConstraints();
	addSelfEdgeConstraints();
	
	addSubtourEliminationVariables();
	addSubtourEliminationConstraints();

	model.add(cons);
	IloCplex cplex(model);
	cplex.exportModel((formulationName + ".lp").c_str());
	if (singleThreaded) cplex.setParam(IloCplex::Threads, 1);

	if (!cplex.solve()) {
		env.error() << "Failed to optimize LP" << endl;
		throw(-1);
	}

	print(cplex);
}

void MTZ::addSubtourEliminationConstraints()	{
	int n = (int)data.vertices.size();
	for (int i = 1; i < n; i++) {
		for (int j = 1; j < n; j++) {
			IloExpr expr(env);
			expr += u[i]- u[j] + 1;
			expr -= (n - 1) * (1 - vars[data.vertices[i].edges[j].id]);
			cons.add(IloRange(expr <= 0));
			expr.end();
		}
	}
}

void MTZ::addSubtourEliminationVariables() {
	int n = (int)data.vertices.size();
	for (int v = 1; v < n; v++) {
		vars.add(u[v] = IloIntVar(env, 2, n, ("u_"+to_string(v)).c_str()));
	}
}