#ifndef TSP_H_
#define TSP_H_

#include <ilcplex/ilocplex.h>
#include "Data.h"

class TSP{
public:
	TSP(Data& data, std::string formulationName, bool singleThreaded);
	virtual void solve() = 0;

protected:
	std::string formulationName;
	IloEnv env;
	IloRangeArray cons;
	Data& data;
	IloModel model;
	bool singleThreaded;
	IloNumVarArray vars;
	void addDecisionVariables(bool useIntegerTypes);
	void addIncomingEdgeConstraints();
	void addObjectiveFunction();
	void addOutgoingEdgeConstraints();
	void addSelfEdgeConstraints();
	IloNumArray getFeasibleTour();
	void print(IloCplex cplex);
};

#endif //TSP_H_