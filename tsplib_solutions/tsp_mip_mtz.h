#ifndef TSP_MIP_MTZ_H_
#define TSP_MIP_MTZ_H_

#include "tsp_common.h"

class TSP_MIP_MTZ {

public:
	const string PROBLEM = "TSP_MIP_MTZ";
	void run(Data&);

private:
	void addDecisionVariables(IloModel, IloNumVarArray, IloRangeArray, Data&);
	void addObjectiveFunction(IloModel, IloNumVarArray, IloRangeArray, Data&);
	void addDegreeConstraints(IloModel, IloNumVarArray, IloRangeArray, Data&);
	void addSubtourEliminationConstraint(IloModel, IloNumVarArray, IloRangeArray, Data&);
	void print(IloCplex, IloNumVarArray, IloRangeArray);

};

#endif // TSP_MIP_MTZ_H_
