#ifndef CUTTINGPLANE_MIP_H_
#define CUTTINGPLANE_MIP_H_

#include <ilcplex/ilocplex.h>
#include "Data.h"


class CuttingPlane_MIP {

public:
	const std::string PROBLEM = "CuttingPlane_MIP";
	CuttingPlane_MIP();
	void run(Data&, bool option1);

private:
	void addDecisionVariables(IloModel, IloNumVarArray, Data&);
	void addDegreeConstraints(IloModel, IloNumVarArray, Data&);
	void addObjectiveFunction(IloModel, IloNumVarArray, Data&);
	bool addSubtourEliminationConstraint(IloCplex cplex, IloNumVarArray var, Data& data);
	bool addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray var, Data& data);
	void print(IloCplex, IloNumVarArray);

};

#endif // CUTTINGPLANE_MIP_H_
