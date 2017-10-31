#ifndef CUTTINGPLANE_LP_H_
#define CUTTINGPLANE_LP_H_

#include <ilcplex/ilocplex.h>
#include <vector>

#include "Data.h"

class CuttingPlane_LP {

public:
	const std::string PROBLEM = "CuttingPlane_LP";
	CuttingPlane_LP();
	void run(Data&);

private:
	void addDecisionVariables(IloModel, IloNumVarArray, Data&);
	void addDegreeConstraints(IloModel, IloNumVarArray, Data&);
	void addObjectiveFunction(IloModel, IloNumVarArray, Data&);
	bool addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray var, Data& data);
	void addSubtourConstraints(IloCplex cplex, IloNumVarArray vars, const std::vector<int>& s, Data& data);
	void print(IloCplex, IloNumVarArray);

};

#endif // CUTTINGPLANE_LP_H_
