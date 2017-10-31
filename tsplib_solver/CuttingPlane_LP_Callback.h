#ifndef CUTTINGPLANE_LP_CALLBACK_H
#define CUTTINGPLANE_LP_CALLBACK_H

#include <ilcplex/ilocplex.h>
#include <vector>

#include "Data.h"

class CuttingPlane_LP_Callback {

public:
	const std::string PROBLEM = "CuttingPlane_LP_Callback";
	CuttingPlane_LP_Callback();
	void run(Data&);

private:
	void addDecisionVariables(IloModel, IloNumVarArray, Data&);
	void addDegreeConstraints(IloModel, IloNumVarArray, Data&);
	void addObjectiveFunction(IloModel, IloNumVarArray, Data&);
	bool addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray var, Data& data);
	void addSubtourConstraints(IloCplex cplex, IloNumVarArray vars, const std::vector<int>& s, Data& data);
	void print(IloCplex, IloNumVarArray);
};

class MyCallback_LP : public IloCplex::LazyConstraintCallbackI
{
public:
	MyCallback_LP(IloEnv env, IloNumVarArray x, Data& data);

	void main();
	void addSubtourConstraints(IloNumVarArray x, const std::vector<int>& s, Data& data);
	IloCplex::CallbackI* duplicateCallback() const;
private:
	Data& data;
	IloNumVarArray x;
};

#endif // CUTTINGPLANE_LP_CALLBACK_H
