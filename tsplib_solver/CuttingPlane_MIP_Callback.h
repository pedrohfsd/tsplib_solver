#ifndef CuttingPlane_MIP_CALLBACK_H_
#define CuttingPlane_MIP_CALLBACK_H_

#include <ilcplex/ilocplex.h>

class Data;

class CuttingPlane_MIP_Callback {

public:
	const std::string PROBLEM = "CuttingPlane_MIP_Callback";
	CuttingPlane_MIP_Callback();
	void run(Data&, bool option1);

private:
	void addDecisionVariables(IloModel, IloNumVarArray, Data&);
	void addDegreeConstraints(IloModel, IloNumVarArray, Data&);
	void addObjectiveFunction(IloModel, IloNumVarArray, Data&);
	bool addSubtourEliminationConstraint(IloCplex cplex, IloNumVarArray var, Data& data);
	bool addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray var, Data& data);
	void print(IloCplex, IloNumVarArray);

};

class MyCallback : public IloCplex::LazyConstraintCallbackI
{
public:
	MyCallback(IloEnv env, IloNumVarArray x, Data& data);

	void main();

	IloCplex::CallbackI* duplicateCallback() const;
private:
	Data& data;
	IloNumVarArray x;
};

#endif
