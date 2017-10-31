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
	void print(IloCplex, IloNumVarArray);
};

template<typename T>
class MyCallback_LP : public T
{
public:
	MyCallback_LP(IloEnv env, IloNumVarArray x, Data& data) : T(env), x(x), data(data) {}

	void main()
	{
		IloRangeArray ranges = separateConstraints(*this, x, data);
		for (int i = 0; i < ranges.getSize(); i++)
			add(ranges[i]);
	}
	IloCplex::CallbackI* duplicateCallback() const { return (new (getEnv()) MyCallback_LP(*this)); }

private:
	Data& data;
	IloNumVarArray x;
};

#endif // CUTTINGPLANE_LP_CALLBACK_H
