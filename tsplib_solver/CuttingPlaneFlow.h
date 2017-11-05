#ifndef CUTING_PLANE_FLOW_H_
#define CUTING_PLANE_FLOW_H_

#include <ilcplex/ilocplex.h>
#include "Data.h"
#include "TSP.h"

class CuttingPlaneFlow : public TSP {

public:
	CuttingPlaneFlow(Data& data, bool singleThreaded);
	virtual void solve();
private:
	bool addFlowConstraint(IloCplex& cplex);
};

class CuttingPlaneFlowCallback : public IloCplex::LazyConstraintCallbackI {
public:
	CuttingPlaneFlowCallback(IloEnv env, IloNumVarArray vars, Data& data);

	void main();
	IloCplex::CallbackI* duplicateCallback() const;
private:
	Data& data;
	IloNumVarArray vars;
};
#endif // CUTING_PLANE_FLOW_H_