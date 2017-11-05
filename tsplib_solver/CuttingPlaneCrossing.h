#ifndef CUTING_PLANE_CROSSING_H_
#define CUTING_PLANE_CROSSING_H_

#include <ilcplex/ilocplex.h>
#include "Data.h"
#include "TSP.h"

class CuttingPlaneCrossing : public TSP {

public:
	CuttingPlaneCrossing(Data& data, bool singleThreaded);
	virtual void solve();
};

class CuttingPlaneCrossingCallback : public IloCplex::LazyConstraintCallbackI {
public:
	CuttingPlaneCrossingCallback(IloEnv env, IloNumVarArray vars, Data& data);
	void main();
	IloCplex::CallbackI* duplicateCallback() const;
private:
	Data& data;
	IloNumVarArray vars;
};

#endif // CUTING_PLANE_CROSSING_H_