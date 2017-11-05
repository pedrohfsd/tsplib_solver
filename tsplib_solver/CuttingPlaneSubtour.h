#ifndef CUTING_PLANE_SUBTOUR_H_
#define CUTING_PLANE_SUBTOUR_H_

#include <ilcplex/ilocplex.h>
#include "Data.h"
#include "TSP.h"

class CuttingPlaneSubtour : public TSP {

public:
	CuttingPlaneSubtour(Data& data, bool singleThreaded);
	virtual void solve();
};

class CuttingPlaneSubtourCallback : public IloCplex::LazyConstraintCallbackI {
public:
	CuttingPlaneSubtourCallback(IloEnv env, IloNumVarArray vars, Data& data);
	void main();
	IloCplex::CallbackI* duplicateCallback() const;
private:
	Data& data;
	IloNumVarArray vars;
};

#endif // CUTING_PLANE_SUBTOUR_H_