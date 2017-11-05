#ifndef MTZ_H_
#define MTZ_H_

#include <ilcplex/ilocplex.h>
#include "Data.h"
#include "TSP.h"

class MTZ: public TSP{

public:
	MTZ(Data& data, bool singleThreaded);
	virtual void solve();

private:
	IloNumVarArray u;
	void addSubtourEliminationConstraints();
	void addSubtourEliminationVariables();
};

#endif // MTZ_H_
