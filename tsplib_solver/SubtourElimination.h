#ifndef SUBTOUR_ELIMINATION_H_
#define SUBTOUR_ELIMINATION_H_

#include <ilcplex/ilocplex.h>
#include "Data.h"

class SubtourElimination {

public:
	static IloRange createCrossingConstraint(const IloEnv& env, const IloNumVarArray& vars, const IloNumArray& vals, Data& data);
	static IloRange createCrossingConstraint(const IloEnv &env, const IloNumVarArray &vars, const IloNumArray &vals, Data &data, std::vector<int> &subtour);
	static IloRange createInsideConstraint(const IloEnv& env, const IloNumVarArray& vars, const IloNumArray& vals, Data& data);
	static IloRange createInsideConstraint(const IloEnv &env, const IloNumVarArray &vars, const IloNumArray &vals, Data &data, std::vector<int> &subtour);
	static IloRangeArray createFlowConstraint(const IloEnv& env, const IloNumVarArray& vars, const IloNumArray& vals, Data& data, int maxLimit, bool skipRedundants);
private:
	static void findSubtour(const IloEnv& env, const IloNumVarArray& vars, const IloNumArray& vals, Data& data, std::vector<int>& connectedComponent);
};

#endif // SUBTOUR_ELIMINATION_H_