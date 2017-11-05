#include "CuttingPlaneCrossing.h"
#include "SubtourElimination.h"

#include <vector>

using namespace std;

CuttingPlaneCrossing::CuttingPlaneCrossing(Data& data, bool singleThreaded) :TSP(data, "CuttingPlaneCrossing", singleThreaded) {}

void CuttingPlaneCrossing::solve() {

	addDecisionVariables(true);
	addObjectiveFunction();
	addIncomingEdgeConstraints();
	addOutgoingEdgeConstraints();
	addSelfEdgeConstraints();

	model.add(cons);
	IloCplex cplex(model);
	cplex.exportModel((formulationName + ".lp").c_str());
	if (singleThreaded) cplex.setParam(IloCplex::Threads, 1);
	cplex.use(new CuttingPlaneCrossingCallback(env, vars, data));

	if (!cplex.solve()) {
		env.error() << "Failed to optimize LP" << endl;
		throw(-1);
	}

	print(cplex);
}

CuttingPlaneCrossingCallback::CuttingPlaneCrossingCallback(IloEnv env, IloNumVarArray vars, Data& data) : IloCplex::LazyConstraintCallbackI(env), vars(vars), data(data) {}

void CuttingPlaneCrossingCallback::main(){
	IloNumArray vals(getEnv(), vars.getSize());
	getValues(vals, vars);
	IloRange range = SubtourElimination::createCrossingConstraint(getEnv(), vars, vals, data);
	if (range.isValid()) add(range);
}

IloCplex::CallbackI* CuttingPlaneCrossingCallback::duplicateCallback() const
{
	return (new (getEnv()) CuttingPlaneCrossingCallback(*this));
}