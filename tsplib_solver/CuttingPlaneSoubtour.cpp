#include "CuttingPlaneSubtour.h"
#include "SubtourElimination.h"

#include <vector>

using namespace std;

CuttingPlaneSubtour::CuttingPlaneSubtour(Data& data, bool singleThreaded) :TSP(data, "CuttingPlaneSubtour", singleThreaded) {}

void CuttingPlaneSubtour::solve() {

	addDecisionVariables(true);
	addObjectiveFunction();
	addIncomingEdgeConstraints();
	addOutgoingEdgeConstraints();
	addSelfEdgeConstraints();

	model.add(cons);
	IloCplex cplex(model);
	cplex.exportModel((formulationName + ".lp").c_str());
	if (singleThreaded) cplex.setParam(IloCplex::Threads, 1);
	cplex.use(new CuttingPlaneSubtourCallback(env, vars, data));

	if (!cplex.solve()) {
		env.error() << "Failed to optimize LP" << endl;
		throw(-1);
	}

	print(cplex);
}

CuttingPlaneSubtourCallback::CuttingPlaneSubtourCallback(IloEnv env, IloNumVarArray vars, Data& data) : IloCplex::LazyConstraintCallbackI(env), vars(vars), data(data) {}

void CuttingPlaneSubtourCallback::main()
{
	IloNumArray vals(getEnv(), vars.getSize());
	getValues(vals, vars);
	IloRange range = SubtourElimination::createInsideConstraint(getEnv(), vars, vals, data);
	if (range.isValid()) add(range);
}

IloCplex::CallbackI* CuttingPlaneSubtourCallback::duplicateCallback() const
{
	return (new (getEnv()) CuttingPlaneSubtourCallback(*this));
}