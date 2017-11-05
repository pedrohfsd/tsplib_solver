#include "CuttingPlaneFlow.h"
#include "SubtourElimination.h"
#include <vector>

#define EPS 1e-5
using namespace std;

CuttingPlaneFlow::CuttingPlaneFlow(Data& data, bool singleThreaded) :TSP(data, "CuttingPlaneFlow", singleThreaded) {}

void CuttingPlaneFlow::solve() {
	clock_t tstart = clock();

	addDecisionVariables(false);
	addObjectiveFunction();
	addIncomingEdgeConstraints();
	addOutgoingEdgeConstraints();
	addSelfEdgeConstraints();

	model.add(cons);
	IloCplex cplex(model);
	if(singleThreaded) cplex.setParam(IloCplex::Threads, 1);
	cplex.setOut(env.getNullStream());
	cplex.setWarning(env.getNullStream());
	while (true) {
		#ifdef _DEBUG
		cplex.exportModel((formulationName + ".lp").c_str());
		#endif
		if (!cplex.solve())	return;
		cout << "Obj = " << cplex.getObjValue() << endl;
		if (!addFlowConstraint(cplex)) break;
	}
	cout << "Elpsed time on linear relaxation: " << (clock() - tstart) / 1000 << " sec." << endl;

	model.add(IloConversion(env, vars, IloNumVar::Bool));
	cplex.addMIPStart(vars, getFeasibleTour());
	cplex.use(new CuttingPlaneFlowCallback(env, vars, data));
	cplex.setOut(cout);
	cplex.setWarning(cout);
	if (!cplex.solve()) return;

	print(cplex);
}

bool CuttingPlaneFlow::addFlowConstraint(IloCplex& cplex) {
	IloNumArray vals(env, vars.getSize());
	cplex.getValues(vals, vars);
	IloRangeArray newCons = SubtourElimination::createFlowConstraint(env, vars, vals, data, 10, true);
	for (int i = 0; i < newCons.getSize(); i++) cplex.getModel().add(newCons[i]);
	return newCons.getSize() > 0;
}

CuttingPlaneFlowCallback::CuttingPlaneFlowCallback(IloEnv env, IloNumVarArray vars, Data& data) : IloCplex::LazyConstraintCallbackI(env), vars(vars), data(data) {}

void CuttingPlaneFlowCallback::main(){
	IloNumArray vals(getEnv(), vars.getSize());
	getValues(vals, vars);
	/*IloRange range = SubtourElimination::createInsideConstraint(getEnv(), vars, vals, data);
	if (range.isValid()) add(range);*/
	IloRangeArray newCons = SubtourElimination::createFlowConstraint(getEnv(), vars, vals, data, 10, true);
	for (int i = 0; i < newCons.getSize(); i++) add(newCons[i]);
}

IloCplex::CallbackI* CuttingPlaneFlowCallback::duplicateCallback() const{
	return (new (getEnv()) CuttingPlaneFlowCallback(*this));
}