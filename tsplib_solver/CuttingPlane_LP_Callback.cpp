#include "CuttingPlane_LP_Callback.h"

#include <stack>
#include <ctime>

#define EPS 1e-5

using namespace std;

CuttingPlane_LP_Callback::CuttingPlane_LP_Callback() {
}

void CuttingPlane_LP_Callback::run(Data& data) {
	clock_t tstart = clock();

	IloEnv   env;
	IloModel model(env, PROBLEM.c_str());
	IloNumVarArray vars(env);
	size_t n = data.vertices.size();

	addDecisionVariables(model, vars, data);
	addObjectiveFunction(model, vars, data);
	addDegreeConstraints(model, vars, data);

	IloCplex cplex(model);
	cplex.setParam(IloCplex::Threads, 1);
	cplex.setOut(env.getNullStream());
	cplex.setWarning(env.getNullStream());

	while (true)
	{
#ifdef _DEBUG
		cplex.exportModel((PROBLEM + ".lp").c_str());
#endif
		if (!cplex.solve())	return;
		cout << "Obj = " << cplex.getObjValue() << endl;
		if (!addSubtourConnectionConstraint(cplex, vars, data)) break;
	}

	IloNumVarArray fixed(env);
	IloNumArray values(env);
	for (int i = 1; i < n; i++)
	{
		fixed.add(vars[data.indexToId(n, i - 1, i)]);
		values.add(1.0);
	}
	fixed.add(vars[data.indexToId(n, n - 1, 0)]);
	values.add(1.0);

	model.add(IloConversion(env, vars, IloNumVar::Bool));
	cplex.addMIPStart(fixed, values);

	cplex.use(new MyCallback_LP<IloCplex::LazyConstraintCallbackI>(env, vars, data));
	//cplex.use(new MyCallback_LP<IloCplex::UserCutCallbackI>(env, vars, data));
	cplex.setOut(cout);
	cplex.setWarning(cout);
	if (!cplex.solve()) return;

	cout << "Total time = " << (clock() - tstart) / (double)CLOCKS_PER_SEC << endl;

#ifdef _DEBUG
	print(cplex, vars);
#endif
}

void CuttingPlane_LP_Callback::addDecisionVariables(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	size_t n = data.vertices.size();
	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < n; j++) {
			vars.add(IloNumVar(env, 0, 1, ("x_" + to_string(i) + "_" + to_string(j)).c_str()));
			if (i == j) vars[vars.getSize() - 1].setBounds(0, 0);
		}
	}
}

void CuttingPlane_LP_Callback::addDegreeConstraints(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	IloRangeArray cons(env);

	size_t n = data.vertices.size();
	for (size_t i = 0; i < n; i++) {
		IloExpr expr(env);
		for (size_t j = 0; j < n; j++)
			expr += vars[data.vertices[i].edges[j].id];
		cons.add(IloRange(expr == 1));
		expr.end();

		expr = IloExpr(env);
		for (int j = 0; j < n; j++)
			expr += vars[data.vertices[j].edges[i].id];
		cons.add(IloRange(expr == 1));
		expr.end();
	}
	model.add(cons);
}

void CuttingPlane_LP_Callback::addObjectiveFunction(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	IloExpr expr(env);
	size_t n = data.vertices.size();
	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < n; j++) {
			Edge edge = data.vertices[i].edges[j];
			expr += edge.cost * vars[edge.id];
		}
	}
	model.add(IloMinimize(env, expr));
	expr.end();
}

IloRange getSubtourConstraint(IloEnv env, IloNumVarArray vars, const vector<int>& s, Data& data) {
	int n = (int)data.vertices.size();

	IloExpr expr(env);
	for (int i = 0; i < s.size(); i++) {
		for (int j = i; j < s.size(); j++) {
			expr += vars[data.vertices[s[i]].edges[s[j]].id];
			expr += vars[data.vertices[s[j]].edges[s[i]].id];
		}
	}
	IloRange range = IloRange(expr <= (int)s.size() - 1);
	expr.end();
	return range;
}

bool CuttingPlane_LP_Callback::addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray vars, Data& data) {
	int n = (int)data.vertices.size();
	vector<double> edges(vars.getSize());
	vector<vector<double>> costMatrix(vars.getSize());
	for (int i = 0; i < vars.getSize(); i++) edges[i] = cplex.getValue(vars[i]);
	for (int i = 0; i < vars.getSize(); i++) costMatrix[i].resize(n);
	Data::toMatrix(edges, n, costMatrix);

	vector<bool> marked(n, false);

	int contraintsAdded = 0;
	for (int j = 1; j < n; j++) {
		if (marked[j]) continue;
		vector<int> minCut;
		double maxFlow = data.findMinCut(0, j, costMatrix, minCut);
		if (maxFlow - 1 >= -EPS) continue; // if maxFLow >= 1

		vector<bool> in_cut(n, false);
		for (int i = 0; i < (int)minCut.size(); i++)
			in_cut[minCut[i]] = true;
		for (int i = 0; i < n; i++)
			if (!in_cut[i]) marked[i] = true;

		cplex.getModel().add(getSubtourConstraint(cplex.getEnv(), vars, minCut, data));
		if (++contraintsAdded == 10) break;
	}
	cout << "Added " << contraintsAdded << " constraints. ";
	return contraintsAdded > 0;
}

void CuttingPlane_LP_Callback::print(IloCplex cplex, IloNumVarArray vars) {
	IloEnv env = cplex.getEnv();
	IloNumArray vals(env);
	env.out() << "Solution status = " << cplex.getStatus() << endl;
	env.out() << "Solution value  = " << cplex.getObjValue() << endl;
	cplex.getValues(vals, vars);
	env.out() << "Values        = " << vals << endl;
	/*cplex.getSlacks(vals, con);
	env.out() << "Slacks        = " << vals << endl;*/
	/*cplex.getDuals(vals, con);
	env.out() << "Duals         = " << vals << endl;
	cplex.getReducedCosts(vals, var);
	env.out() << "Reduced Costs = " << vals << endl;*/
}

IloRangeArray separateConstraints(IloCplex::ControlCallbackI& callback, IloNumVarArray x, Data& data)
{
	IloRangeArray cons(callback.getEnv());
	int n = (int)data.vertices.size();

	vector<double> edges;
	vector<vector<double>> costMatrix;
	for (int i = 0; i < x.getSize(); i++) edges.push_back(callback.getValue(x[i]));
	for (int i = 0; i < x.getSize(); i++) costMatrix.push_back(vector<double>(n));
	Data::toMatrix(edges, n, costMatrix);

	vector<bool> marked(n, false);

	int contraintsAdded = 0;
	for (int j = 1; j < n; j++) {
		if (marked[j]) continue;
		vector<int> minCut;
		double maxFlow = data.findMinCut(0, j, costMatrix, minCut);
		if (maxFlow - 1 >= -EPS) continue; // if maxFLow >= 1

		vector<bool> in_cut(n, false);
		for (int i = 0; i < (int)minCut.size(); i++)
			in_cut[minCut[i]] = true;
		for (int i = 0; i < n; i++)
			if (!in_cut[i]) marked[i] = true;

		cons.add(getSubtourConstraint(callback.getEnv(), x, minCut, data));
		if (++contraintsAdded == 10) break;
	}
	return cons;
}
