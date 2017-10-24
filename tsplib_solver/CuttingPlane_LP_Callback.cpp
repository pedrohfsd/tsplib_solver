#include "CuttingPlane_LP_Callback.h"

#include <stack>

#define EPS 1e-5

using namespace std;

CuttingPlane_LP_Callback::CuttingPlane_LP_Callback() {
}

void CuttingPlane_LP_Callback::run(Data& data) {
	IloEnv   env;
	IloModel model(env, PROBLEM.c_str());
	IloNumVarArray vars(env);
	size_t n = data.vertices.size();

	addDecisionVariables(model, vars, data);
	addObjectiveFunction(model, vars, data);
	addDegreeConstraints(model, vars, data);

	IloCplex cplex(model);
	cplex.setParam(IloCplex::Threads, 1);
	//cplex.setOut(env.getNullStream());
	//cplex.setWarning(env.getNullStream());

	cplex.exportModel((PROBLEM + ".lp").c_str());
	cplex.use(new MyCallback_LP(env, vars, data));
	if (!cplex.solve())	return;
	cout << "Obj = " << cplex.getObjValue() << endl;

	print(cplex, vars);
}

void CuttingPlane_LP_Callback::addDecisionVariables(IloModel model, IloNumVarArray vars, Data& data) {
	IloEnv env = model.getEnv();
	size_t n = data.vertices.size();
	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < n; j++) {
			vars.add(IloBoolVar(env, 0, 1, ("x_" + to_string(i) + "_" + to_string(j)).c_str()));
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

bool CuttingPlane_LP_Callback::addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray vars, Data& data) {
	size_t n = data.vertices.size();
	vector<double> edges;
	vector<vector<double>> costMatrix;
	for (int i = 0; i < vars.getSize(); i++) edges.push_back(cplex.getValue(vars[i]));
	for (int i = 0; i < vars.getSize(); i++) costMatrix.push_back(vector<double>(n));
	Data::toMatrix(edges, n, costMatrix);

	for (size_t j = 1; j < n; j++) {
		vector<int> s;
		vector<int> t;
		double maxFlow = data.findMinCut(0, j, costMatrix, s);
		if (maxFlow - 1 > -EPS) continue; // if maxFLow == 1
		addSubtourConstraints(cplex, vars, s, t, data);
		return true;
	}
	return false;
}

void CuttingPlane_LP_Callback::addSubtourConstraints(IloCplex cplex, IloNumVarArray vars, const vector<int>& s, const vector<int>& t, Data& data) {
	IloModel model = cplex.getModel();
	IloRangeArray cons(cplex.getEnv());
	int n = data.vertices.size();

	IloExpr expr(cplex.getEnv());
	for (int i = 0; i < s.size(); i++) {
		for (int j = i; j < s.size(); j++) {
			expr += vars[data.vertices[s[i]].edges[s[j]].id];
			expr += vars[data.vertices[s[j]].edges[s[i]].id];
		}
	}
	cons.add(IloRange(expr <= (int)s.size() - 1));
	expr.end();
	model.add(cons);
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

MyCallback_LP::MyCallback_LP(IloEnv env, IloNumVarArray x, Data& data) : IloCplex::LazyConstraintCallbackI(env), x(x), data(data) {}

void MyCallback_LP::main()
{
	IloRangeArray cons(getEnv());
	IloExpr expr(getEnv());
	size_t n = data.vertices.size();

	vector<double> edges;
	vector<vector<double>> costMatrix;
	for (int i = 0; i < x.getSize(); i++) edges.push_back(getValue(x[i]));
	for (int i = 0; i < x.getSize(); i++) costMatrix.push_back(vector<double>(n));
	Data::toMatrix(edges, n, costMatrix);

	for (size_t j = 1; j < n; j++) {
		vector<int> s;
		vector<int> t;
		double maxFlow = data.findMinCut(0, j, costMatrix, s);
		if (maxFlow - 1 > -EPS) continue; // if maxFLow == 1
		addSubtourConstraints(x, s, t, data);
	}
}

void MyCallback_LP::addSubtourConstraints(IloNumVarArray x, const vector<int>& s, const vector<int>& t, Data& data) {
	IloModel model = getModel();
	IloRangeArray cons(getEnv());
	int n = data.vertices.size();

	IloExpr expr(getEnv());
	for (int i = 0; i < s.size(); i++) {
		for (int j = i; j < s.size(); j++) {
			expr += x[data.vertices[s[i]].edges[s[j]].id];
			expr += x[data.vertices[s[j]].edges[s[i]].id];
		}
	}
	cons.add(IloRange(expr <= (int)s.size() - 1));
	expr.end();
	model.add(cons);
}

IloCplex::CallbackI* MyCallback_LP::duplicateCallback() const
{
	return (new (getEnv()) MyCallback_LP(*this));
}