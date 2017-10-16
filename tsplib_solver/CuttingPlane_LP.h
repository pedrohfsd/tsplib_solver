#ifndef CuttingPlane_LP_H_
#define CuttingPlane_LP_H_

class Data;

class CuttingPlane_LP {

public:
	const std::string PROBLEM = "CuttingPlane_LP";
	CuttingPlane_LP();
	void run(Data&);

private:
	void addDecisionVariables(IloModel, IloNumVarArray, Data&);
	void addDegreeConstraints(IloModel, IloNumVarArray, Data&);
	void addObjectiveFunction(IloModel, IloNumVarArray, Data&);
	bool addSubtourConnectionConstraint(IloCplex cplex, IloNumVarArray var, Data& data);
	void addSubtourConstraints(IloCplex cplex, IloNumVarArray vars, const std::vector<int>& s, const std::vector<int>& t, Data& data);
	void print(IloCplex, IloNumVarArray);

};

#endif // CuttingPlane_LP_H_
