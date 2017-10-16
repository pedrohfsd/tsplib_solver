#ifndef MTZ_H_
#define MTZ_H_

class Data;

class MTZ {

public:
	const std::string PROBLEM = "MTZ";
	MTZ();
	void run(Data&);

private:
	void addDecisionVariables(IloModel, IloNumVarArray, IloRangeArray, Data&);
	void addDegreeConstraints(IloModel, IloNumVarArray, IloRangeArray, Data&);
	void addObjectiveFunction(IloModel, IloNumVarArray, IloRangeArray, Data&);
	void addSubtourEliminationConstraint(IloModel, IloNumVarArray, IloRangeArray, Data&);
	void print(IloCplex, IloNumVarArray, IloRangeArray);

};

#endif // MTZ_H_
