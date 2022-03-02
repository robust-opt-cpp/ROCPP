#
# ROPy/BB.py
#

from ROPy import *


def BB():
	T, I = 4, 5
	B, theta = 163.0, 1.0
	CostUB = {1: 40.0, 2: 86.0, 3: 55.0, 4: 37.0, 5: 30.0}
	ValueUB = {1: 1030.0, 2: 1585.0, 3: 971.0, 4: 971.0, 5: 694.0}

	obsCost = {}
	for t in range(1, T+1):
		obsCost[t] = 0.0

	# Create an empty stochastic model with T periods for the BB problem
	BBModel = ROPyOptModelDDID(T, uncOptModelObjType.stochastic)

	Value = {}
	Cost = {}
	for i in range(1, I + 1):
		# Create the value and cost uncertainties associated with box i
		Value[i] = ROPyUnc("Value_" + str(i))
		Cost[i] = ROPyUnc("Cost_" + str(i))

	# Create the measurement decisions and pair the uncertain parameters
	MVval = {}
	MVcost = {}
	for i in range(1, I + 1):
		# Create the measurement variables associated with the value of box i
		BBModel.addDDU(Value[i], 1, T, obsCost)
		# Create the measurement variables associated with the cost of box i
		BBModel.addDDU(Cost[i], 1, T, obsCost)
		# Get the measurement variables and store them in MVval and MVcost
		for t in range(1, T + 1):
			MVval[t, i] = BBModel.getMeasVar(Value[i].getName(), t);
			MVcost[t, i] = BBModel.getMeasVar(Cost[i].getName(), t);

	# Pair the uncertain parameters to ensure they are observed at the same time
	for i in range(1, I + 1):
		BBModel.pairUncertainties(Value[i], Cost[i])

	# Create the keep decisions
	Keep = {}
	for t in range(1, T + 1):
		for i in range(1, I +1):
			if t == 1:  # In the first period, the Keep variables are static
				Keep[t, i] = ROPyStaticVarBool("Keep_"+str(t)+"_"+str(i))
			else:  # In the other periods, the Keep variables are adaptive
				Keep[t, i] = ROPyAdaptVarBool("Keep_"+str(t)+"_"+str(i), t)

	# Create the constraints and add them to the problem
	StoppedSearch = ROPyExpr()
	for t in range(1, T + 1):
		# Create the constraint that at most one box be opened at t (none if the search has stopped)
		NumOpened = ROPyExpr()
		# Update the expressions and and the constraint to the problem
		for i in range(1, I + 1):
			StoppedSearch = StoppedSearch + Keep[t, i]
			if t > 1:
				NumOpened = NumOpened + MVval[t, i] - MVval[t-1, i]
			else:
				NumOpened = NumOpened + MVval[t, i]
		BBModel.addConstraint(NumOpened <= 1. - StoppedSearch)
		# Constraint that only one of the open boxes can be kept
		for i in range(1, I + 1):
			BBModel.addConstraint(Keep[t, i] <= MVval[t - 1, i] if t > 1 else Keep[t, i] <= 0.0)

	# Constraint on the amount spent
	AmountSpent = ROPyExpr()
	for i in range(1, I + 1):
		AmountSpent = AmountSpent + Cost[i] * MVval[T, i]
	BBModel.addConstraint(AmountSpent <= B)

	# Create the uncertainty set constraints and add them to the problem
	for i in range(1, I + 1):
		# Add the upper and lower bounds on the values
		BBModel.addConstraintUncSet(Value[i] >= 0.0)
		BBModel.addConstraintUncSet(Value[i] <= ValueUB[i])
		# Add the upper and lower bounds on the costs
		BBModel.addConstraintUncSet(Cost[i] >= 0.)
		BBModel.addConstraintUncSet(Cost[i] <= CostUB[i])

	# Create the objective function expression
	BBObj = ROPyExpr()
	for t in range(1, T + 1):
		for i in range(1, I + 1):
			BBObj = BBObj + theta ** (t - 1) * Value[i] * Keep[t, i]

	# Set objective (multiply by -1 for maximization)
	BBModel.setObjective(-1.0 * BBObj)

	# Construct the reformulation orchestrator
	pOrch = ROPyOrchestrator()

	# Construct the piecewise linear decision rule reformulation strategy
	# Build the dict containing the breakpoint configuration
	BPconfig = {}
	BPconfig["Value_1"] = 3
	BPconfig["Value_2"] = 3
	BPconfig["Value_4"] = 3
	pPWApprox = ROPyPWDR(BPconfig)

	# Construct the robustify engine reformulation strategy
	pRE = ROPyRobustifyEngine()
	
	# Approximate the adaptive decisions using the linear/constant decision rule approximator and robustify
	strategyVec = [pPWApprox, pRE]
	BBModelPWCFinal = pOrch.Reformulate(BBModel, strategyVec)

	pSolver = ROPySolver(ROPySolverParams())

	# Solve the problem
	pSolver.solve(BBModelPWCFinal)
	# Retrieve the optimal solution from the solver
	optimalSln = pSolver.getSolution()
	# Print the optimal decision (from the original model)
	# Prints decision rules from the original problem automatically
	pPWApprox.printOut(BBModelPWCFinal, optimalSln, Keep[4, 4])
	pPWApprox.printOut(BBModel, optimalSln, Value[4])


if __name__ == '__main__':
	BB()




































