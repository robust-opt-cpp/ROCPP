from ROPY import *

def PB():
	theta = 1.0
	T, I, M = 4, 5, 4

	c = {}
	c[1] = 0.69; c[2] = 0.43; c[3] = 0.01; c[4] = 0.91; c[5] = 0.64

	NomVal = {}
	NomVal[1] = 5.2; NomVal[2] = 8; NomVal[3] = 19.4; NomVal[4] = 9.6; NomVal[5] = 13.2
	
	RiskCoeff = {}
	RiskCoeff[1, 1] = 0.17; RiskCoeff[1, 2] = -0.7; RiskCoeff[1, 3] = -0.13; RiskCoeff[1, 4] = -0.6
	RiskCoeff[2, 1] = 0.39; RiskCoeff[2, 2] = 0.88; RiskCoeff[2, 3] = 0.74; RiskCoeff[2, 4] = 0.78
	RiskCoeff[3, 1] = 0.17; RiskCoeff[3, 2] = -0.6; RiskCoeff[3, 3] = -0.17; RiskCoeff[3, 4] = -0.84
	RiskCoeff[4, 1] = 0.09; RiskCoeff[4, 2] = -0.07; RiskCoeff[4, 3] = -0.52; RiskCoeff[4, 4] = 0.88
	RiskCoeff[5, 1] = 0.78; RiskCoeff[5, 2] = 0.94; RiskCoeff[5, 3] = 0.43; RiskCoeff[5, 4] = -0.58
	
	obsCost = {}
	for i in range(1, I + 1):
		obsCost[i] = {}
		for t in range(1, T + 1):
			obsCost[i][t] = c[i]
		
	# Create an empty robust model with T periods for the PB problem
	PBModel = RoPyOptModelDDID(T, uncOptModelObjType.robust)
	
	# Create dictionary for the uncertain parameter of value
	Value = {}
	for i in range(1, I + 1):
		# Create the uncertainty associated with box i and add it to Value
		Value[i] = RoPyUnc("Value_"+str(i))

	# Create dictionary for the uncertain parameters of risk factor
	Factor = {}
	for m in range(1, M + 1):
		# The risk factors are not observable
		Factor[m] = RoPyUnc("Factor_"+str(m), 1, False)
	
	# Create dictionary for the Measurement variables reprsenting the observation decisions
	# Create dictionary for the Keep variables representing the selction decisions
	MeasVar = {}; Keep = {}
	for i in range(1, I + 1): 
		# Create the measurement variables associated with the value of box i
		PBModel.addDDU(Value[i], 1, T, obsCost[i])
		# Get the measurement variables and store them in MeasVar
		for t in range(1, T + 1):
			MeasVar[t, i] = PBModel.getMeasVar(Value[i].getName(), t)
	
	for t in range(1, T + 1): 
		for i in range (1, I + 1): 
			if (t == 1):  # In the first period, the Keep variables are static
				Keep[t, i] = RoPyStaticVarBool("Keep_" + str(t) + "_" + str(i))
			else:  # In the other periods, the Keep variables are adaptive
				Keep[t, i] = RoPyAdaptVarBool("Keep_" + str(t) + "_" + str(i), t)
	
	# Create the constraints and add them to the problem
	StoppedSearch = RoPyExpr()
	for t in range(1, T + 1):  
		# Create the constraint that at most one box be opened at t (none if the search has stopped)
		NumOpened = RoPyExpr()
		# Update the expressions and and the constraint to the problem
		for i in range(1, I + 1): 
			StoppedSearch = StoppedSearch + Keep[t, i]
			if t > 1:
				NumOpened = NumOpened + MeasVar[t, i] - MeasVar[t - 1, i]
			else:
				NumOpened = NumOpened + MeasVar[t, i]
		
		PBModel.addConstraint( NumOpened <= 1. - StoppedSearch)
		# Constraint that only one of the open boxes can be kept
		for i in range(1, I + 1):
			PBModel.addConstraint( Keep[t, i] <= MeasVar[t - 1, i] if t > 1 else Keep[t, i] <= 0.0)
	
	
	# Create the uncertainty set constraints and add them to the problem
	# Add the upper and lower bounds on the risk factors
	for m in range(1, M + 1):
		PBModel.addConstraintUncSet(Factor[m] >= -1.0)
		PBModel.addConstraintUncSet(Factor[m] <= 1.0)
	
	# Add the expressions for the box values in terms of the risk factors
	for i in range(1, I + 1):
		ValueExpr = RoPyExpr()
		for m in range (1, M + 1):
			ValueExpr = ValueExpr + RiskCoeff[i, m] * Factor[m]
		PBModel.addConstraintUncSet(Value[i] == (1.0 + 0.5 * ValueExpr) * NomVal[i])
	
	# Create the objective function expression
	PBObj = RoPyExpr()
	for t in range(1, T + 1): 
		for i in range(1, I + 1):
			PBObj = PBObj + theta ** (t - 1) * Value[i] * Keep[t, i]
	# Set objective (multiply by -1 for maximization)
	PBModel.setObjective(-1.0 * PBObj)

	# Construct the reformulation orchestrator
	pRO = RoPyOrchestrator()
	
	# Construct the finite adaptability reformulation strategy with 2 candidate policies in the each time stage
	kMap = {2: 2, 3: 2, 4: 2}
	pKADR = RoPyKadapt(kMap)
	
	# Construct the robustify engine reformulation strategy
	pRE = RoPyRobustifyEngine()
	
	# Copnstruct the linearization reformulation strategy with big M approach
	pLinear = RoPyBTR_bigM()
	
	# Approximate the adaptive decisions using the linear/constant decision rule approximator and robustify
	strategyVec = [pKADR, pRE, pLinear]
	PBModelKADRFinal = pRO.Reformulate(PBModel, strategyVec)
	
	# Construct the solver
	pSolver = RoPySolver(RoPySolverParams())

	# Solve the problem
	pSolver.solve(PBModelKADRFinal)
	
	# Retrieve the optimal solution from the solver
	optimalSln = pSolver.getSolution()

	# Print the optimal decision (from the original model)
	# Prints decision rules for variable Keep_4_2 from the original problem automatically
	pKADR.printOut(PBModel, optimalSln, Keep[4, 2])
	# Prints the observation decision for uncertainty Value_2 from the original problem automatically
	pKADR.printOut(PBModel, optimalSln, Value[2])

if __name__ == '__main__':
	PB()

