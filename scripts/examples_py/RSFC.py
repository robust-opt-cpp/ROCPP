from ROPY import *

def RSFC():

	T = 12; InitInventory = 0.0; InitCommit = 100.0; NomDemand = 100.0; rho = 0.1
	CostDPp = 10.0; CostDPm = 10.0; CostDCp = 10.0; CostDCm = 10.0

	OrderUB = {}; OrderLB = {};
	HoldingCost = {}; ShortageCost = {}; OrderCost = {}
	for t in range(1, T + 1):
		OrderLB[t] = 0.0
		OrderUB[t] = 200.0
		OrderCost[t] = 10.0
		ShortageCost[t + 1] = 10.0
		HoldingCost[t + 1] = 2.0
	

	CumOrderUB = {}; CumOrderLB = {}
	for t in range(1, T + 1):
		CumOrderLB[t] = 0.0
		CumOrderUB[t] = 200.0 * t

	# Set "box" here to use box uncertainty set
	uncertaintySetType = "ellipsoidal"

	# Safety parameter for ellipsoidal uncertainty
	Omega = 3.0
	Omega = Omega * NomDemand * rho

	# Create an empty robust model with T + 1 periods for the RSFC problem
	RSFCModel = ROPYUncMSOptModel(T + 1, uncOptModelObjType.robust)

	# Create the Demand map to store the uncertain parameters of the problem
	Demand = {}
	# Iterate over all time periods when there is uncertainty
	for t in range(1, T + 1):
		# Create the uncertain parameters, and add them to Demand
		Demand[t] = ROPYUnc("Demand_" + str(t), t + 1)


	# Create maps to store the decision variables of the problem
	Orders = {}; Commits = {}  # Quantity ordered, Commitments made
	# Iterate over all time periods from 1 to T
	for t in range(1, T + 1):
		# Create the commitment variables (these are static)
		Commits[t] = ROPYStaticVarDouble("Commit_" + str(t), 0.)
		if t == 1:  # In the first period, the order variables are static
			Orders[t] = ROPYStaticVarDouble("Order_" + str(t),OrderLB[t], OrderUB[t])
		else:  # In the other periods, the order variables are adaptive
			Orders[t] = ROPYAdaptVarDouble("Order_" + str(t), t, OrderLB[t], OrderUB[t])

	MaxDC = {}  # Upper bound on deviation between commitments
	MaxDP = {}  # Upper bound on deviation from plan
	MaxHS = {}  # Upper bound on holding and shortage costs
	# Iterate over all time periods 1 to T
	for t in range(1, T + 1):
		# Create upper bounds on the deviation between successive commitments
		MaxDC[t] = ROPYStaticVarDouble("MaxDC_" + str(t))
		# Create upper bounds on the deviation of orders from commitments
		if t == 1:  # In the first period, these are static
			MaxDP[t] = ROPYStaticVarDouble("MaxDP_" + str(t))
		else:  # In the other periods, these are adaptive
			MaxDP[t] = ROPYAdaptVarDouble("MaxDP_" + str(t), t)
		# Create upper bounds on holding and shortage costs (these are adaptive)
		MaxHS[t + 1] = ROPYAdaptVarDouble("MaxHS_" + str(t + 1), t + 1)

	# Create the constraints of the problem
	# Create an expression for the amount of inventory held and initialize it
	Inventory =  ROPYExpr()
	Inventory = Inventory + InitInventory
	# Create an expression for the cumulative amount ordered
	CumOrders = ROPYExpr()
	# Iterate over all time periods and add the constraints to the problem
	for t in range(1, T + 1):
		# Create the upper and lower bounds on the cumulative orders
		CumOrders = CumOrders + Orders[t]
		RSFCModel.addConstraint(CumOrders >= CumOrderLB[t])
		RSFCModel.addConstraint(CumOrders <= CumOrderUB[t])
		# Create upper bound on deviations from commitments
		RSFCModel.addConstraint(MaxDP[t] >= CostDPp*(Orders[t]-Commits[t]))
		RSFCModel.addConstraint(MaxDP[t] >= -CostDPm*(Orders[t]-Commits[t]))
		# Create upper bound on deviations between commitments
		if t == 1:
			RSFCModel.addConstraint(MaxDC[t] >= CostDCp*(Commits[t]-InitCommit))
			RSFCModel.addConstraint(MaxDC[t] >= -CostDCm*(Commits[t]-InitCommit))
		else:
			RSFCModel.addConstraint(MaxDC[t] >= CostDCp*(Commits[t]-Commits[t-1]))
			RSFCModel.addConstraint(MaxDC[t] >= -CostDCm*(Commits[t]-Commits[t-1]))
		
		# Update the inventory
		Inventory = Inventory + Orders[t] - Demand[t];
		# Create upper bound on shortage/holding costs
		RSFCModel.addConstraint(MaxHS[t+1] >= HoldingCost[t+1]*Inventory)
		RSFCModel.addConstraint(MaxHS[t+1] >= (-1.*ShortageCost[t+1]*Inventory))

	# Create an expression that will contain the objective function
	RSFCObj = ROPYExpr()
	# Iterate over all periods and add the terms to the objective function
	for t in range(1, T + 1):
		RSFCObj = RSFCObj + OrderCost[t] * Orders[t] + MaxDC[t] + MaxDP[t] + MaxHS[t+1]

	RSFCModel.setObjective(RSFCObj) # Add the objective to the problem


	# Create the uncertainty set
	if uncertaintySetType=="ellipsoidal":
		# Create a vector that will contain all the elements of the norm term
		EllipsoidElements = []
		# Populate the vector with the difference between the demand and the nominal demand
		for t in range(1, T + 1):
			EllipsoidElements.append(Demand[t] - NomDemand)
		# Create the norm term
		EllipsoidalConstraintTerm = ROPYNorm(EllipsoidElements)
		# Create the ellipsoidal uncertainty constraint
		RSFCModel.addConstraintUncSet(EllipsoidalConstraintTerm <= Omega)

	elif uncertaintySetType=="box":
		for t in range(1, T + 1): 
			# Add the upper and lower bounds on the demand to the uncertainty set
			RSFCModel.addConstraintUncSet(Demand[t] >= NomDemand*(1.0-rho))
			RSFCModel.addConstraintUncSet(Demand[t] <= NomDemand*(1.0+rho))
	
	# Construct the reformulation orchestrator
	pOrch = ROPYOrchestrator()
	
	# Construct the linear decision rule reformulation strategy
	pLDR = ROPYLinearDR()

	# Construct the robustify engine reformulation strategy
	pRE = ROPYRobustifyEngine()

	# Approximate the adaptive decisions using the linear/constant decision rule approximator and robustify
	strategyVec = [pLDR, pRE]
	RSFCModelLDRFinal = pOrch.Reformulate(RSFCModel, strategyVec)

	# Construct the solver
	pSolver = ROPYSolver(ROPYSolverParams())

	# Solve the problem
	pSolver.solve(RSFCModelLDRFinal)

	# Retrieve the optimal solution from the solver
	optimalSln = pSolver.getSolution()
	pLDR.printOut(RSFCModel, optimalSln, RSFCModel.getVar("Order_10"))

	# Get the optimal objective value
	optVal = pSolver.getOptValue()

	print(optVal)


if __name__ == '__main__':
	RSFC()
