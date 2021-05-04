//
//  RSFC.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "ROCPP.h"
#include <vector>
#include <map>
#include <fstream>
#include <iomanip>


int main(int argc, const char * argv[]) {

    uint T(12);
    double InitInventory(0.);
    double InitCommit(100.);
    double NomDemand(100.);
    double rho(0.1);
    double CostDPp(10.);
    double CostDPm(10.);
    double CostDCp(10.);
    double CostDCm(10.);

    map<uint,double> OrderUB;
    map<uint,double> OrderLB;
    map<uint,double> HoldingCost;
    map<uint,double> ShortageCost;
    map<uint,double> OrderCost;
    for (uint t=1; t<=T; t++)
    {
        OrderLB[t]=0.;
        OrderUB[t]=200.;
        OrderCost[t]=10.;
        ShortageCost[t+1]=10.;
        HoldingCost[t+1]=2.;
    }

    map<uint,double> CumOrderUB;
    map<uint,double> CumOrderLB;
    for (uint t=1; t<=T; t++)
    {
        CumOrderLB[t]=0.;
        CumOrderUB[t]=200.0*t;
    }

    // Set "box" here to use box uncertainty set
    string uncertaintySetType("ellipsoidal");

    // Safety parameter for ellipsoidal uncertainty
    double Omega(3.);
    Omega = Omega*NomDemand*rho;

    // Create an empty robust model with T + 1 periods for the RSFC problem
    ROCPPOptModelIF_Ptr RSFCModel(new ROCPPUncMSOptModel(T+1, robust));

    // Create the Demand map to store the uncertain parameters of the problem
    map<uint,ROCPPUnc_Ptr> Demand;
    // Iterate over all time periods when there is uncertainty
    for (uint t=1; t<=T; t++)
        // Create the uncertain parameters, and add them to Demand
        Demand[t] = ROCPPUnc_Ptr(new ROCPPUnc("Demand_"+to_string(t),t+1));


    // Create maps to store the decision variables of the problem
    map<uint,ROCPPVarIF_Ptr> Orders, Commits;    // Quantity ordered, Commitments made
    // Iterate over all time periods from 1 to T
    for (uint t=1; t<=T; t++) {
        // Create the commitment variables (these are static)
        Commits[t]=ROCPPVarIF_Ptr(new ROCPPStaticVarReal("Commit_"+to_string(t),0.));
        if (t==1) // In the first period, the order variables are static
            Orders[t] = ROCPPVarIF_Ptr(new ROCPPStaticVarReal("Order_"+to_string(t),OrderLB[t],OrderUB[t]));
        else // In the other periods, the order variables are adaptive
            Orders[t] = ROCPPVarIF_Ptr(new ROCPPAdaptVarReal("Order_"+to_string(t),t,OrderLB[t],OrderUB[t]));
    }

    map<uint,ROCPPVarIF_Ptr> MaxDC; // Upper bound on deviation between commitments
    map<uint,ROCPPVarIF_Ptr> MaxDP; // Upper bound on deviation from plan
    map<uint,ROCPPVarIF_Ptr> MaxHS; // Upper bound on holding and shortage costs
    // Iterate over all time periods 1 to T
    for (uint t=1; t<=T; t++) {
        // Create upper bounds on the deviation between successive commitments
        MaxDC[t] = ROCPPVarIF_Ptr(new ROCPPStaticVarReal("MaxDC_"+to_string(t)));
        // Create upper bounds on the deviation of orders from commitments
        if (t==1)   // In the first period, these are static
            MaxDP[t] = ROCPPVarIF_Ptr(new ROCPPStaticVarReal("MaxDP_"+to_string(t)));
        else        // In the other periods, these are adaptive
            MaxDP[t] = ROCPPVarIF_Ptr(new ROCPPAdaptVarReal("MaxDP_"+to_string(t),t));
        // Create upper bounds on holding and shortage costs (these are adaptive)
        MaxHS[t+1]=ROCPPVarIF_Ptr(new ROCPPAdaptVarReal("MaxHS_"+to_string(t+1),t+1));
    }

    // Create the constraints of the problem
    // Create an expression for the amount of inventory held and initialize it
    ROCPPExpr_Ptr Inventory(new ROCPPExpr());
    Inventory = Inventory + InitInventory;
    // Create an expression for the cumulative amount ordered
    ROCPPExpr_Ptr CumOrders(new ROCPPExpr());
    // Iterate over all time periods and add the constraints to the problem
    for (uint t=1; t<=T; t++) {
        // Create the upper and lower bounds on the cumulative orders
        CumOrders = CumOrders + Orders[t];
        RSFCModel->add_constraint(CumOrders >= CumOrderLB[t]);
        RSFCModel->add_constraint(CumOrders <= CumOrderUB[t]);
        // Create upper bound on deviations from commitments
        RSFCModel->add_constraint(MaxDP[t] >= CostDPp*(Orders[t]-Commits[t]));
        RSFCModel->add_constraint(MaxDP[t] >= -CostDPm*(Orders[t]-Commits[t]));
        // Create upper bound on deviations between commitments
        if (t==1) {
            RSFCModel->add_constraint(MaxDC[t] >= CostDCp*(Commits[t]-InitCommit));
            RSFCModel->add_constraint(MaxDC[t] >= -CostDCm*(Commits[t]-InitCommit));
        }
        else {
            RSFCModel->add_constraint(MaxDC[t] >= CostDCp*(Commits[t]-Commits[t-1]));
            RSFCModel->add_constraint(MaxDC[t] >= -CostDCm*(Commits[t]-Commits[t-1]));
        }
        // Update the inventory
        Inventory = Inventory + Orders[t] - Demand[t];
        // Create upper bound on shortage/holding costs
        RSFCModel->add_constraint(MaxHS[t+1] >= HoldingCost[t+1]*Inventory);
        RSFCModel->add_constraint(MaxHS[t+1] >= (-1.*ShortageCost[t+1]*Inventory));
    }

    // Create an expression that will contain the objective function
    ROCPPExpr_Ptr RSFCObj(new ROCPPExpr());
    // Iterate over all periods and add the terms to the objective function
    for (uint t=1; t<=T; t++) {
        RSFCObj = RSFCObj + OrderCost[t] * Orders[t] + MaxDC[t] + MaxDP[t] + MaxHS[t+1];
    }
    RSFCModel->set_objective(RSFCObj); // Add the objective to the problem


    // Create the uncertainty set
    if (uncertaintySetType=="ellipsoidal")
    {
        // Create a vector that will contain all the elements of the norm term
        vector<ROCPPExpr_Ptr> EllipsoidElements;
        // Populate the vector with the difference between the demand and the nominal demand
        for (uint t=1; t<=T; t++)
            EllipsoidElements.push_back(Demand[t] - NomDemand);
        // Create the norm term
        shared_ptr<ConstraintTermIF> EllipsoidalConstraintTerm(new NormTerm(EllipsoidElements));
        // Create the ellipsoidal uncertainty constraint
        RSFCModel->add_constraint_uncset(EllipsoidalConstraintTerm <= Omega);

    }
    else if (uncertaintySetType=="box")
    {
        for (uint t=1; t<=T; t++) {
            // Add the upper and lower bounds on the demand to the uncertainty set
            RSFCModel->add_constraint_uncset(Demand[t] >= NomDemand*(1.0-rho));
            RSFCModel->add_constraint_uncset(Demand[t] <= NomDemand*(1.0+rho));
        }
    }

    // Construct the reformulation orchestrator
    ROCPPOrchestrator_Ptr pRO(new ROCPPOrchestrator());

    // Construct the linear/constant decision rule reformulation strategy
    ROCPPStrategy_Ptr pLDR(new ROCPPLinearDR());
    // Construct the robustify engine reformulation strategy
    ROCPPStrategy_Ptr pRE(new ROCPPRobustifyEngine());

    // Approximate the adaptive decisions using the linear/constant decision rule approximator and robustify
    vector<ROCPPStrategy_Ptr> strategyVec {pLDR, pRE};
    ROCPPOptModelIF_Ptr RSFCModelLDRFinal = pRO->Reformulate(RSFCModel, strategyVec);

#ifdef USE_SCIP
    // Construct the solver; in this case, use the gurobi solver as a deterministic solver
    ROCPPSolverInterface_Ptr pSolver(new ROCPPSCIP(SolverParams()));
#elif defined(USE_GUROBI)
    ROCPPSolverInterface_Ptr pSolver(new ROCPPGurobi(SolverParams()));
#endif
    // Solve the problem
    pSolver->solve(RSFCModelLDRFinal);

    // Retrieve the optimal solution from the solver
    map<string,double> optimalSln(pSolver->getSolution());
    // Print the optimal decision (from the original model)
    // Prints decision rules from the original problem automatically
    ROCPPLinearDR_Ptr pLDRApprox = static_pointer_cast<LinearDecisionRule>(pLDR);
    pLDRApprox->printOut(RSFCModel, optimalSln, RSFCModel->getVar("Order_10"));

    // Get the optimal objective value
    double optVal(pSolver->getOptValue());

    cout << std::setprecision(8) << optVal << endl;

    return 0;
}
