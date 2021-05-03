//
//  BB.cpp
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


int main(int argc, const char * argv[])
{
    
    uint T(4);
    uint I(5);
    double B(163.0);
    double theta(1.0);

    map<uint, double> CostUB;
    CostUB[1]=40.;CostUB[2]=86.;CostUB[3]=55.;CostUB[4]=37.;CostUB[5]=30.;

    map<uint, double> ValueUB;
    ValueUB[1]=1030.;ValueUB[2]=1585.;ValueUB[3]=971.;ValueUB[4]=971.;ValueUB[5]=694.;

    map<uint, double> obsCost;
    for (uint t = 1; t <= T; t++){
        obsCost.insert(make_pair(t, 0.));
    }
    
    // Create an empty stochastic model with T periods for the BB problem
    ROCPPOptModelIF_Ptr BBModel(new ROCPPOptModelDDID(T, stochastic));

    map<uint, ROCPPUnc_Ptr> Value, Cost;
    for (uint i = 1; i <= I; i++) {
        // Create the value and cost uncertainties associated with box i
        Value[i] = ROCPPUnc_Ptr(new ROCPPUnc("Value_"+to_string(i)));
        Cost[i] = ROCPPUnc_Ptr(new ROCPPUnc("Cost_"+to_string(i)));
    }

    // Create the measurement decisions and pair the uncertain parameters
    map<uint, map<uint, ROCPPVarIF_Ptr> > MVcost, MVval;
    for (uint i = 1; i <= I; i++) {
        // Create the measurement variables associated with the value of box i
        BBModel->add_ddu(Value[i], 1, T, obsCost);
        // Create the measurement variables associated with the cost of box i
        BBModel->add_ddu(Cost[i], 1, T, obsCost);
        // Get the measurement variables and store them in MVval and MVcost
        for (uint t = 1; t <= T; t++) {
            MVval[t][i] = BBModel->getMeasVar(Value[i]->getName(), t);
            MVcost[t][i] = BBModel->getMeasVar(Cost[i]->getName(), t);
        }
    }

    // Pair the uncertain parameters to ensure they are observed at the same time
    for (uint i = 1; i <= I; i++)
        BBModel->pair_uncertainties(Value[i], Cost[i]);

    // Create the keep decisions
    map<uint, map<uint, ROCPPVarIF_Ptr> > Keep;
    for (uint t = 1; t <= T; t++) {
        for (uint i = 1; i <= I; i++) {
            if (t == 1)  // In the first period, the Keep variables are static
                Keep[t][i] = ROCPPVarIF_Ptr(new ROCPPStaticVarBool("Keep_"+to_string(t)+"_"+to_string(i)));
            else   // In the other periods, the Keep variables are adaptive
                Keep[t][i] = ROCPPVarIF_Ptr(new ROCPPAdaptVarBool("Keep_"+to_string(t)+"_"+to_string(i), t));
        }
    }

    // Create the constraints and add them to the problem
    ROCPPExpr_Ptr StoppedSearch(new ROCPPExpr());
    for (uint t = 1; t <= T; t++) {
        // Create the constraint that at most one box be opened at t (none if the search has stopped)
        ROCPPExpr_Ptr NumOpened(new ROCPPExpr());
        // Update the expressions and and the constraint to the problem
        for (uint i = 1; i <= I; i++) {
            StoppedSearch = StoppedSearch + Keep[t][i];
            if (t>1)
                NumOpened = NumOpened + MVval[t][i] - MVval[t-1][i];
            else
                NumOpened = NumOpened + MVval[t][i];
        }
        BBModel->add_constraint( NumOpened <= 1. - StoppedSearch );
        // Constraint that only one of the open boxes can be kept
        for (uint i = 1; i <= I; i++)
            BBModel->add_constraint( (t>1) ? (Keep[t][i] <= MVval[t-1][i]) : (Keep[t][i] <= 0.));
    }

    // Constraint on the amount spent
    ROCPPExpr_Ptr AmountSpent(new ROCPPExpr());
    for (uint i = 1; i <= I; i++)
        AmountSpent = AmountSpent + Cost[i] * MVval[T][i];
    BBModel->add_constraint(AmountSpent <= B);
    
    // Create the uncertainty set constraints and add them to the problem
    for (uint i = 1; i <= I; i++) {
        // Add the upper and lower bounds on the values
        BBModel->add_constraint_uncset(Value[i] >= 0.);
        BBModel->add_constraint_uncset(Value[i] <= ValueUB[i]);
        // Add the upper and lower bounds on the costs
        BBModel->add_constraint_uncset(Cost[i] >= 0.);
        BBModel->add_constraint_uncset(Cost[i] <= CostUB[i]);
    }

    // Create the objective function expression
    ROCPPExpr_Ptr BBObj(new ROCPPExpr());
    for (uint t = 1; t <= T; t++)
        for (uint i = 1; i <= I; i++)
            BBObj = BBObj + pow(theta,t-1)*Value[i]*Keep[t][i];

    // Set objective (multiply by -1 for maximization)
    BBModel->set_objective(-1.0*BBObj);

    // Construct the reformulation orchestrator
    ROCPPOrchestrator_Ptr pRO(new ReformulationOrchestrator());
    
    // Construct the piecewise linear decision rule reformulation strategy
    // Build the map containing the breakpoint configuration
    map<string,uint> BPconfig;
    BPconfig["Value_1"] = 3;
    BPconfig["Value_2"] = 3;
    BPconfig["Value_4"] = 3;
    ROCPPStrategy_Ptr pPWApprox(new PiecewiseDecisionRule(BPconfig));
    
    // Construct the robustify engine reformulation strategy
    ROCPPStrategy_Ptr pRE (new RobustifyEngine());
    
    // Approximate the adaptive decisions using the linear/constant decision rule approximator and robustify
    vector<ROCPPStrategy_Ptr> strategyVec {pPWApprox, pRE};
    ROCPPOptModelIF_Ptr BBModelPWCFinal = pRO->Reformulate(BBModel, strategyVec);
    
    
#ifdef USE_SCIP
    // Construct the solver; in this case, use the gurobi solver as a deterministic solver
    ROCPPSolverInterface_Ptr pSolver(new ROCPPSCIP(SolverParams()));
#elif defined(USE_GUROBI)
    ROCPPSolverInterface_Ptr pSolver(new ROCPPGurobi(SolverParams()));
#endif
    // Solve the problem
    pSolver->solve(BBModelPWCFinal);
    // Retrieve the optimal solution from the solver
    map<string,double> optimalSln(pSolver->getSolution());
    // Print the optimal decision (from the original model)
    // Prints decision rules from the original problem automatically
    ROCPPPWDR_Ptr pPWApproxDR = static_pointer_cast<PiecewiseDecisionRule>(pPWApprox);
    
    pPWApproxDR->printOut(BBModel, optimalSln, Keep[4][4]);
    pPWApproxDR->printOut(BBModel, optimalSln, Value[4]);

    return 0;
}
