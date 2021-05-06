//
//  PB.cpp
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
    
    double theta(1.);
    uint T(4);
    uint I(5);
    uint M(4);

    map<uint, double> c;
    c[1]=0.69;c[2]=0.43;c[3]=0.01;c[4]=0.91;c[5]=0.64;

    map<uint, double> NomVal;
    NomVal[1]=5.2;NomVal[2]=8;NomVal[3]=19.4;NomVal[4]=9.6;NomVal[5]=13.2;
    
    map<uint, map<uint, double> > RiskCoeff;
    RiskCoeff[1][1]=0.17; RiskCoeff[1][2]= -0.7; RiskCoeff[1][3]= -0.13; RiskCoeff[1][4]= -0.6;
    RiskCoeff[2][1]=0.39; RiskCoeff[2][2]=0.88; RiskCoeff[2][3]=0.74; RiskCoeff[2][4]=0.78;
    RiskCoeff[3][1]=0.17; RiskCoeff[3][2]= -0.6; RiskCoeff[3][3]= -0.17; RiskCoeff[3][4]= -0.84;
    RiskCoeff[4][1]=0.09; RiskCoeff[4][2]= -0.07; RiskCoeff[4][3]= -0.52; RiskCoeff[4][4]=0.88;
    RiskCoeff[5][1]=0.78; RiskCoeff[5][2]=0.94; RiskCoeff[5][3]=0.43; RiskCoeff[5][4]= -0.58;
    
    map<uint, map<uint, double> > obsCost;
    for (uint i = 1; i <= I; i++)
    {
        for (uint t = 1; t <= T; t++){
            obsCost[i].insert(make_pair(t, c[i]));
        }
    }
    
    // Create an empty robust model with T periods for the PB problem
    ROCPPOptModelIF_Ptr PBModel(new ROCPPOptModelDDID(T, robust));
    
    // Create map for the uncertain parameter of value
    map<uint, ROCPPUnc_Ptr> Value;
    for (uint i = 1; i <= I; i++)
        // Create the uncertainty associated with box i and add it to Value
        Value[i] = ROCPPUnc_Ptr(new ROCPPUnc("Value_"+to_string(i)));

    // Create map for the uncertain parameters of risk factor
    map<uint, ROCPPUnc_Ptr> Factor;
    for (uint m = 1; m <= M; m++)
        // The risk factors are not observable
        Factor[m]= ROCPPUnc_Ptr(new ROCPPUnc("Factor_"+to_string(m),1,false));
    
    // Create map for the Measurement variables reprsenting the observation decisions
    // Create map for the Keep variables representing the selction decisions
    map<uint, map<uint, ROCPPVarIF_Ptr> > MeasVar, Keep;
    for (uint i = 1; i <= I; i++) {
        // Create the measurement variables associated with the value of box i
        PBModel->add_ddu(Value[i], 1, T, obsCost[i]);
        // Get the measurement variables and store them in MeasVar
        for (uint t = 1; t <= T; t++)
            MeasVar[t][i] = PBModel->getMeasVar(Value[i]->getName(), t);
    }
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
                NumOpened = NumOpened + MeasVar[t][i] - MeasVar[t-1][i];
            else
                NumOpened = NumOpened + MeasVar[t][i];
        }
        PBModel->add_constraint( NumOpened <= 1. - StoppedSearch );
        // Constraint that only one of the open boxes can be kept
        for (uint i = 1; i <= I; i++)
            PBModel->add_constraint( (t>1) ? (Keep[t][i] <= MeasVar[t-1][i]) : (Keep[t][i] <= 0.));
    }
    
    // Create the uncertainty set constraints and add them to the problem
    // Add the upper and lower bounds on the risk factors
    for (uint m = 1; m <= M; m++) {
        PBModel->add_constraint_uncset(Factor[m] >= -1.0);
        PBModel->add_constraint_uncset(Factor[m] <= 1.0);
    }
    // Add the expressions for the box values in terms of the risk factors
    for (uint i = 1; i <= I; i++) {
        ROCPPExpr_Ptr ValueExpr(new ROCPPExpr());
        for (uint m = 1; m <= M; m++)
            ValueExpr = ValueExpr + RiskCoeff[i][m]*Factor[m];
        PBModel->add_constraint_uncset(  Value[i] == (1.+0.5*ValueExpr) * NomVal[i] );
    }
    
    
    // Create the objective function expression
    ROCPPExpr_Ptr PBObj(new ROCPPExpr());
    for (uint t = 1; t <= T; t++)
        for (uint i = 1; i <= I; i++)
            PBObj = PBObj + pow(theta,t-1)*Value[i]*Keep[t][i];
    // Set objective (multiply by -1 for maximization)
    PBModel->set_objective(-1.0*PBObj);

    // Construct the reformulation orchestrator
    ROCPPOrchestrator_Ptr pOrch(new ROCPPOrchestrator());
    
    // Construct the finite adaptability reformulation strategy with 2 candidate policies in the each time stage
    map<uint, uint> kMap = {{2, 2}, {3, 2}, {4, 2}};
    ROCPPStrategy_Ptr pKadaptStrategy(new ROCPPKAdapt(kMap));
    
    // Construct the robustify engine reformulation strategy
    ROCPPStrategy_Ptr pRE (new ROCPPRobustifyEngine());
    
    //Copnstruct the linearization reformulation strategy with big M approach
    ROCPPStrategy_Ptr pBTR (new ROCPPBTR_bigM());
    
    // Approximate the adaptive decisions using the linear/constant decision rule approximator and robustify
    vector<ROCPPStrategy_Ptr> strategyVec {pKadaptStrategy, pRE, pBTR};
    ROCPPOptModelIF_Ptr PBModelKADRFinal = pOrch->Reformulate(PBModel, strategyVec);
    
    // Construct the solver; in this case, use the gurobi or SCIP solver as a deterministic solver
    SolverParams sparams = SolverParams();
#ifdef USE_GUROBI
    ROCPPSolverInterface_Ptr pSolver(new GurobiModeller(sparams, false) );
#elif defined(USE_SCIP)
    ROCPPSolverInterface_Ptr pSolver(new SCIPModeller(sparams, false) );
#else
    throw MyException("Can not find your solver.");
#endif
    // Solve the problem
    pSolver->solve(PBModelKADRFinal);
    
    // Retrieve the optimal solution from the solver
    map<string,double> optimalSln(pSolver->getSolution());

    // Print the optimal decision (from the original model)
    // Print decision rules for variable Keep_4_2 from the original problem automatically
    ROCPPKAdapt_Ptr pKADRApprox = static_pointer_cast<KadaptabilityDecisionRule>(pKADR);
    pKADRApprox->printOut(PBModel, optimalSln, Keep[4][2]);
    // Prints the observation decision for uncertainty Value_2 from the original problem automatically
    pKADRApprox->printOut(PBModel, optimalSln, Value[2]);

    return 0;
}
