//
//  main.cpp
//  ROCPP
//
//  Created by 靳晴 on 1/10/21.
//

#include <stdio.h>
#include <fstream>
#include "ROCPP.h"
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>



int main()
{
    // Create an empty robust model with T + 1 periods for the RSFC problem
    ROCPPOptModelIF_Ptr RSFCModel(new ROCPPDetOptModel());
    
    // Create decision variables
    ROCPPVarIF_Ptr x1(new ROCPPStaticVarReal("x1",0.));
    ROCPPVarIF_Ptr x2(new ROCPPStaticVarReal("x2",0.));
    
    RSFCModel->add_constraint(x1 + 2.0 * x2 + 0.5 <= 1.0);
    RSFCModel->add_constraint(2.0*x1 + 3.0*x2 <= 2.0);

    RSFCModel->set_objective(-1.0*x1 - x2 -0.5); // Add the objective to the problem
    
    // Construct the solver; in this case, use the gurobi solver as a deterministic solver
    ROCPPSolver_Ptr pSolver(new ROCPPSCIP(SolverParams()));
    
    ROCPPMISOCP_Ptr m(convertToMISOCP(RSFCModel));
    // Solve the problem
    pSolver->solve(m);
    
    // Retrieve the optimal solution from the solver
    map<string,double> optimalSln(pSolver->getSolution());
    
    // Get the optimal objective value
    double optVal(pSolver->getOptValue());

    return 0;
}
