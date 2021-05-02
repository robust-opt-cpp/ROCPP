//
//  ReformulationOrchestrator.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "ReformulationOrchestrator.hpp"
#include "IncludeFiles.hpp"



ROCPPOptModelIF_Ptr ReformulationOrchestrator::Reformulate(ROCPPOptModelIF_Ptr pIn, ROCPPStrategy_Ptr pStrategy) const
{
    if (!pStrategy->isApplicable(pIn))
        throw MyException("Reformulation strategy " + pStrategy->getName() + " not applicable");
    
    return pStrategy->Reformulate(pIn);
}


ROCPPOptModelIF_Ptr ReformulationOrchestrator::Reformulate(ROCPPOptModelIF_Ptr pIn, vector<ROCPPStrategy_Ptr> strategyVec) const
{
    ROCPPOptModelIF_Ptr pOut;
    
    for (vector<ROCPPStrategy_Ptr>::const_iterator sit = strategyVec.begin(); sit != strategyVec.end(); sit++)
    {
        if (sit == strategyVec.begin())
            pOut = Reformulate(pIn, (*sit));
        else
            pOut = Reformulate(pOut, (*sit));
    }
    
    return pOut;
}
