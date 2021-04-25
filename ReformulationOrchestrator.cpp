//
//  ReformulationOrchestrator.cpp
//  ROCPP
//
//  Created by Phebe Vayanos on 4/24/21.
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
    ROCPPOptModelIF_Ptr pOut = pIn;
    
    for (vector<ROCPPStrategy_Ptr>::const_iterator sit = strategyVec.begin(); sit != strategyVec.end(); sit++)
    {
        pOut = (*sit)->Reformulate(pOut);
    }
    
    return pOut;
}
