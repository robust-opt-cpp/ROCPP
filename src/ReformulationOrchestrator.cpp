/*
 * ROCPP/ReformulationOrchestrator.cpp
 *
 * This software is Copyright © 2020 The University of Southern California. All Rights Reserved.
 * Authors: Phebe Vayanos, Qing Jin, George Elissaios
 *
 * This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Paper: "ROC++: Robust Optimization in C++"
 * Homepage: https://sites.google.com/usc.edu/robust-opt-cpp/home
 */

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
