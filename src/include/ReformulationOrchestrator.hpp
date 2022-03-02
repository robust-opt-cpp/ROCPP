/*
 * ROCPP/ReformulationOrchestrator.hpp
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

#ifndef ReformulationOrchestrator_hpp
#define ReformulationOrchestrator_hpp

#include <stdio.h>
#include <vector>
#include "HeaderIncludeFiles.hpp"


class ReformulationStrategyIF
{
public:
    ReformulationStrategyIF(){}
    ~ReformulationStrategyIF(){}
    
    virtual ROCPPOptModelIF_Ptr Reformulate(ROCPPOptModelIF_Ptr pIn) = 0;
    virtual bool isApplicable(ROCPPOptModelIF_Ptr pIn) const = 0;
    virtual string getName() const = 0;
};


class ReformulationOrchestrator
{
public:
    
    ReformulationOrchestrator(){}
    ~ReformulationOrchestrator(){}
    
    ROCPPOptModelIF_Ptr Reformulate(ROCPPOptModelIF_Ptr pIn, ROCPPStrategy_Ptr pStrategy) const;
    ROCPPOptModelIF_Ptr Reformulate(ROCPPOptModelIF_Ptr pIn, vector<ROCPPStrategy_Ptr> strategyVec) const;
    
};





#endif /* ReformulationOrchestrator_hpp */
