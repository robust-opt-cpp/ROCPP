//
//  ReformulationOrchestrator.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef ReformulationOrchestrator_hpp
#define ReformulationOrchestrator_hpp

#include <stdio.h>
#include <vector>
#include "HeaderIncludeFiles.hpp"


class ReformulationStrategyIF;
typedef ReformulationStrategyIF ROCPPStrategy;
typedef shared_ptr<ROCPPStrategy> ROCPPStrategy_Ptr;


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
