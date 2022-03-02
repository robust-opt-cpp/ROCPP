/*
 * ROCPP/SCIPModeller.hpp
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

#ifdef USE_SCIP
#ifndef SCIPModeller_hpp
#define SCIPModeller_hpp

#include <stdio.h>
#include <vector>
#include "HeaderIncludeFiles.hpp"
#include "SolverModeller.hpp"
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "scip/cons_nonlinear.h"
#include "nlpi/nlpi_ipopt.h"
#include <map>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% SCIP VARIABLE CONTAINER %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

struct SCIPVariableContainer
{
    SCIPVariableContainer(){}
    ~SCIPVariableContainer(){}
    
    void Reset(){m_contVarMap.clear();m_boolVarMap.clear();m_intVarMap.clear(); m_allVarsMap.clear();}
    
    map<string,SCIP_Var*> m_contVarMap;
    map<string,SCIP_Var*> m_boolVarMap;
    map<string,SCIP_Var*> m_intVarMap;
    map<string,SCIP_Var*> m_allVarsMap;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% SCIP MODELLER %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Class used to interface with the SCIP deterministic optimization solvers
class SCIPModeller : public SolverModellerIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the SCIPModeller class
    SCIPModeller(const SolverParams &pSParams, bool useLazyNACs=false);
    
    /// Constructor of the SCIPModeller class with default parameters
    SCIPModeller();
    
    /// Destructor of the SCIPModeller class
    ~SCIPModeller(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void solve(ROCPPOptModelIF_Ptr pModelIn, bool writeSlnToFile = false, string fileName = "", bool writeSlnToConsle = true, const map<string, double>& WSvars = (map<string, double>()), const map<string,int>& priorities = (map<string,int>()), bool deleteModel=false);
    
    void Reset();
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    double getMIPGap() const;
    double getOptValue() const;
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SCIPVariableContainer m_pSCIPVC;
    vector<SCIP_CONS*> m_pCstr;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    SCIP_RETCODE setParameters(SCIP* scip) const;
    SCIP_RETCODE setPriorities(SCIP* scip, const map<string,int>& priorities);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Creater Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SCIP_RETCODE solveSCIPModel(ROCPPCPLEXMISOCP_Ptr pModelIn, bool writeSlnToConsle, const map<string, double>& WSvars, const map<string,int>& priorities);
    SCIP_RETCODE createSCIPmodel(ROCPPCPLEXMISOCP_Ptr pModel, SCIP* scip);
    SCIP_RETCODE addSCIPdecisionVars(ROCPPCPLEXMISOCP_Ptr pModel, SCIP* scip);
    SCIP_RETCODE addConstraint(SCIP* scip, ROCPPConstraintIF_Ptr pCstrIn, uint num);
};

#endif /* SCIPModeller_hpp */
#endif
