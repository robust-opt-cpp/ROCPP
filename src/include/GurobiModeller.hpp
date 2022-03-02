/*
 * ROCPP/GurobiModeller.hpp
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

#ifdef USE_GUROBI
#ifndef GurobiModeller_hpp
#define GurobiModeller_hpp

#include "HeaderIncludeFiles.hpp"
#include "SolverModeller.hpp"
#include "gurobi_c++.h"
#include <map>

class GRBVar;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% GUROBI VARIABLE CONTAINER %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

struct GurobiVariableContainer
{
    GurobiVariableContainer(){}
    ~GurobiVariableContainer(){}
    
    void Reset(){m_contVarMap.clear();m_boolVarMap.clear();m_intVarMap.clear(); m_allVarsMap.clear();}
    
    map<string,GRBVar> m_contVarMap;
    map<string,GRBVar> m_boolVarMap;
    map<string,GRBVar> m_intVarMap;
    map<string,GRBVar> m_allVarsMap;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% GUROBI MODELLER %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Class used to interface with the Gurobi deterministic optimization solvers
class GurobiModeller : public SolverModellerIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the GurobiModeller class
    GurobiModeller(const SolverParams &pSParams, bool useLazyNACs=false);
    
    /// Constructor of the GurobiModeller class with default parameters
    GurobiModeller();
    
    /// Destructor of the GurobiModeller class
    ~GurobiModeller(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    double getMIPGap() const;
    double getOptValue() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void solve(ROCPPOptModelIF_Ptr pModelIn, bool writeSlnToFile = false, string fileName = "", bool writeSlnToConsle = true, const map<string, double>& WSvars = (map<string, double>()), const map<string,int>& priorities = (map<string,int>()), bool deleteModel=false);
    
    
    void Reset();
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    GurobiVariableContainer m_pGurobiVC;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void setParameters(GRBModel& Model) const;
    void setPriorities(const map<string,int>& priorities);
    void saveResults(GRBModel& Model, pair<bool,string> writeSlnToFile) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Creater Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void createGUROBImodel(ROCPPCPLEXMISOCP_Ptr pModel,GRBEnv &env,GRBModel &Model); //  IloNumVarArray &contVars,IloBoolVarArray &boolVars, IloIntVarArray &intVars,
    void addGUROBIdecisionVars(ROCPPCPLEXMISOCP_Ptr pModel, GRBEnv &env, GRBModel &Model);
    void addConstraint(GRBEnv &env, GRBModel& Model, ROCPPConstraintIF_Ptr pCstrIn) const;
    void addObjective(GRBEnv &env, GRBModel& Model, ROCPPObjectiveIF_Ptr pObjIn) const;
    void addGUROBIwarmstart(const map<string,double>& WSvars) const;
};


#endif /* GurobiModeller_hpp */
#endif
