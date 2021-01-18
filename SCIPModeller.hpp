//
//  SCIPModeller.hpp
//  ROCPP
//
//  Created by 靳晴 on 1/16/21.
//

#ifndef SCIPModeller_hpp
#define SCIPModeller_hpp

#include <stdio.h>
#include <vector>
#include "HeaderIncludeFiles.hpp"
#include "SolverModeller.hpp"
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include <map>

class ConstraintIF;
class ObjectiveFunctionIF;
class CPLEXMISOCP;

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
    
    void solve(boost::shared_ptr<OptimizationModelIF> pModelIn, bool writeSlnToFile = false, string fileName = "", bool writeSlnToConsle = true, const map<string, double>& WSvars = (map<string, double>()), const map<string,int>& priorities = (map<string,int>()), bool deleteModel=false);
    
    void Reset();
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SCIPVariableContainer m_pSCIPVC;
    vector<SCIP_CONS*> m_pCstr;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    bool setParameters(SCIP* scip) const;
    //void saveResults(SCIP* scip, pair<bool,string> writeSlnToFile) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Creater Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    bool solveSCIPModel(boost::shared_ptr<CPLEXMISOCP> pModelIn, bool writeSlnToFile, string fileName, bool writeSlnToConsle);
    bool createSCIPmodel(boost::shared_ptr<CPLEXMISOCP> pModel, SCIP* scip);
    bool addSCIPdecisionVars(boost::shared_ptr<CPLEXMISOCP> pModel, SCIP* scip);
    bool addConstraint(SCIP* scip, boost::shared_ptr<ConstraintIF> pCstrIn, uint num);
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% SCIP SOLVER TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

typedef SCIPModeller ROCPPSCIP;
typedef boost::shared_ptr<SCIPModeller> ROCPPSCIP_Ptr;
#endif /* SCIPModeller_hpp */
