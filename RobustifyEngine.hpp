//
//  RobustifyEngine.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef RobustifyEngine_hpp
#define RobustifyEngine_hpp

#include "HeaderIncludeFiles.hpp"


/// Robustify engine class (automatically dualizes all semi-infinite constraints)
class RobustifyEngine : public ReformulationStrategyIF
{
public:
    
    RobustifyEngine(uint dualVarsCounter = 0, string dualNme_suff = "", string dualNme = "dual") :
    m_dualNme(dualNme), m_dualNme_suff(dualNme_suff), m_dualVarsCounter(dualVarsCounter), m_uncertaintySetMatricesCalculated(false) {}
    ~RobustifyEngine(){}
    
    /// Robusity the given model
    /// @note First calculate the UncertaintySetMatrices then robustify each constraint.
    /// @see CalculateUncertaintySetMatrices, robustifyConstraint
    /// @param pIn Model to be robusified
    /// @param feasible Indicate whether the given constraint is feasible or not, only set to false when we dualize an infeasible problem
    ROCPPBilinMISOCP_Ptr robustify(ROCPPUncSSOptModel_Ptr pIn, bool feasible = true);
    
    /// Calculate the coefficient map for uncertainties and the coefficient map for deterministic part
    /// @note Store the coefficent of each uncertainty  in each linear expression(out of and in the norm term)  of each constraint defining uncertainty set in m_EMvec
    /// @note Store the deterministic part in each linear expression(out of and in the norm term) in each constraint defining uncertainty set in m_EVvec
    void calculateUncertaintySetMatrices(ROCPPUncSSOptModel_Ptr const pIn);
    
    /// Create and store the dual variables in the given constraint and add duality constraint into output model
    /// @param pOut Optimization model after dualization
    /// @param pCstr Constraint to be robustified
    /// @param dualVars Store dual variable for each linear expression(outside and in the norm term) in each constraint defining uncertainty set and store it in this vector
    /// @param feasible Indicate whether the given constraint is feasible or not, only set to false when we dualize an infeasible problem
    void createDualVars(ROCPPBilinMISOCP_Ptr pOut, ROCPPConstraint_Ptr pCstr, vector<vector<ROCPPVarIF_Ptr> >& dualVars, bool feasible = true);
    
    /// Robusify the given constraint
    /// @param pConstraint Constraint to be robustified
    /// @param pIn Model to be robusified
    /// @param feasible Indicate whether the given constraint is feasible or not, only set to false when we dualize an infeasible problem
    /// @note First create the dual variables being used for robustifying this constraint, then add the dual constraint into the output model
    /// @see createDualVars
    void robustifyConstraint(ROCPPConstraint_Ptr pConstraint, ROCPPUncSSOptModel_Ptr const pIn, ROCPPBilinMISOCP_Ptr pOut, bool feasible = true);
    
    /// Get the number of dual variables
    uint getDualVarsCnt() const {return m_dualVarsCounter;}
    
    
    
    ROCPPOptModelIF_Ptr Reformulate(ROCPPOptModelIF_Ptr pIn);//{return ROCPPOptModelIF_Ptr(robustify(ROCPPOptModelIF_Ptr(pIn)));}
    bool isApplicable(ROCPPOptModelIF_Ptr pIn) const;
    string getName() const {return "robustification";}
    
private:
    
    /// Name of the dual variable
    string m_dualNme;
    string m_dualNme_suff;
    /// Number of the dual variabls
    uint m_dualVarsCounter;
    /// Indicate whether the uncertainty set matriced are calculated or not
    bool m_uncertaintySetMatricesCalculated;
    ///  Map from time-stage to -> dimensions are constraint defining the us, number of linear expressions in that constraint, number of total uncertainties in the model
    map<uint, vector<vector<vector< pair<bool, ROCPPExpr_Ptr> > > > > m_EMvec; // map from time-stage to -> dimensions are constraint defining the us, row of us constraint matrix, col of us constraint matrix
    /// Map from time-stage -> dimensions are constraint defining the us number, number of linear expressions in that constraint
    map<uint, vector<vector< ROCPPExpr_Ptr> > > m_EVvec; // map from time-stage dimensions are constraint defining the us number, row of us constraint matrix
    /// Map from pair( time-stage, constraint num) to bool = true if constraint is equality constraint
    map<pair<uint,uint>, bool> m_isEqCstr; // map from pair( time-stage, constraint num) to bool = true if constraint is equality constraint
};

#endif /* RobustifyEngine_hpp */
