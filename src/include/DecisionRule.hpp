/*
 * ROCPP/DecisionRule.hpp
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

#ifndef DecisionRule_hpp
#define DecisionRule_hpp

#include <stdio.h>
#include <vector>
#include <cmath>
#include "HeaderIncludeFiles.hpp"



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%% DECISION RULE APPROXIMATOR INTERFACE %%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


class ReformulationStrategyIF;

/// Decision rule approximator interface
class DecisionRuleIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the DecisionRuleIF class
    /// @param memory memory of the decision rule: number of time-periods for which the decision-maker can recall information observed
    DecisionRuleIF(uint memory=1000) : m_memory(memory) {}
    
    /// Destructor of the DecisionRuleIF class
    virtual ~DecisionRuleIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    uint getMemory() const {return m_memory;}
    
    virtual ROCPPOptModelIF_Ptr approximate(ROCPPOptModelIF_Ptr pIn) = 0;
    
    
protected:
    
    /// Memory of this approximator
    uint m_memory;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% CONTINOUS VARIABLE APPROXIMATOR INTERFACE %%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Continous variable decision rule interface class
class ContinuousVarsDRIF : public OneToExprVariableConverterIF, public DecisionRuleIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the continuous variable decision rule interface class
    ContinuousVarsDRIF(uint memory=1000) : DecisionRuleIF(memory) {}
    
    /// Destructor of the continuous variable decision rule interface class
    virtual ~ContinuousVarsDRIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Find all adaptive continuous variable in the given constraints and objective
    void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% LINEAR DECISION RULE APPROXIMATOR %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Linear decision rule
/*!
 Class for approximating continuous variable by linear function of the uncertain parameters
*/
class LinearDecisionRule: public ContinuousVarsDRIF, public ReformulationStrategyIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the LinearDecisionRule class
    /// @param UC Container that contains all uncertainty being used to approximate the decision variable in this problem
    LinearDecisionRule(ROCPPuncContainer_Ptr UC, uint memory=1000, double bigM=100.) : ContinuousVarsDRIF(memory), m_UC(UC), m_uncContSet(true), m_bigM(bigM) {}
    
    /// Constructor of the LinearDecisionRule class
    LinearDecisionRule(uint memory=1000, double bigM=100.) : ContinuousVarsDRIF(memory), m_uncContSet(false), m_bigM(bigM) {}
    
    /// Destructor of the LinearDecisionRule class
    ~LinearDecisionRule(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Approximated the real-valued adaptive decisions in the given model by linear desicion rules
    /// First set the uncertainty container and then do the approximation
    /// @param pIn Model to be approximated
    /// @param resetAndSave Indicates whether to reset the translation map in class OneToExprVariableConverterIF
    /// @see VariableConverterIF::convertVar(ROCPPOptModelIF_Ptr, bool)
    ROCPPOptModelIF_Ptr convertVar(ROCPPOptModelIF_Ptr pIn, bool resetAndSave=false);
    
    ROCPPOptModelIF_Ptr approximate(ROCPPOptModelIF_Ptr pIn);
    
    /// Create the map from the original decision variable to decisions that are affine in the history of observations
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd);
    
    /// Set the uncertainty container of this class
    void setUncContainer(ROCPPuncContainer_Ptr UC){m_UC=UC; m_uncContSet=true;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the coefficient decision variable associated with the given uncertain parameter in the approximation for the given decision variable
    ROCPPVarIF_Ptr getCoeffDV(string dvName, string uncName) const;
    
    /// Return map m_mapOrigDVUncPairToCoeffDV
    map<pair<string,string>, ROCPPVarIF_Ptr> getLDRCoeff() const{return m_mapOrigDVUncPairToCoeffDV;}
    
    /// Return map m_mapOrigDVToUncAndCoeffDV
    multimap<string, pair<string, ROCPPVarIF_Ptr> > getLDRExpr() const{return m_mapOrigDVToUncAndCoeffDV;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Print the solution (in the form of an expression) for the given variable
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &variableValue, ROCPPVarIF_Ptr dv);
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc);
    
    
    ROCPPOptModelIF_Ptr Reformulate(ROCPPOptModelIF_Ptr pIn){return approximate(pIn);}
    bool isApplicable(ROCPPOptModelIF_Ptr pIn) const;
    string getName() const {return "linear decision rule";}
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Uncertain parameter container of this approximator
    ROCPPuncContainer_Ptr m_UC;
    
    bool m_uncContSet;
    
    /// Map of pair of original variable name and uncertainty name to the coefficient variable
    map<pair<string,string>, ROCPPVarIF_Ptr> m_mapOrigDVUncPairToCoeffDV;
    
    /// Map of original variable name to the pair of uncertainty name and the coefficient variable
    multimap<string, pair<string, ROCPPVarIF_Ptr> > m_mapOrigDVToUncAndCoeffDV;
    
    /// Map of original variable name to the name of decision variable representing the constant term in the linear decision rule
    map<string, string> m_cst;
    
    /// Value of the big M constant
    double m_bigM;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% DISCRETE VARIABLE APPROXIMATOR INTERFACE %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Discrete variable decision rule interface
class DiscreteVarsDRIF : public OneToOneVariableConverterIF, DecisionRuleIF
{
    
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the DiscreteVarsDRIF class
    DiscreteVarsDRIF(uint memory=1000) : DecisionRuleIF(memory) {}
    
    /// Destructor of the DiscreteVarsDRIF class
    virtual ~DiscreteVarsDRIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Find all adaptive binary and integer variable in the given constraints and objective
    void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% CONSTANT VARIABLE APPROXIMATOR INTERFACE %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Constant decision rule
/*!
 Class for approximating discrete variables with an expression that is constant in the uncertain parameters
*/
class ConstantDecisionRule : public DiscreteVarsDRIF, public ReformulationStrategyIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the ConstantDecisionRule class
    ConstantDecisionRule(uint memory=1000) : DiscreteVarsDRIF(memory) {}
    
    /// Destructor of the ConstantDecisionRule class
    ~ConstantDecisionRule(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPVarIF_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &variableValue, ROCPPVarIF_Ptr dv);
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc);
    
    ROCPPOptModelIF_Ptr approximate(ROCPPOptModelIF_Ptr pIn);
    
    ROCPPOptModelIF_Ptr Reformulate(ROCPPOptModelIF_Ptr pIn){return approximate(pIn);}
    bool isApplicable(ROCPPOptModelIF_Ptr pIn) const;
    string getName() const {return "constant decision rule";}
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% TOOL FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool DoublesAreEssentiallyEqual(double A, double B, double epsilon);

bool allPositive(vector<uint> element);


template <class InputIterator>
double product(InputIterator first, InputIterator last);

double product(vector<double>::const_iterator first, vector<double>::const_iterator last);





#endif /* DecisionRule_hpp */
