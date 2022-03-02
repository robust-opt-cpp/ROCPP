/*
 * ROCPP/Constraint.hpp
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


#ifndef Constraint_hpp
#define Constraint_hpp

#include <stdio.h>
#include <map>
#include <vector>
#include "HeaderIncludeFiles.hpp"


typedef vector<ROCPPCstrTermIF_Ptr>::const_iterator ConstraintLHS_const_iterator;
typedef vector<ROCPPCstrTermIF_Ptr>::iterator ConstraintLHS_iterator;


/// Constraint right hand side parameters class
struct rhsParams
{
    /// Constructor of the rhsParams structure
    rhsParams() : m_rhs(0.), m_rhsIsZero(true), m_rhsSet(false) {}
    
    /// Value of the right hand side
    double m_rhs;
    
    /// Parameter indicating if the right hand side of the constraint is zero
    bool m_rhsIsZero;
    
    /// Parameter indicating if the right hand side of the constraint is set
    bool m_rhsSet;
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% CONSTRAINT INTERFACE %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Constraint interface class
class ConstraintIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Constructor of the constraint interface class
    /// @param definesUncertaintySet boolean variable, indicates if this constraint defines the uncertainty set or not
    /// @param isNAC boolean variable, indicates if this constraint is a non-anticipativity constraint or not
    ConstraintIF(bool definesUncertaintySet, bool isNAC=false);
    
    /// Destructor of the constraint interface class
    virtual ~ConstraintIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Constant iterator in decision variable map
    typedef dvMapType::const_iterator varsIterator;
    
    /// Return a constant iterator pointing to the beginning of the decision variable map
    virtual varsIterator varsBegin() const = 0;
    
    /// Return a constant iterator pointing to the end of the decision variable map
    virtual varsIterator varsEnd() const = 0;
    
//    /// Constant iterator in the uncertain parameter map
//    typedef uncMapType::const_iterator uncertaintiesIterator;
//
//    /// Return a constant iterator pointing to the beginning of the uncertain parameter map
//    /// @warning Not valid in SOS constraint
//    virtual uncertaintiesIterator uncertaintiesBegin() const;
//
//    /// Return a constant iterator pointing to the end of the uncertain parameter map
//    /// @warning Not valid in SOS constraint
//    virtual uncertaintiesIterator uncertaintiesEnd() const;
//
//    /// Constant iterator in LHSExpression terms
//    typedef ConstraintLHS_const_iterator const_iterator;
//
//    /// Return a constant iterator pointing to the beginning of the LHSExpression terms
//    /// @warning Not valid in SOS constraint
//    virtual const_iterator begin() const;
//
//    /// Return a constant iterator pointing to the end of the LHSExpression terms
//    /// @warning Not valid in SOS constraint
//    virtual const_iterator end() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Map the decision variables in this constraint to new variables
    virtual ROCPPConstraintIF_Ptr mapVars(const map<string, ROCPPExpr_Ptr> &mapFromVarToExpression) const = 0;
    
    /// Map the uncertain parameters in this constraint to new uncertain parameters
    virtual ROCPPConstraintIF_Ptr mapUncs(const map<string, ROCPPExpr_Ptr> &mapFromUncToExpression) const = 0;
    
    /// Replace the given term in this expression with the given decision variable
    virtual ROCPPConstraintIF_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr> &term, ROCPPVarIF_Ptr var) const = 0;
    
    /// Replace the bilinear term in this objecet with the given decision variable
    virtual ROCPPConstraintIF_Ptr replaceBilinearTerm(map<pair<string,string>, ROCPPVarIF_Ptr> &allTerm, uint &count) const = 0;
    
    /// Map the decision variables in this constraint to some expressions
    virtual ROCPPConstraintIF_Ptr mapVars(const map<string,ROCPPVarIF_Ptr> &mapFromOldToNewVars) const = 0;
    
    /// Map the uncertain parameters in this constraint to some expressions
    virtual ROCPPConstraintIF_Ptr mapUnc(const map<string,ROCPPUnc_Ptr> &mapFromOldToNewUnc) const = 0;
    
    /// Add the decisions variables involved in a product in this term to the given container dvs
    virtual void add_vars_involved_in_prod(dvContainer &dvs) const = 0;
    
    /// Add the integer variables in this term to the given container
    virtual void add_int_vars(dvContainer &dvs) const = 0;
    
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c, ROCPPVarIF_Ptr pVariable);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c, ROCPPUnc_Ptr pUncertainty,  ROCPPVarIF_Ptr pVariable);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c, ROCPPUnc_Ptr pUncertainty);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c, ROCPPVarIF_Ptr pVariable1, ROCPPVarIF_Ptr pVariable2);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c, ROCPPUnc_Ptr pUncertainty, ROCPPVarIF_Ptr pVariable1, ROCPPVarIF_Ptr pVariable2);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(ROCPPconstCstrTermIF_Ptr term);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c, ROCPPconstCstrTermIF_Ptr term);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(ROCPPconstExpr_Ptr pExpression);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c, ROCPPconstExpr_Ptr pExpression);
//
//    /// @note Only valid in classic constraint
//    virtual void add_lhs(double c, ROCPPconstExpr_Ptr pExpression,  ROCPPVarIF_Ptr pVariable);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Get the number of times the term given in the multimap appears in this constraint
    /// @param term multimap from the name of the decision variable to the desicion variable used to define this term
    /// @note The same decision variable may appear many times in a term (indicating that we have a product of the same variable)
    virtual uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr> &term) const = 0;
    
    /// Get the all products of two variables in this constraint.
    virtual void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr> > &termMap) const = 0;
    
    /// Get the decision variable container of this constraint
    virtual ROCPPconstdvContainer_Ptr getDVContainer() const = 0;
    
    /// Get the uncertain parameter container of this constraint
    virtual ROCPPconstuncContainer_Ptr getUncContainer() const = 0;
    
    /// Get the number of real-valued decision variables in this constraint
    virtual uint getNumContVars() const = 0;
    
    /// Get the number of integer decision variables in this constraint
    virtual uint getNumIntVars() const = 0;
    
    /// Get the number of boolean variables in this constraint
    virtual uint getNumBoolVars() const = 0;
    
    /// Get the number of adaptive real-valued decision variables in this constraint
    virtual uint getNumAdaptiveContVars() const = 0;
    
    /// Get the number of adaptive decision variables in this constraint
    virtual uint getNumAdaptiveVars() const = 0;

    /// Get the maximum time-stage of any decision variable in the constraint
    virtual uint getTimeStage() const = 0;
    
    /// Get the number of uncertaint parameters in the constraint
    virtual size_t getNumUncertainties() const = 0;
    
    /// Return true if and only if there is a product between two decision variables in this constraint
    virtual bool hasNonlinearities() const = 0;
    
    /// Return true if and only if the constraint involves a product of uncertain parameters
    virtual bool hasProdsUncertainties() const = 0;
    
    /// Return true if and only if there is a product between two real-valued decision variables in this constraint
    virtual bool hasProdsContVars() const = 0;
    
    /// Return true if and only if this object is not empty
    virtual bool isWellDefined() const = 0;
    
    /// Return true if and only if this constraint defines the uncertainty set
    bool definesUncertaintySet() const {return m_definesUncertaintySet;}
    
    /// Return true if and only if this constraint does not involve any uncertain parameters
    bool isDeterministic() const {return ( getNumUncertainties() == 0 );}
    
    /// Return false if and only if the left hand side is constant and the left and right hand side constants are equal, within epsilon (this constraint can be deleted)
    virtual bool isUseful(double epsilon = 1.e-10)  const {return true;}
    
    /// Return true if and only if the constraint is a non-anticipativity constraint (useful for problems with decision-dependent information discovery)
    bool isNAC() const {return m_isNAC;}
    
    /// Return true if and only if the constraint has no decision variables
    virtual bool hasNoDVs() const = 0;
    
    /// Return true if and only if the constraint is a classic (equality or inequality) constraint
    virtual bool isClassicConstraint() const {return false;}
    
    /// Return true if and only if the constraint is SOS
    virtual bool isSOSConstraint() const {return false;}
    
    /// Return true if and only if the constraint is a logical if then constraint (useful in CPLEX solver)
    virtual bool isIfThenConstraint() const {return false;}
    
    /// Find if any decision variable in the container dvs exists in this object
    /// @note Calls dvContainer::AnyVarIsInvolved()
    virtual bool AnyVarIsInvolved(dvContainer& dvs) const = 0;
    
    /// Return true if and only if the constraint has a norm term
    virtual bool hasNormTerm() const = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Set the paramaters of this constraint
    /// @param defUncertaintyset parameter to indicate if this constraint defines the uncertainty set or not
    /// @param isNAC parameter to indicate if this constraint is a non-anticipativity constraint or not
    void setParams(bool defUncertaintyset, bool isNAC) {m_definesUncertaintySet = defUncertaintyset; m_isNAC = isNAC;}
    
    /// Set the right hand side of this constraint
    /// @note Only valid in classic constraint
//    virtual void set_rhs(pair<double,bool> rhs);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return a copy of this constraint
    virtual ROCPPConstraintIF_Ptr Clone() const = 0;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Write the constraint to a stream
    /// @param ofs output file stream
    /// @param cnt number (counter) of the constraint
    virtual void WriteToStream(ofstream &ofs, uint cnt) const = 0;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
protected:
    
    /// Indicate whether this constraint defines uncertainty set or not
    bool m_definesUncertaintySet;
    
    /// Indicate whether this constraint is a non-anticipativity constraint or not
    bool m_isNAC;
};





//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% CLASSIC CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Classic constraint interface class
class ClassicConstraintIF : public ConstraintIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of classic constraint class
    /// @param definesUncertaintySet true if and only if this constraint is used to define the uncertainty set
    /// @param isNAC true if and only if this constraint is a non-anticipativity constraint
    ClassicConstraintIF(bool definesUncertaintySet, bool isNAC=false);
    
    /// Destructor of classic constraint class
    ~ClassicConstraintIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator in LHSExpression terms
    typedef ConstraintLHS_const_iterator const_iterator;
    
    /// @note Call LHExpression::begin() for the left-hand side expression of this constraint
    const_iterator begin() const;
    
    /// @note Call LHExpression::end() for the left-hand side expression of this constraint
    const_iterator end() const;
    
    /// @note Call LHExpression::varsBegin() for the left-hand side expression of this constraint
    varsIterator varsBegin() const;
    
    /// @note Call LHExpression::varsEnd() for the left-hand side expression of this constraint
    varsIterator varsEnd() const;
    
    
    /// Constant iterator in the uncertain parameter map
        typedef uncMapType::const_iterator uncertaintiesIterator;
    
    
    /// @note Call LHExpression::uncBegin() for the left-hand side expression of this constraint
    uncertaintiesIterator uncertaintiesBegin() const;
    
    /// @note Call LHExpression::uncEnd() for the left-hand side expression of this constraint
    uncertaintiesIterator uncertaintiesEnd() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Create a new ProductTerm using the given input and add it to the left hand side of this constraint
    /// @param c constant to add to the expression
    void add_lhs(double c);
    
    /// Create a new ProductTerm using the given inputs and add it to the left hand side of this constraint
    /// @param c coefficient of the term to add
    /// @param pVariable decision variable of the term to add
    void add_lhs(double c, ROCPPVarIF_Ptr pVariable);
    
    /// Create a new ProductTerm using the given inputs and add it to the left hand side of this constraint
    /// @param c coefficient of the term to add
    /// @param pUncertainty uncertain parameter of the term to add
    /// @param pVariable decision variable of the term to add
    void add_lhs(double c, ROCPPUnc_Ptr pUncertainty,  ROCPPVarIF_Ptr pVariable);
    
    /// Create a new ProductTerm using the given inputs and add it to the left hand side of this constraint
    /// @param c coefficient of the term to add
    /// @param pUncertainty uncertain parameter of the term to add
    void add_lhs(double c, ROCPPUnc_Ptr pUncertainty);
    
    /// Create a new ProductTerm using the given inputs and add it to the left hand side of this constraint
    /// @param c coefficient of the term to add
    /// @param pVariable1 first decision variable of the term to add
    /// @param pVariable2 second decision variable of the term to add
    void add_lhs(double c, ROCPPVarIF_Ptr pVariable1, ROCPPVarIF_Ptr pVariable2);
    
    /// Create a new ProductTerm using the given inputs and add it to the left hand side of this constraint
    /// @param c coefficient of the term to add
    /// @param pUncertainty uncertain parameter of the term to add
    /// @param pVariable1 first decision variable of the term to add
    /// @param pVariable2 second decision variable of the term to add
    void add_lhs(double c, ROCPPUnc_Ptr pUncertainty, ROCPPVarIF_Ptr pVariable1, ROCPPVarIF_Ptr pVariable2);
    
    /// Add the given term to the left hand side of this constraint
    /// @note If the term already exists, change the coefficient
    /// @warning One expression can only have one norm term with coefficient equal to 1
    void add_lhs(ROCPPconstCstrTermIF_Ptr term);
    
    /// Add the product of the given inputs into the left hand side of this constraint
    /// @param c coefficient
    /// @param term term to add to expression after multiplying it by coefficient
    /// @note Call LHExpression::add(ROCPPconstCstrTermIF_Ptr) after multiplying term by c
    void add_lhs(double c, ROCPPconstCstrTermIF_Ptr term);
    
    /// Add the given expression to the left hand side of this constraint
    /// @param pExpression expression to add to this expression after multiplying it by coefficient c
    /// @note Call LHExpression::add(ROCPPconstExpr_Ptr) after scaling it by
    void add_lhs(ROCPPconstExpr_Ptr pExpression);
    
    /// Add the given expression multiplied by c to the left hand side of this constraint
    /// @param c coefficient
    /// @param pExpression expression to add to this expression after multiplying it by coefficient c
    /// @note Call LHExpression::add(ROCPPconstExpr_Ptr) after scaled
    void add_lhs(double c, ROCPPconstExpr_Ptr pExpression);
    
    /// Add the product of the given inputs to the left hand side of this constraint
    /// @param c coefficient
    /// @param pVariable decision variable
    /// @param pExpression expression to add to this expression after multiplying it by coefficient c and by decision variable pVariable
    /// @note Call LHExpression::add(ROCPPconstExpr_Ptr)
    void add_lhs(double c, ROCPPconstExpr_Ptr pExpression,  ROCPPVarIF_Ptr pVariable);
    
    /// Set right-hand side parameter of this constraint
    /// @param rhs first element of the pair is the right-hand side parameter, the second one is true if the parameter is 0, false otherwise
    void set_rhs(pair<double,bool> rhs);
    
    /// @note Call LHExpression::mapVars(const map<string, ROCPPExpr_Ptr>) for the left-hand side expression of this constraint
    ROCPPConstraintIF_Ptr mapVars(const map<string, ROCPPExpr_Ptr> &mapFromVarToExpression) const;
    
    /// @note Call LHExpression::mapUncs() for the left-hand side expression of this constraint
    ROCPPConstraintIF_Ptr mapUncs(const map<string, ROCPPExpr_Ptr> &mapFromUncToExpression) const;
    
    /// @note Call LHExpression::replaceTermWithVar() for the left-hand side expression of this constraint
    ROCPPConstraintIF_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr> &term, ROCPPVarIF_Ptr var) const;
    
    /// @note Call LHExpression::replaceBilinearTerm() for the left-hand side expression of this constraint
    ROCPPConstraintIF_Ptr replaceBilinearTerm(map<pair<string,string>, ROCPPVarIF_Ptr> &allTerm, uint &count) const;
    
    /// @note Call LHExpression::mapVars(const map<string,ROCPPVarIF_Ptr>) for the left-hand side expression of this constraint
    ROCPPConstraintIF_Ptr mapVars(const map<string,ROCPPVarIF_Ptr> &mapFromOldToNewVars) const;
    
    /// @note Call LHExpression::mapUnc() for the left-hand side expression of this constraint
    ROCPPConstraintIF_Ptr mapUnc(const map<string,ROCPPUnc_Ptr> &mapFromOldToNewUnc) const;
    
    /// @note Call LHExpression::hasProdsContVars() for the left-hand side expression of this constraint
    void add_vars_involved_in_prod(dvContainer &dvs) const;
    
    /// @note Call LHExpression::add_int_vars() for the left-hand side expression of this constraint
    void add_int_vars(dvContainer &dvs) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// @note Call LHExpression::getNumTimesTermAppears() for the left-hand side expression of this constraint
    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr> &term) const;
    
    /// @note Call LHExpression::getAllProductsOf2Variables() for the left-hand side expression of this constraint
    void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr> > &termMap) const;
    
    ROCPPconstdvContainer_Ptr getDVContainer() const;
    ROCPPconstuncContainer_Ptr getUncContainer() const;
    
    uint getNumContVars() const;
    uint getNumIntVars() const;
    uint getNumBoolVars() const;
    uint getNumAdaptiveContVars() const;
    uint getNumAdaptiveVars() const;
    
    uint getTimeStage() const;
    size_t getNumUncertainties() const;
    
    /// @note Call LHExpression::hasNonlinearities() for the left-hand side expression of this constraint
    bool hasNonlinearities() const;
    
    /// @note Call LHExpression::hasProdsUncertainties() for the left-hand side expression of this constraint
    bool hasProdsUncertainties() const;
    
    /// @note Call LHExpression::hasProdsContVars() for the left-hand side expression of this constraint
    bool hasProdsContVars() const;
    
    /// Return true if the LHExpression::isWellDefined() is true and the right-hand side parameter is set
    bool isWellDefined() const;
    
    bool hasNoDVs() const;
    bool isClassicConstraint() const {return true;}
    virtual bool isEqConstraint() const = 0;
    
    /// @note Call LHExpression::AnyVarIsInvolved() for the left-hand side expression of this constraint
    bool AnyVarIsInvolved(dvContainer& dvs) const;
    
    bool hasNormTerm() const;
    
    bool isLinear() const;
    bool isQuadratic() const;
    
    pair<double,bool> get_rhs() const;
    
    ROCPPExpr_Ptr getLHS() const;
    
    /// Get the part without norm term in the left-hand side expression of this constraint
    /// @note Call LHExpression::getLineatPart()
    ROCPPExpr_Ptr getLinearPart() const;
    
    /// Get the  norm term in the left-hand side expression of this constraint
    /// @note Call LHExpression::getNormTerm()
    ROCPPNormTerm_Ptr getNormTerm() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPConstraintIF_Ptr Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void WriteToStream(ofstream &ofs, uint cnt) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
private:
    /// Left-hand side expression of this constraint
    ROCPPExpr_Ptr m_pLHS;
    
    /// Right-hand side parameter of this constraint
    rhsParams m_rhsParams;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% INEQUALITY CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Inequality constraint
class IneqConstraint : public ClassicConstraintIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of inequality constraint
    /// @param definesUncertaintySet true if and only if this constraint is used to define the uncertainty set
    /// @param isNAC true if and only if this constraint is a non-anticipativity constraint
    IneqConstraint(bool definesUncertaintySet=false, bool isNAC=false) : ClassicConstraintIF(definesUncertaintySet, isNAC){}
    
    /// Destructor of inequality constraint
    ~IneqConstraint(){}
    
    bool isEqConstraint() const {return false;}
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% EQUALITY CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Equality constraint
class EqConstraint : public ClassicConstraintIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of equality constraint
    /// @param definesUncertaintySet true if and only if this constraint is used to define the uncertainty set
    /// @param isNAC true if and only if this constraint is a non-anticipativity constraint
    EqConstraint(bool definesUncertaintySet=false, bool isNAC=false) : ClassicConstraintIF(definesUncertaintySet, isNAC){}
    
    /// Destructor of equality constraint
    ~EqConstraint(){}
    
    bool isEqConstraint() const {return true;}
    
    bool isUseful(double epsilon) const;
};


//
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
////%%%%%%%%%%%%%%%%%%%%%%%%% SOS CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%%%
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
///// SOS constraint
//class SOSConstraint : public ConstraintIF
//{
//public:
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// Constructor of the SOSConstraint class
//    /// @param SOSType denotes the type of the SOS constraint: can be 1 or 2: 1 means at most one of the decision variables can take value other than 0; 2 means at most two of the decision variables can take value other than 0
//    SOSConstraint(uint SOSType);
//
//    /// Destructor of the SOSConstraint class
//    ~SOSConstraint(){}
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// Map from decision variable name to pair of the decision variable and weight associated with the variable in the SOS constraint
//    typedef map<string, pair<ROCPPVarIF_Ptr, double> > SOSMapType;
//
//    /// Constant iterator into SOSMapType
//    typedef SOSMapType::const_iterator sos_iterator;
//
//    /// Return constant iterator pointing to the beginning of the m_sosMap
//    sos_iterator sosBegin() const {return m_sosMap.begin();}
//
//    /// Return constant iterator pointing to the end of the m_sosMap
//    sos_iterator sosEnd() const {return m_sosMap.end();}
//
//    /// Return iterator pointing to the beginning of the dvContainer of this constraint
//    /// @note Call dvContainer::begin() for the dvContainer of this constraint
//     varsIterator varsBegin() const;
//
//    /// Return iterator pointing to the end of the dvContainer of this constraint
//    /// @note Call dvContainer::end() for the dvContainer of this constraint
//    varsIterator varsEnd() const;
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// Added the decision variable dv with associated weight to the constraint
//    /// @param weight is the weight associated with the decision variable in the SOS constraint
//    void add(ROCPPVarIF_Ptr dv, double weight);
//
//    /// @note Call dvContainer::add_int_vars() for the dvContainer of this constraint
//    void add_int_vars(dvContainer &dvs) const;
//
//    /// @note No products of variables in SOSConstraint
//    void add_vars_involved_in_prod(dvContainer &dvs) const {}
//
//    /// @warning Should consider modelling it as a classic constaint if the given expression has more than a single variable
//    ROCPPConstraintIF_Ptr mapVars(const map<string, ROCPPExpr_Ptr> &mapFromVarToExpression) const;
//
//    /// @note Return a copy of the original constraint because there is no uncertainty in the SOS constraint
//    ROCPPConstraintIF_Ptr mapUncs(const map<string, ROCPPExpr_Ptr> &mapFromUncToExpression) const {return this->Clone();}
//
//    /// @note Only replaces the term in the given map if the term is a single variable
//    ROCPPConstraintIF_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr> &term, ROCPPVarIF_Ptr var) const;
//
//    /// @note Returns a copy of the original constraint since there is no uncertain parameters in SOSConstraint
//    ROCPPConstraintIF_Ptr replaceBilinearTerm(map<pair<string,string>, ROCPPVarIF_Ptr> &allTerm, uint &count) const;
//
//    ROCPPConstraintIF_Ptr mapVars(const map<string,ROCPPVarIF_Ptr> &mapFromOldToNewVars) const;
//
//    /// @note Returns a copy of the original constraint since there is no uncertain parameters in SOSConstraint
//    ROCPPConstraintIF_Ptr mapUnc(const map<string,ROCPPUnc_Ptr> &mapFromOldToNewUnc) const {return this->Clone();}
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// Return the type of the SOS constraint
//    /// @return 1 if at most one of the decision variables can take value other than 0, 2 if at most two of the decision variables can take value other than 0
//    uint getSOSType() const {return m_SOSType;}
//
//    /// Return the number of decision variables in this constraint
//    size_t getSize() const {return m_sosMap.size();}
//
//    ROCPPconstdvContainer_Ptr getDVContainer() const;
//    ROCPPconstuncContainer_Ptr getUncContainer() const;
//
//    /// Check whether this constraint has decision variables or not
//    /// @return True if there is no decision variable in this constraint
//    bool hasNoDVs() const;
//
//    /// @return True if there are at least two variables in this constraint
//    bool isWellDefined() const;
//
//    uint getNumContVars() const;
//    uint getNumIntVars() const;
//    uint getNumBoolVars() const;
//    uint getNumAdaptiveContVars() const;
//    uint getNumAdaptiveVars() const;
//    size_t getNumVars() const;
//    size_t getNumUncertainties() const;
//
//    /// Return the maximum time-stage of the decision variables in this constraint
//    uint getTimeStage() const;
//
//    /// @note Always false for SOSConstraint
//     bool hasNonlinearities() const;
//
//     /// @note Always false for SOSConstraint
//     bool hasProdsUncertainties() const;
//
//     /// @note Always false for SOSConstraint
//     bool hasProdsContVars() const;
//
//    /// @note Call dvContainer::AnyVarIsInvolved() for the dvContainer of this constraint
//    bool AnyVarIsInvolved(dvContainer& dvs) const;
//
//    /// @note Only counts the term in the given map if it is a single variable
//    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr> &term) const;
//
//    /// @note There are no products of decision variables in SOSConstraints
//    void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr> > &termMap) const {};
//
//    bool hasNormTerm() const {return false;}
//    bool isSOSConstraint() const {return true;}
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// Create a copy of this constraint
//    ROCPPConstraintIF_Ptr Clone() const;
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    void WriteToStream(ofstream &ofs, uint cnt) const;
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//private:
//
//    /// Type of the SOSContraint
//    uint m_SOSType;
//
//    /// Decision variable container
//    ROCPPdvContainer_Ptr m_pDVContainer;
//
//    /// Map from the variable name to the pair of the variable and its weight
//    SOSMapType m_sosMap;
//};
//
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
////%%%%%%%%%%%%%%%%%%%%%%% IF THEN CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
///// If-then constraint
//class IfThenConstraint : public ConstraintIF
//{
//public:
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// IfThenConstraint constructor
//    /// @param lhs "If" part
//    /// @param rhs "Then" part
//    IfThenConstraint(ROCPPConstraintIF_Ptr lhs, ROCPPConstraintIF_Ptr rhs);
//
//    /// IfThenConstraint destructor
//    ~IfThenConstraint(){}
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// @note Call dvContainer::begin() for the dvContainer of this constraint
//    varsIterator varsBegin() const;
//
//    /// @note Call dvContainer::end() for the dvContainer of this constraint
//    varsIterator varsEnd() const;
//
//    /// Return a constant iterator pointing to the beginning of the m_terms
//    const_iterator begin() const;
//
//    /// Return the end of the m_terms
//    const_iterator end() const;
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// @note Call ClassicConstraintIF::mapVars(const map<string, ROCPPExpr_Ptr>) for both statements in the if-then constraint
//    ROCPPConstraintIF_Ptr mapVars(const map<string, ROCPPExpr_Ptr> &mapFromVarToExpression) const;
//
//    /// @note Call ClassicConstraintIF::mapUncs() for both statements in the if-then constraint
//    ROCPPConstraintIF_Ptr mapUncs(const map<string, ROCPPExpr_Ptr> &mapFromUncToExpression) const;
//
//    /// @note Call ClassicConstraintIF::replaceTermWithVar() for both statements in the if-then constraint
//    ROCPPConstraintIF_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr> &term, ROCPPVarIF_Ptr var) const;
//
//    /// @note Call ClassicConstraintIF::replaceBilinearTerm() for both of two statements in the if-then constraint
//    ROCPPConstraintIF_Ptr replaceBilinearTerm(map<pair<string,string>, ROCPPVarIF_Ptr> &allTerm, uint &count) const;
//
//    /// @note Call ClassicConstraintIF::mapVars(const map<string,ROCPPVarIF_Ptr>) for both of two statements in the if-then constraint
//    ROCPPConstraintIF_Ptr mapVars(const map<string,ROCPPVarIF_Ptr> &mapFromOldToNewVars) const;
//
//    /// @note Call ClassicConstraintIF::mapUnc() for both statements in the if-then constraint
//    ROCPPConstraintIF_Ptr mapUnc(const map<string,ROCPPUnc_Ptr> &mapFromOldToNewUnc) const;
//
//    void add_vars_involved_in_prod(dvContainer &dvs) const { m_lhs->add_vars_involved_in_prod(dvs); m_rhs->add_vars_involved_in_prod(dvs); }
//    void add_int_vars(dvContainer &dvs) const { m_lhs->add_int_vars(dvs); m_rhs->add_int_vars(dvs); }
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// Returns the "If" part of the constraint
//    ROCPPConstraintIF_Ptr get_lhs() const {return m_lhs->Clone();}
//
//    /// Returns the "Then" part of the constraint
//    ROCPPConstraintIF_Ptr get_rhs() const {return m_rhs->Clone();}
//
//    /// @note Calculate the total number of times the term appears in both statements of the if-then constraint
//    /// @note Call ClassicConstraintIF::getNumTimesTermAppears() for both statements in the if-then constraint
//    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr> &term) const {return (m_lhs->getNumTimesTermAppears(term) + m_rhs->getNumTimesTermAppears(term));}
//
//    /// @note Get all products of two variables in both statements in the if-then constraint
//    /// @note Call ClassicConstraintIF::getAllProductsOf2Variables() for both statements in the if-then constraint
//    void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr> > &termMap) const
//    {m_lhs->getAllProductsOf2Variables(freqMap,termMap); m_rhs->getAllProductsOf2Variables(freqMap,termMap);}
//
//
//    ROCPPconstdvContainer_Ptr getDVContainer() const;
//    ROCPPconstuncContainer_Ptr getUncContainer() const;
//
//    uint getNumContVars() const;
//    uint getNumIntVars() const;
//    uint getNumBoolVars() const;
//    uint getNumAdaptiveContVars() const;
//    uint getNumAdaptiveVars() const;
//    size_t getNumVars() const;
//
//    uint getTimeStage() const {return max(m_lhs->getTimeStage(),m_rhs->getTimeStage());}
//    size_t getNumUncertainties() const {return m_pUncContainer->getNumUncertainties();}
//
//    /// Return true if any of the two statements has nonlinearities
//    /// @note Call ClassicConstraintIF::hasNonlinearities() for both statements in the if-then constraint
//    bool hasNonlinearities() const {return ( (m_lhs->hasNonlinearities()) || (m_rhs->hasNonlinearities()) );}
//
//    /// Return true if any of the two statement has a product of uncertain parameters
//    /// @note Call ClassicConstraintIF::hasProdsUncertainties() for both statements in the if-then constraint
//    bool hasProdsUncertainties() const {return ( (m_lhs->hasProdsUncertainties()) || (m_rhs->hasProdsUncertainties()) );}
//
//    /// Return true if any of the two statement has products of continuous variables
//    /// @note Call ClassicConstraintIF::hasProdsContVars() for both statements in the if-then constraint
//    bool hasProdsContVars() const {return ( (m_lhs->hasProdsContVars()) || (m_rhs->hasProdsContVars()) );}
//
//    /// Return true if both of the two statements are well defined
//    /// @note Call ClassicConstraintIF::isWellDefined() for both statements in the if-then constraint
//    bool isWellDefined() const {return ( (m_lhs->isWellDefined()) && (m_rhs->isWellDefined()) );}
//
//    bool isIfThenConstraint() const {return true;}
//
//    bool AnyVarIsInvolved(dvContainer& dvs) const {return ( (m_lhs->AnyVarIsInvolved(dvs)) || (m_rhs->AnyVarIsInvolved(dvs)) );}
//    bool hasNormTerm() const {return ( (m_lhs->hasNormTerm()) || (m_rhs->hasNormTerm()) );}
//    bool hasNoDVs() const;
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    ROCPPConstraintIF_Ptr Clone() const;
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    void WriteToStream(ofstream &ofs, uint cnt) const;
//
//private:
//
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%
//    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    /// If part of constraint
//    ROCPPConstraintIF_Ptr m_lhs;
//
//    /// Else part of constraint
//    ROCPPConstraintIF_Ptr m_rhs;
//
//    /// Decision variable container of this constraint
//    ROCPPdvContainer_Ptr m_pDVContainer;
//
//    /// Uncertain parameter container of this constraint
//    ROCPPuncContainer_Ptr m_pUncContainer;
//
//    /// Vector of terms in this constraint
//    vector<ROCPPCstrTermIF_Ptr> m_terms;
//};

bool DoublesAreEssentiallyEqual(double A, double B, double epsilon);
ROCPPConstraintIF_Ptr createConstraint(ROCPPExpr_Ptr lhs, double rhs, bool isEqual, bool definesUncertaintySet=false, bool isNAC=false);


#endif /* Constraint_hpp */

