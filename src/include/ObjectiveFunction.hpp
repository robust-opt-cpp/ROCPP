//
//  ObjectiveFunction.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef ObjectiveFunction_hpp
#define ObjectiveFunction_hpp

#include "HeaderIncludeFiles.hpp"
#include <stdio.h>


/// Enumerator listing the possible objective function types
enum objType {
    /// Linear objective function
    simpleObj,
    /// Objective function given by the maximum of finitely many linear functions
    maxObj
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% OBJECTIVE FUNCTION INTERFACE %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

///  Objective function interface class
class ObjectiveFunctionIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Constructor of the objective function interface class
    /// @param type type of the objective function (linear or piecewise linear and convex)
    ObjectiveFunctionIF(objType type) : m_type(type){}
    
    /// Destructor of the objective function interface class
    virtual ~ObjectiveFunctionIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Constant iterator to decision variable map
    typedef dvMapType::const_iterator varsIterator;
    
    /// Return a constant iterator pointing to the beginning of the decision variable map
    virtual varsIterator varsBegin() const = 0;
    
    /// Return a constant iterator pointing to the end of the decision variable map
    virtual varsIterator varsEnd() const = 0;
    
    /// Constant iterator to uncertain parameter map
    typedef uncMapType::const_iterator uncsIterator;
    
    /// Return a constant iterator pointing to the beginning of the uncertain parameter map
    virtual uncsIterator uncBegin() const = 0;
    
    /// Return a constant iterator pointing to the end of the uncertain parameter map
    virtual uncsIterator uncEnd() const = 0;
    
    /// Constant iterator to a collection of LHSExpressions defining the objective function
    typedef vector<ROCPPExpr_Ptr>::const_iterator obj_fun_iterator;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Check compatibility for the given expression to be given as (one of the) objective functions
    /// @param pExpression expression of the objective function
    /// @note throws an exception if pExpression is nonlinear (has products of decision variables, products of uncertain parameters, or norm terms)
    void checkCompatibility(ROCPPExpr_Ptr pExpression) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Add the product of the inputs as a new term to all parts of the objective function
    /// @param pVar decision variable in the ProductTerm to add to all parts of the objective function
    /// @param cost cost in the ProductTerm to add to all parts of the objective function
    virtual void add_to_obj(ROCPPVarIF_Ptr pVar, double cost) = 0;
    
    /// Map the decision variables in this objective function to new variables
    virtual ROCPPObjectiveIF_Ptr mapObjVars(const map<string,ROCPPVarIF_Ptr> &mapFromOldToNewVars) const = 0;
    
    /// Map the uncertain parameters in this objective function to new uncertain parameters
    virtual ROCPPObjectiveIF_Ptr mapObjUnc(const map<string,ROCPPUnc_Ptr> &mapFromOldToNewUnc) const = 0;
    
    /// Map the decision variables in this objective function to an expression
    virtual ROCPPObjectiveIF_Ptr mapVars(const map<string, ROCPPExpr_Ptr> &mapFromVarToExpression) const = 0;
    
    /// Map the uncertain parameters in this objective function to an expression
    virtual ROCPPObjectiveIF_Ptr mapUncs(const map<string, ROCPPExpr_Ptr> &mapFromUncToExpression) const = 0;
    
    /// Convert the objective function to constraints using an epigraph reformulation
    /// @param epigraphVar epigraph variable used to write the problem in epigraph form
    /// @param epigraphConstraints will contain all the constraints used to write the objective function in epigraph form
    virtual void convertToEpigraph(ROCPPVarIF_Ptr epigraphVar, vector<ROCPPConstraintIF_Ptr> &epigraphConstraints) const = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Add the decisions variables involved in a product in this objective to the given container dvs
    virtual void add_vars_involved_in_prod(dvContainer &dvs) const = 0;
    
    /// Add the integer variables in this objective to the given container
    virtual void add_int_vars(dvContainer &dvs) const = 0;
    
    /// Get the objective function type (linear or piecewise linear and convex)
    objType getObjType() const{return m_type;}
    
    /// Get the number of terms in the objective function
    /// @return 1 if the objective function is linear (simple objective function); else return the number of terms in the piecewise linear function (max objective)
    virtual size_t getNumTermsMaxObjective() const = 0;
    
    /// Get the vector of all objective functions
    virtual vector<ROCPPExpr_Ptr> getObj() const = 0;
    
    /// Get the i th objective function expression
    /// @param i Number of the objective function
    virtual ROCPPExpr_Ptr getObj(uint i) const = 0;
    
    /// Get the number of times the term given in the multimap appears in this objective function
    /// @param term multimap from the name of the decision variable to the desicion variable used to define this term
    /// @note The same decision variable may appear many times in a term (indicating that we have a product of the same variable)
    virtual uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr> &term) const = 0;
    
    /// Replace in this objective function the term given with the decision variable provided
    /// @param term map including the decision variables in the term to be replaced
    /// @param var variable used to replace the term
    /// @note Only replace the nonlinear term with variable
    virtual ROCPPObjectiveIF_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr> &term, ROCPPVarIF_Ptr var) const = 0;
    
    /// Return true if this objective function is linear
    virtual bool isSimpleObjective() const {return false;}
    
    /// Return true if this objective function is given as the maximum of finitely many linear functions
    virtual bool isMaxObjective() const {return false;}
    
    /// Return true if there is a product between two decision variables in this objective function
    virtual bool hasNonlinearities() const = 0;
    
    /// Return true if there is a product between two uncertain parameters in this objective function
    virtual bool hasProdsUncertainties() const = 0;
    
    /// Return true if there is a product between two continuous variables in this objective function
    virtual bool hasProdsContVars() const = 0;
    
    /// Return true if this objective function does not involve any uncertain parameters
    virtual bool isDeterministic() const = 0;
    
    /// Get the number of adaptive decision variables in this objective function
    virtual uint getNumAdaptiveVars() const = 0;
    
    /// Get the maximum time-stage of any decision variable in the constraint
    virtual uint getTimeStage() const = 0;
    
    /// Check whether the given variable is involved in the specific objective
    virtual bool varIsInvolved(string varName, uint i = 1) const = 0;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual ROCPPObjectiveIF_Ptr Clone() const = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Write the objective function to a stream
    /// @param ofs output file stream
    virtual void WriteToStream(ofstream &ofs) = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
private:
    /// Type of this objective function
    const objType m_type;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% SIMPLE OBJECTIVE %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Linear objective function
class SimpleObjective:public ObjectiveFunctionIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Constructor of the SimpleObjective class
    SimpleObjective();
    
    /// Constructor of the SimpleObjective class
    /// @param objFun expression used to define the objective function
    SimpleObjective(ROCPPExpr_Ptr objFun);
    
    /// Destructor of the SimpleObjective class
    ~SimpleObjective(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @note Call LHExpression::varsBegin() on the expression used to define the objective function
    varsIterator varsBegin() const {return m_pObjFun->varsBegin();}
    
    /// @note Call LHExpression::varsEnd() on the expression used to define the objective function
    varsIterator varsEnd() const {return m_pObjFun->varsEnd();}
    
    /// @note Call LHExpression::uncBegin() on the expression used to define the objective function
    uncsIterator uncBegin() const {return m_pObjFun->uncBegin();}
    
    /// @note Call LHExpression::uncEnd() on the expression used to define the objective function
    uncsIterator uncEnd() const {return m_pObjFun->uncEnd();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    void add_to_obj(ROCPPVarIF_Ptr pVar, double cost);
    
    /// @note Call LHExpression::add_vars_involved_in_prod() on the expression used to define the objective function
    void add_vars_involved_in_prod(dvContainer &dvs) const;
    
    /// @note Call LHExpression::add_int_vars() on the expression used to define the objective function
    void add_int_vars(dvContainer &dvs) const;
    
    ROCPPObjectiveIF_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr> &term, ROCPPVarIF_Ptr var) const;
    
    /// @note Call LHExpression::mapExprVars() on the expression defining the objective function
    ROCPPObjectiveIF_Ptr mapObjVars(const map<string,ROCPPVarIF_Ptr> &mapFromOldToNewVars) const;
    
    /// @note Call LHExpression::mapExprUnc() on the expression defining the objective function
    ROCPPObjectiveIF_Ptr mapObjUnc(const map<string,ROCPPUnc_Ptr> &mapFromOldToNewUnc) const;
    
    /// @note Call LHExpression::mapVars() for the expression defining the objective function
    ROCPPObjectiveIF_Ptr mapVars(const map<string, ROCPPExpr_Ptr> &mapFromVarToExpression) const;
    
    /// @note Call LHExpression::mapUncs() for the expression defining the objective function
    ROCPPObjectiveIF_Ptr mapUncs(const map<string, ROCPPExpr_Ptr> &mapFromUncToExpression) const;
    
    void convertToEpigraph(ROCPPVarIF_Ptr epigraphVar, vector<ROCPPConstraintIF_Ptr> &epigraphConstraints) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    vector<ROCPPExpr_Ptr> getObj() const;
    
    ROCPPExpr_Ptr getSimpleObj() const {return m_pObjFun;};
    
    /// @warning The input parameter for SimpleObjective must be 1
    ROCPPExpr_Ptr getObj(uint i) const;
    
    /// @note Return 1 for SimpleObjective
    size_t getNumTermsMaxObjective() const {return 1;}
    
    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr> &term) const;
    
    /// @note Return true for SimpleObjective
    bool isSimpleObjective() const {return true;}
    
    /// @note Call LHExpression::hasNonlinearities() on the expression defining the objective function
    bool hasNonlinearities() const;
    
    /// @note Call LHExpression::hasProdsUncertainties() on the expression defining the objective function
    bool hasProdsUncertainties() const;
    
    /// @note Call LHExpression::hasProdsContVars() on the expression defining the objective function
    bool hasProdsContVars() const;
    
    bool isDeterministic() const;
    uint getNumAdaptiveVars() const;
    uint getTimeStage() const;
    
    /// @warning The input parameter for SimpleObjective must be 1
    bool varIsInvolved(string varName, uint i = 1) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPObjectiveIF_Ptr Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    void WriteToStream(ofstream &ofs);
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Expression used to define the objective function
    ROCPPExpr_Ptr m_pObjFun;
};

 
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% MAX OBJECTIVE %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Piecewise linear convex objective function
class MaxObjective: public ObjectiveFunctionIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Constructor of the MaxObjective class
    MaxObjective();
    
    /// Constructor of the MaxObjective class
    MaxObjective(vector<ROCPPExpr_Ptr> objFuns);
    
    /// Destructor of the MaxObjective class
    ~MaxObjective(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @note Call dvContainer::begin() on the dvContainer of this objective
    varsIterator varsBegin() const;
    
    /// @note Call dvContainer::end() on the dvContainer of this objective
    varsIterator varsEnd() const;
    
    /// @note Call uncContainer::begin() on the uncContainer of this objective
    uncsIterator uncBegin() const;
    
    /// @note Call uncContainer::end() on the uncContainer of this objective
    uncsIterator uncEnd() const;
    
    /// Constant iterator on the parts of the objective function
    typedef vector<ROCPPExpr_Ptr>::const_iterator obj_iterator;
    
    /// Return a constant iterator pointing to the beginning of the vector of objective functions
    obj_iterator objectiveBegin() const {return m_pObjFuns.begin();}
    
    /// Return a constant iterator pointing to the end of the vector of objective functions
    obj_iterator objectiveEnd() const {return m_pObjFuns.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    void add_to_obj(ROCPPVarIF_Ptr pVar, double cost);
    
    /// @note Call dvContainter::add_vars_involved_in_prod() on the dvContainer of this objective
    virtual void add_vars_involved_in_prod(dvContainer &dvs) const;
    
    /// @note Call dvContainter::add_int_vars() on the dvContainer of this objective
    virtual void add_int_vars(dvContainer &dvs) const;
    
    /// @note Call LHExpression::mapExprVars() on every expression of the objective function
    ROCPPObjectiveIF_Ptr mapObjVars(const map<string,ROCPPVarIF_Ptr> &mapFromOldToNewVars) const;
    
    /// @note Call LHExpression::mapExprUnc() on every expression of the objective function
    ROCPPObjectiveIF_Ptr mapObjUnc(const map<string,ROCPPUnc_Ptr> &mapFromOldToNewUnc) const;
    
    /// @note Call LHExpression::mapVars() on every expression of the objective function
    ROCPPObjectiveIF_Ptr mapVars(const map<string, ROCPPExpr_Ptr> &mapFromVarToExpression) const;
    
    /// @note Call LHExpression::mapUncs() on every expression of the objective function
    ROCPPObjectiveIF_Ptr mapUncs(const map<string, ROCPPExpr_Ptr> &mapFromUncToExpression) const;
    
    ROCPPObjectiveIF_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr> &term, ROCPPVarIF_Ptr var) const;
    
    void convertToEpigraph(ROCPPVarIF_Ptr epigraphVar, vector<ROCPPConstraintIF_Ptr> &epigraphConstraints) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    size_t getNumTermsMaxObjective() const {return m_pObjFuns.size();}
    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr> &term) const;
    vector<ROCPPExpr_Ptr> getObj() const;
    ROCPPExpr_Ptr getObj(uint i) const;
    
    bool isMaxObjective() const {return true;}
    bool hasNonlinearities() const;
    bool hasProdsUncertainties() const;
    bool hasProdsContVars() const;
    bool isDeterministic() const;
    uint getNumAdaptiveVars() const;
    uint getTimeStage() const;
    
    bool varIsInvolved(string varName, uint i = 1) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPObjectiveIF_Ptr Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    void WriteToStream(ofstream &ofs);
    
private:
    /// Vector of the objective expressions
    vector<ROCPPExpr_Ptr> m_pObjFuns;
    
    /// Decision variable container
    ROCPPdvContainer_Ptr m_dvContainer;
    
    /// Uncertain parameter container
    ROCPPuncContainer_Ptr m_uncContainer;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% CREATE OBJ FUNCTION %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPObjectiveIF_Ptr createObjective(ROCPPExpr_Ptr objFun);
ROCPPObjectiveIF_Ptr creatMaxObjective(vector<ROCPPExpr_Ptr> objFuns);


#endif /* ObjectiveFunction_hpp */
