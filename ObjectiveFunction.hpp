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

class ConstraintIF;
class UncertaintyIF;
class DecisionVariableIF;
class dvContainer;
class uncContainer;
class LHSExpression;


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
    typedef vector<boost::shared_ptr<LHSExpression> >::const_iterator obj_fun_iterator;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Check compatibility for the given expression to be given as (one of the) objective functions
    /// @param pExpression expression of the objective function
    /// @note throws an exception if pExpression is nonlinear (has products of decision variables, products of uncertain parameters, or norm terms)
    void checkCompatibility(boost::shared_ptr<LHSExpression> pExpression) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Add the product of the inputs as a new term to all parts of the objective function
    /// @param pVar decision variable in the ProductTerm to add to all parts of the objective function
    /// @param cost cost in the ProductTerm to add to all parts of the objective function
    virtual void add_to_obj(boost::shared_ptr<DecisionVariableIF> pVar, double cost) = 0;
    
    /// Map the decision variables in this objective function to new variables
    virtual boost::shared_ptr<ObjectiveFunctionIF> mapObjVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const = 0;
    
    /// Map the uncertain parameters in this objective function to new uncertain parameters
    virtual boost::shared_ptr<ObjectiveFunctionIF> mapObjUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const = 0;
    
    /// Map the decision variables in this objective function to an expression
    virtual boost::shared_ptr<ObjectiveFunctionIF> mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const = 0;
    
    /// Map the uncertain parameters in this objective function to an expression
    virtual boost::shared_ptr<ObjectiveFunctionIF> mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const = 0;
    
    /// Convert the objective function to constraints using an epigraph reformulation
    /// @param epigraphVar epigraph variable used to write the problem in epigraph form
    /// @param epigraphConstraints will contain all the constraints used to write the objective function in epigraph form
    virtual void convertToEpigraph(boost::shared_ptr<DecisionVariableIF> epigraphVar, vector<boost::shared_ptr<ConstraintIF> > &epigraphConstraints) const = 0;
    
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
    virtual vector<boost::shared_ptr<LHSExpression> > getObj() const = 0;
    
    /// Get the i th objective function expression
    /// @param i Number of the objective function
    virtual boost::shared_ptr<LHSExpression> getObj(uint i) const = 0;
    
    /// Get the number of times the term given in the multimap appears in this objective function
    /// @param term multimap from the name of the decision variable to the desicion variable used to define this term
    /// @note The same decision variable may appear many times in a term (indicating that we have a product of the same variable)
    uint getNumTimesTermAppears(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term) const;
    
    /// Replace in this objective function the term given with the decision variable provided
    /// @param term map including the decision variables in the term to be replaced
    /// @param var variable used to replace the term
    /// @note Only replace the nonlinear term with variable
    virtual boost::shared_ptr<ObjectiveFunctionIF> replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const = 0;
    
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
    
    virtual boost::shared_ptr<ObjectiveFunctionIF> Clone() const = 0;
    
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
    SimpleObjective(boost::shared_ptr<LHSExpression> objFun);
    
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

    void add_to_obj(boost::shared_ptr<DecisionVariableIF> pVar, double cost);
    
    /// @note Call LHExpression::add_vars_involved_in_prod() on the expression used to define the objective function
    void add_vars_involved_in_prod(dvContainer &dvs) const;
    
    /// @note Call LHExpression::add_int_vars() on the expression used to define the objective function
    void add_int_vars(dvContainer &dvs) const;
    
    boost::shared_ptr<ObjectiveFunctionIF> replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const;
    
    /// @note Call LHExpression::mapExprVars() on the expression defining the objective function
    boost::shared_ptr<ObjectiveFunctionIF> mapObjVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const;
    
    /// @note Call LHExpression::mapExprUnc() on the expression defining the objective function
    boost::shared_ptr<ObjectiveFunctionIF> mapObjUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const;
    
    /// @note Call LHExpression::mapVars() for the expression defining the objective function
    boost::shared_ptr<ObjectiveFunctionIF> mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const;
    
    /// @note Call LHExpression::mapUncs() for the expression defining the objective function
    boost::shared_ptr<ObjectiveFunctionIF> mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const;
    
    void convertToEpigraph(boost::shared_ptr<DecisionVariableIF> epigraphVar, vector<boost::shared_ptr<ConstraintIF> > &epigraphConstraints) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    vector<boost::shared_ptr<LHSExpression> > getObj() const;
    
    /// @warning The input parameter for SimpleObjective must be 1
    boost::shared_ptr<LHSExpression> getObj(uint i) const;
    
    /// @note Return 1 for SimpleObjective
    size_t getNumTermsMaxObjective() const {return 1;}
    
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
    
    boost::shared_ptr<ObjectiveFunctionIF> Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    void WriteToStream(ofstream &ofs);
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Expression used to define the objective function
    boost::shared_ptr<LHSExpression> m_pObjFun;
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
    MaxObjective(vector<boost::shared_ptr<LHSExpression> > objFuns);
    
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
    typedef vector<boost::shared_ptr<LHSExpression> >::const_iterator obj_iterator;
    
    /// Return a constant iterator pointing to the beginning of the vector of objective functions
    obj_iterator objectiveBegin() const {return m_pObjFuns.begin();}
    
    /// Return a constant iterator pointing to the end of the vector of objective functions
    obj_iterator objectiveEnd() const {return m_pObjFuns.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    void add_to_obj(boost::shared_ptr<DecisionVariableIF> pVar, double cost);
    
    /// @note Call dvContainter::add_vars_involved_in_prod() on the dvContainer of this objective
    virtual void add_vars_involved_in_prod(dvContainer &dvs) const;
    
    /// @note Call dvContainter::add_int_vars() on the dvContainer of this objective
    virtual void add_int_vars(dvContainer &dvs) const;
    
    /// @note Call LHExpression::mapExprVars() on every expression of the objective function
    boost::shared_ptr<ObjectiveFunctionIF> mapObjVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const;
    
    /// @note Call LHExpression::mapExprUnc() on every expression of the objective function
    boost::shared_ptr<ObjectiveFunctionIF> mapObjUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const;
    
    /// @note Call LHExpression::mapVars() on every expression of the objective function
    boost::shared_ptr<ObjectiveFunctionIF> mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const;
    
    /// @note Call LHExpression::mapUncs() on every expression of the objective function
    boost::shared_ptr<ObjectiveFunctionIF> mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const;
    
    boost::shared_ptr<ObjectiveFunctionIF> replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const;
    
    void convertToEpigraph(boost::shared_ptr<DecisionVariableIF> epigraphVar, vector<boost::shared_ptr<ConstraintIF> > &epigraphConstraints) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    size_t getNumTermsMaxObjective() const {return m_pObjFuns.size();}
    vector<boost::shared_ptr<LHSExpression> > getObj() const;
    boost::shared_ptr<LHSExpression> getObj(uint i) const;
    
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
    
    boost::shared_ptr<ObjectiveFunctionIF> Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    void WriteToStream(ofstream &ofs);
    
private:
    /// Vector of the objective expressions
    vector<boost::shared_ptr<LHSExpression> > m_pObjFuns;
    
    /// Decision variable container
    boost::shared_ptr<dvContainer> m_dvContainer;
    
    /// Uncertain parameter container
    boost::shared_ptr<uncContainer> m_uncContainer;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% CREATE VAR FUNCTION %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<ObjectiveFunctionIF> createObjective(boost::shared_ptr<LHSExpression> objFun);
boost::shared_ptr<ObjectiveFunctionIF> creatMaxObjective(vector<boost::shared_ptr<LHSExpression> > objFuns);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% OBJECTIVE TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


typedef ObjectiveFunctionIF ROCPPObjectiveIF;
typedef boost::shared_ptr<ROCPPObjectiveIF> ROCPPObjectiveIF_Ptr;

typedef SimpleObjective ROCPPSimpleObjective;
typedef boost::shared_ptr<ROCPPSimpleObjective> ROCPPSimpleObjective_Ptr;

typedef MaxObjective ROCPPMaxObjective;
typedef boost::shared_ptr<ROCPPMaxObjective> ROCPPMaxObjective_Ptr;


#endif /* ObjectiveFunction_hpp */
