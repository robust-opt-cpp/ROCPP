//
//  OptimizationModel.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef OptimizationModel_hpp
#define OptimizationModel_hpp

#include "HeaderIncludeFiles.hpp"
#include <stdio.h>
#include <vector>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% PROBLEM ENUMERATED TYPE %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Enumerator listing the possible optimization problem types
enum problemType{
    /// Uncertain problem
    uncertainType,
    /// Uncertain problem with decision-dependent information discovery
    dduType,
    /// Uncertain single-stage problem
    uncertainssType,
    /// Uncertain multi-stage problem
    uncertainmsType,
    /// MISOCP problem
    misocpType,
    /// Bilinear MISOCP problem
    bmisocpType,
    /// Deterministic problem
    deterministicType,
    /// CPLEX MISOCP problem
    cplexmisocpType
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% OBJECTIVE ENUMERATED TYPE %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Enumerator listing the possible types of objectives (worst-case or expectation)
enum uncOptModelObjType {
    /// Worst-case objective (robust)
    robust,
    /// Expectation objective (stochastic)
    stochastic
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% MEASUREMENT VARIABLE PAIR STRUCT %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Structure containing an uncertain parameter whose time of revelation is decision-dependent and its associated measurement variable
struct measPair
{
    /// Constructor of the measPair structure
    measPair(ROCPPUnc_Ptr ddu, ROCPPVarIF_Ptr var) : m_measVar(var), m_ddu(ddu) {}
    
    /// Destructor of the measPair structure
    ~measPair(){}
    
    /// Measurement variable
    ROCPPVarIF_Ptr m_measVar;
    
    /// Uncertain parameter whose time of revelation is decision-dependent
    ROCPPUnc_Ptr m_ddu;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% OPTIMIZATIONMODEL INTERFACE %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Optimization model interface class
class OptimizationModelIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of OptimizationModelIF
    /// @param numTimeStages Planning horizon of this model: maximum time when any decision is made or any uncertain parameter is revealed
    OptimizationModelIF(uint numTimeStages=1);
    
    /// Destructor of OptimizationModelIF
    virtual ~OptimizationModelIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator for optimization model constraints
    typedef vector<ROCPPConstraint_Ptr>::const_iterator constraintIterator;
    
    /// Constant iterator for optimization model decision variable map
    typedef dvMapType::const_iterator varsIterator;
    
    /// Constant iterator for optimization model uncertain parameter map
    typedef map<string, ROCPPUnc_Ptr>::const_iterator uncertaintiesIterator;
    
    /// Return a constant iterator pointing to the beginning of the constraint vector
    constraintIterator constraintBegin() const {return m_constraints.begin();}
    
    /// Return a constant iterator pointing to the end of the constraint vector
    constraintIterator constraintEnd() const {return m_constraints.end();}
    
    /// Return a constant iterator pointing to the beginning of the decision variable map (m_pDVContainer)
    varsIterator varsBegin() const;
    
    /// Return a constant iterator pointing to the end of the decision variable map (m_pDVContainer)
    varsIterator varsEnd() const;
    
    /// Return a constant iterator pointing to the beginning of the uncertain parameter map (m_pUncContainer)
    /// @warning Not valid in deterministic model
    virtual uncertaintiesIterator uncertaintiesBegin() const;
    
    /// Return a constant iterator pointing to the end of the uncertain parameter map (m_pUncContainer)
    /// @warning Not valid in deterministic model
    virtual uncertaintiesIterator uncertaintiesEnd() const;
    
    /// Return a constant iterator pointing to the begining of the block map
    map<string, vector<ROCPPConstraint_Ptr> >::const_iterator blockMapBegin() const {return m_mapBlockConstraints.begin();}
    
    /// Return a constant iterator pointing to the end of the block map
    map<string, vector<ROCPPConstraint_Ptr> >::const_iterator blockMapEnd() const {return m_mapBlockConstraints.end();}
    
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Check compatibility for the given constraint of this optimization model
    virtual void checkCompatibility(ROCPPConstraint_Ptr pConstraint) const;
    
    /// Check compatibility for the given decision variable of this optimization model
    virtual void checkCompatibility(ROCPPVarIF_Ptr pVariable) const;
    
    /// Check compatibility for the given objective function of this optimization model
    /// @warning Cannot add objective function which contains products of continuous variables or products of uncertianties or has a larger time stage that the planning horizon into the optimization model
    virtual void checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Add a constraint into the optimization model after setting the constraint attributes
    /// @note We first map all the variables and uncertainties to make sure they are not used in any other model and then add the new constraint to the model
    /// @see ConstraintIF::mapVars(const map<string,ROCPPVarIF_Ptr>), ConstraintIF::mapUnc(), OptimizationModelIF::createVarMap(ROCPPConstraint_Ptr), OptimizationModelIF::createUncMap(ROCPPConstraint_Ptr), OptimizationModelIF::push_constraint()
    /// blockNme allows to add a constraint
    void add_constraint(ROCPPConstraint_Ptr pConstraint, string blockNme = "main");
    
    /// Add a soc constraint into the optimization model
    /// @param coneHead Cone head decision variable
    /// @param otherVars Other variables
    void add_soc_constraint(ROCPPVarIF_Ptr coneHead, const vector<ROCPPVarIF_Ptr> &otherVars, string blockNme = "main");
    
    /// Add constraints into the optimization model
    void add_constraints(vector<ROCPPConstraint_Ptr>::const_iterator first, vector<ROCPPConstraint_Ptr>::const_iterator last, string blockNme = "main");
    
    /// Add a constraint defining the uncertainty set into the model
    void add_constraint_uncset(ROCPPConstraint_Ptr pUncCstr, string blockNme = "main");
    
    /// Add an epipragh variable as the objective function and an epigraph constraint into this model
    void add_epigraph();

    virtual void add_ddu(ROCPPUnc_Ptr pUncertainty, uint firstTimeStageObservable, uint lastTimeStageObservable, const map<uint, double> &obsCosts);
    
    /// Set the given objective as the objective function of the model
    /// @note We first map all the variables and uncertainties to make sure they are not used in any other model and then set the new objective
    /// @see ConstraintIF::mapVars(const map<string,ROCPPVarIF_Ptr>), ConstraintIF::mapUnc(), OptimizationModelIF::createVarMap(ROCPPConstraint_Ptr), OptimizationModelIF::createUncMap(ROCPPConstraint_Ptr)
    virtual void set_objective(ROCPPObjectiveIF_Ptr pObj);
    
    /// Create an objective function using the given expression and add it to the optimization model
    /// @see Optimization::set_objective(ROCPPObjectiveIF_Ptr)
    void set_objective(ROCPPExpr_Ptr objFun);
    
    /// Create an objective function using the given expressions and add it to the optimization model
    /// @see Optimization::set_objective(ROCPPObjectiveIF_Ptr)
    void set_objective(vector<ROCPPExpr_Ptr> objFuns);
    
    /// Replace the given term in this model with the given decision variable
    /// @see ObjectiveFunctionIF::replaceTermWithVar(), ConstraintIF::replaceTermWithVar()
    ROCPPOptModelIF_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr> &term, ROCPPVarIF_Ptr var) const;
    
    /// Copy the information in the given model and set it to this one, make sure the information of ddu is kept after clone or reformulation
    virtual void set_ddu(ROCPPOptModelIF_Ptr pIn);
    
    /// Copy the information in the given maps and set it to this one, make sure the information of ddu is kept after clone or reformulation
    virtual void set_ddu(const map<pair<string,uint>, measPair> &dduToMeasMap, const map< string, pair<uint,uint> > &dduStagesObs);
    
    virtual void set_objType(uncOptModelObjType pType);
    
    /// Create the variable map for the given constraint
    /// Iterate through all variables in the given constraint. If a variable with the same name exists in the model, use the old one. If a variables with the same name does not exist, create a new variable and add it into the model.
    /// @return A map from the variable name to the variable
    /// @see OptimizationModelIF::varIsDefined(), OptimizationModelIF::add_var()
    map<string, ROCPPVarIF_Ptr> createVarMap(ROCPPConstraint_Ptr pConstraint);
    
    /// Create the variable map for the given objective function
    /// Iterate through all variables in the given objective function. If a variable with the same name exists in the model, use the old one. If a variables with the same name does not exist, create a new variable and add it into the model.
    /// @return A map from the variable name to the variable
    /// @see OptimizationModelIF::varIsDefined(), OptimizationModelIF::add_var()
    map<string, ROCPPVarIF_Ptr> createVarMap(ROCPPObjectiveIF_Ptr objFun);
    
    /// @note Only valid in uncertain model
    virtual map<string, ROCPPUnc_Ptr> createUncMap(ROCPPConstraint_Ptr pConstraint);
    
    /// @note Only valid in uncertain model
    virtual map<string, ROCPPUnc_Ptr> createUncMap(ROCPPObjectiveIF_Ptr objFun);
    
    virtual void pair_uncertainties(ROCPPUnc_Ptr u1, ROCPPUnc_Ptr u2);
    
    /// Convert the objective function to its expectation
    virtual void getExpectation();
    
    virtual size_t getNumUncertainties() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return true if and only if the given uncertainty has a time of revelation that is decision-dependent
    virtual bool isDDU(string uncName) const {return false;}
    
    /// Return true if and only if the given uncertainty is observable
    virtual bool isObservable(string uncName) const;
    
    /// Return true if and only if the variable with the given name is adaptive
    /// @see OpimizationModelIF::getVar(), DecisionVariableIf::isAdaptive()
    bool isAdaptive(string varName) const;
    
    /// Return true if and only if this model does not include a norm term
    bool isInStandardForm() const;
    
    /// Return true if and only if this model is uncertain
    virtual bool isUncertainOptimizationModel() const {return false;}
    
    /// Return true if and only if this model is a ddu model
    virtual bool isMultiStageOptModelDDID() const {return false;}
    
    /// Return true if and only if the decision variable exists in this model
    /// @note Call dvContainer::find() for the dvContainer of this model
    bool varIsDefined(string varName) const;
    
    ///  Return true if and only if the decision variable exists in the i th objective function of the max term
    /// @note Call ObjectiveFunctionIF::varIsDefined()
    bool VarInObj(string varName, uint i = 1) const;
    
    /// Return the problem type
    virtual problemType getType() const = 0;
    
    /// Return the measurement variable for the given uncertainty and the time stage
    virtual ROCPPVarIF_Ptr getMeasVar(string dduncName, uint timeStage) const;
    
    /// Return the variable called varName
    ROCPPVarIF_Ptr getVar(string varName) const;
    
    /// Return the uncertain parameter called uncName
    virtual ROCPPUnc_Ptr getUnc(string uncName) const;
    
    /// Return the number of times the given term in the map appears in this model
    /// @see ObjectiveFunctionIF::getNumTimesTermAppears(), ConstraintIF::getNumTimesTermAppears()
    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr> &term) const;
    
    /// Return all products of two variables in this model
    /// @note There are no nonlinearities in the objective function
    /// @see ConstraintIF::getAllProductsOf2Variables()
    void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map<pair<string,string>, multimap<string, ROCPPVarIF_Ptr> > &termMap) const;
    
    /// Return a pointer pointing to the objective function of this problem
    ROCPPObjectiveIF_Ptr getObj() const {return m_pObj;}
    
    /// Return a map from decision dependent uncertain parameters to associated measurement variables
    /// @return the first element of the map is a pair of uncertainty name and time stage; the second element is the measurement structure corresponding to this pair. If there is no such parameter, return an empty map.
    virtual map<pair<string,uint>, measPair> getDDUToMeasMap() const;
    
    /// Return a map from decision dependent uncertain parameters to the stages when the parameter can be observed
    /// @return the first element of the map is the uncertainty name; the second element is a pair with the first and the last stages when the parameter is observable. If there is no such parameter, return an empty map.
    virtual map<string, pair<uint,uint> > getdduStagesObs() const;
    
    /// Return the first time stage when uncertainty called uncName can be observed
    /// @note Not valid in deterministic model
    virtual uint getFirstStageObservable(string uncName) const;
    
    /// Return the last time stage when uncertainty called uncName can be observed
    /// @note Not valid in deterministic model
    virtual uint getLastStageObservable(string uncName) const;
    
    /// Return the decision variable container (m_dvContainer) of this object
    ROCPPconstdvContainer_Ptr getDVContainer() const {return m_pDVContainer; }
    
    /// Return the uncertainty container (m_uncContainer) of this object
    /// @warning Not valid in deterministic model
    virtual ROCPPuncContainer_Ptr getUncContainer() const;
    
    /// Return the objective type
    virtual uncOptModelObjType getObjType() const;
    
    /// Return the number of constraints in the model
    size_t getNumConstraints() const {return m_constraints.size();}
    
    /// Return the time stage of the model
    uint getNumTimeStages() const {return m_numTimeStages;};
    
    /// Return the number of real-valued decision variables in this term
    uint getNumContVars() const;
    
    /// Return the number of integer decision variables in this term
    uint getNumIntVars() const;
    
    /// Return the number of boolen decision variables in this term
    uint getNumBoolVars() const;
    
    /// Return the number of adaptive real-valued decision variables in this term
    uint getNumAdaptiveContVars() const;
    
    /// Return the number of adaptive decision variables in this term
    uint getNumAdaptiveVars() const;
    
    /// Return the number of decision variables in this term
    size_t getNumVars() const;
    
    /// Return true if and only if there is a product between two decision variables in this optimization model
    bool hasNonlinearities() const;
    
    /// Return true if the problem has rectangular uncertainty set
    virtual bool hasRectangularUncertaintySet() const;
    
    virtual bool hasRealVarsInUncertaintySet() const {throw MyException("Problems that are not uncertain do not have an uncertainty set");}
    
    virtual bool hasDecisionDependentUncertaintySet() const {throw MyException("Problems that are not uncertain do not have an uncertainty set");}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Clone this problem
    virtual ROCPPOptModelIF_Ptr Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual void WriteToFile(string folderName, string fileName) const;
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Decision variable container of this model
    ROCPPdvContainer_Ptr m_pDVContainer;
    
    /// Vector of constraints in this model
    vector<ROCPPConstraint_Ptr> m_constraints;
    
    /// Objective function of this model
    ROCPPObjectiveIF_Ptr m_pObj;
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Protected Functions %%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Add the given variable into the dvContainer of this model
    virtual void add_var(ROCPPVarIF_Ptr pVariable);
    
    /// Add the variable in the given container into the dvContainer of this model
    virtual void add_vars(ROCPPconstdvContainer_Ptr pDVcontainer);
    
    /// Add the information discovery cost to the objective model
    /// @note Used in DDUOptimizationModel
    void add_ddu_obj(ROCPPVarIF_Ptr pVar, double cost);
    
    /// Push the constraint back into the vector of constraints of this model
    void push_constraint(ROCPPConstraint_Ptr pConstraint);
    
    /// Get index of the given constraint in the m_mapCstrIdxToUncertaintySet map
    ptrdiff_t getConstraintIdx(constraintIterator pConstraintIt) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Length of planning horizon of this model
    uint m_numTimeStages;
    
    /// Vector collecting the uncertainty set constraints of this model
    vector<ROCPPConstraint_Ptr> m_uncertaintySet;
    
    /// Uncertain parameter container of this model
    ROCPPuncContainer_Ptr m_pUncContainer;
    
    /// Container of decision dependent uncertain parameters in this model
    ROCPPuncContainer_Ptr m_dduContainer;
    
    /// Container of non decision dependent uncertain parameters in this model
    ROCPPuncContainer_Ptr m_nondduContainer;
    
    /// Map from measurement varaibles to measurement pairs
    map<string, measPair> m_measVars;
    
    /// Map from pair of uncertain parameter name and time stage to measurement pair
    map< pair<string,uint>, measPair> m_dduToMeasMap;
    
    /// Map from name of the uncertain parameter to the first and last times when it can be observed
    map<string, pair<uint,uint> > m_dduStagesObs;
    
    /// Map from block name to vector of problem constraint indices
    map<string, vector<ROCPPConstraint_Ptr> > m_mapBlockConstraints;
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%% Deterministic Optimization Model %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Deterministic optimization model
class DeterministicOptimizationModel : public OptimizationModelIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the DeterministicOptimizationModel class
    DeterministicOptimizationModel(){}
    
    /// Destructor of the DeterministicOptimizationModel class
    ~DeterministicOptimizationModel(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @warning Cannot add uncertain constraint into a deterministic model
    void checkCompatibility(ROCPPConstraint_Ptr pConstraint) const;
    
    /// @warning Cannot add adaptive variable into a deterministic model
    void checkCompatibility(ROCPPVarIF_Ptr pVariable) const;
    
    /// @warning Cannot add uncertain objective function into a deterministic model
    void checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    problemType getType() const {return deterministicType;}
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% Uncertain Optimization Model %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Uncertain optimization model
class UncertainOptimizationModel : public OptimizationModelIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the UncertainOptimizationModel class
    /// @param numTimeStages Planning horizon of this model
    UncertainOptimizationModel(uint numTimeStages = 1, uncOptModelObjType objType = robust);
    
    /// Destructor of the UncertainOptimizationModel class
    virtual ~UncertainOptimizationModel(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Const iterator in the uncertainty set constraints of this model
    typedef vector<ROCPPConstraint_Ptr>::const_iterator uncertaintySetIterator;
    
    /// Return a constant iterator pointing to the beginning of the uncertain parameter container
    uncertaintiesIterator uncertaintiesBegin() const;
    
    /// Return a constant iterator pointing to the end of the uncertain parameter container
    uncertaintiesIterator uncertaintiesEnd() const;
    
    /// Return a constant iterator pointing to the beginning of the uncertainty set
    uncertaintySetIterator uncertaintySetBegin() const {return m_uncertaintySet.begin();}
    
    /// Return a constant iterator pointing to the end of the uncertainty set
    uncertaintySetIterator uncertaintySetEnd() const {return m_uncertaintySet.end();}
    
    /// Return a constant iterator pointing to the beginning of the uncertainty set for this constraint
    /// if the constraint does not have its own uncertainty set, return a pointer to the beginning of the overall uncertainty set
    uncertaintySetIterator uncertaintySetBegin(string blockNme) const;
    
    /// Return a constant iterator pointing to the end of the uncertainty set for this constraint
    /// if the constraint does not have its own uncertainty set, return a pointer to the end of the overall uncertainty set
    uncertaintySetIterator uncertaintySetEnd(string blockNme) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @warning Cannot add constraint which has norm term but does not define uncertainty set into the model
    virtual void checkCompatibility(ROCPPConstraint_Ptr pConstraint) const;
    virtual void checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Create the uncertain parameter map for the given constraint
    /// Iterate through all uncertainties in the given constraint. If an uncertain parameter with the same name exists in the model, use the old one. If no parameter with the same name exists, create a new one and add it to the model.
    virtual map<string, ROCPPUnc_Ptr> createUncMap(ROCPPConstraint_Ptr pConstraint);
    
    /// Create the uncertainty map for the given objective function
    /// Iterate through all uncertainties in the given objective function. If an uncertain parameter with the same name exists in the model, use the old one. If no parameter with the same name exists, create a new one and add it to the model.
    map<string, ROCPPUnc_Ptr> createUncMap(ROCPPObjectiveIF_Ptr objFun);
    
    void set_objType(uncOptModelObjType pType) {m_objType = pType;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void getExpectation();
    
    /// Return the number of the uncertain parameters in this model
    size_t getNumUncertainties() const;
    
    /// Return the location of the given uncertain parameter in the uncContainer of this model
    uint getAlphabeticalLocation(ROCPPUnc_Ptr pUncertainty) const;
    
    /// Return the location of the given uncertain parameter in the container of the observable uncertainties of this model
    uint getObservableAlphabeticalLocation(ROCPPUnc_Ptr pUncertainty) const;
    
    /// Return 1 for non-ddu model
    virtual uint getFirstStageObservable(string uncName) const;
    
    /// Return the time stage for non-ddu model
    virtual uint getLastStageObservable(string uncName) const;
    
    /// Return the variables affecting the uncertainty set in the dvContainer
    void getVarsAffectingUncSet(dvContainer& dvs);
    
    /// Return the number of uncertainty set constraints
    size_t getNumUncertaintySetConstraints() const {return m_uncertaintySet.size();}
    
    /// Return the number of problem constraints (constraints that do not define the uncertainty set)
    size_t getNumProblemConstraints() const {return getNumConstraints() - getNumUncertaintySetConstraints();}
    
    /// Return the number of observable uncertain parameters
    size_t getNumObsUncertainties() const;
    
    /// Return true if and only if the uncertain parameter is defined in this model
    bool uncIsDefined(string uncName) const;
    
    bool isUncertainOptimizationModel() const {return true;}
    bool isObservable(string uncName) const;
    bool hasRectangularUncertaintySet() const;
    
    bool hasRealVarsInUncertaintySet() const;
    
    bool hasDecisionDependentUncertaintySet() const;
    
    virtual problemType getType() const {return uncertainType;}
    uncOptModelObjType getObjType() const {return m_objType;}
    ROCPPUnc_Ptr getUnc(string uncName) const;
    ROCPPuncContainer_Ptr getUncContainer() const {return m_pUncContainer; }
    ROCPPuncContainer_Ptr getObsUncContainer() const;
    
    //ROCPPOptModelIF_Ptr Clone() const;
    
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Protected Functions %%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Add uncertainty to the m_pUncContainer of this modeel
    virtual void add_uncertainty( ROCPPUnc_Ptr pUnc);
    
    /// Add uncertainties to the m_pUncContainer of this model
    void add_uncertainties(ROCPPconstuncContainer_Ptr pUncContainer);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Uncertain objective type of this model
    uncOptModelObjType m_objType;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%% Uncertain Single Stage Optimization Model %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Single stage uncertain optimization model
class UncertainSingleStageOptimizationModel : public UncertainOptimizationModel
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of UncertainSingleStageOptimizationModel
    UncertainSingleStageOptimizationModel(uncOptModelObjType objType = robust) :
    UncertainOptimizationModel(1,objType)
    {}
    
    /// Destructor of UncertainSingleStageOptimizationModel
    ~UncertainSingleStageOptimizationModel(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @warning No adaptive decision variable is allowed in the single stage uncertain optimization model
    void checkCompatibility(ROCPPVarIF_Ptr pVariable) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual problemType getType() const {return uncertainssType;}
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% MISOCP Optimization Model %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Mix integer socp model
class MISOCP : public DeterministicOptimizationModel
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of MISOCP
    MISOCP() : DeterministicOptimizationModel()
    {}
    
    /// Destructor of MISOCP
    ~MISOCP(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @warning No uncertainty and bilinearity is allowed in this model
    void checkCompatibility(ROCPPConstraint_Ptr pConstraint) const;
    
    /// @warning No adaptive decision variable is allowed in this model
    void checkCompatibility(ROCPPVarIF_Ptr pVariable) const;
    
    /// @warning No adaptive variable and no uncertainty are allowed in this model
    void checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual problemType getType() const {return misocpType;}
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% Bilinear MISOC Optimization Model %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Bilinear  mix integer socp model
class Bilinear_MISOCP : public DeterministicOptimizationModel
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of Bilinear_MISOCP
    Bilinear_MISOCP() : DeterministicOptimizationModel()
    {}
    
    /// Destructor of Bilinear_MISOCP
    ~Bilinear_MISOCP(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @warning No uncertainty is allowed in this model
    void checkCompatibility(ROCPPConstraint_Ptr pConstraint) const;
    
    /// @warning No adaptive decision variable is allowed in this model
    void checkCompatibility(ROCPPVarIF_Ptr pVariable) const;
    
    /// @warning No adaptive variable and no uncertainty are allowed in this model
    void checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual problemType getType() const {return bmisocpType;}
};
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% CPLEX MISOC Optimization Model %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// CPLEX MISOCP model
class CPLEXMISOCP : public DeterministicOptimizationModel
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of CPLEXMISOCP
    ///
    /// Turn all constraints in the given model to the from of: cone head^2 >= \sum non cone head ^2 and create a CLPEXMISCOP problem
    /// @param pIn Input model
    /// @param baseVarNme Base name of the variables
    CPLEXMISOCP(ROCPPMISOCP_Ptr pIn, string baseVarNme = "s");
    
    /// Destructor of CPLEXMISOCP
    ~CPLEXMISOCP(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @warning No uncertainty is allowed in this model
    void checkCompatibility(ROCPPConstraint_Ptr pConstraint) const;
    
    /// @warning No adaptive decision variable is allowed in this model
    void checkCompatibility(ROCPPVarIF_Ptr pVariable) const;
    
    /// @warning No adaptive variable and no uncertainty is allowed in this model
    void checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Goer Functions %%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Create product term for the cone head variable and call function CPLEXMISOCP::add_cplexsoc_constraint(ROCPPProdTerm_Ptr, const vector<ROCPPProdTerm_Ptr>)
    void add_cplexsoc_constraint(ROCPPVarIF_Ptr coneHead, const vector<ROCPPVarIF_Ptr> &otherVars);
    
    /// Add constraint in the form of: cone head^2 >= sum non cone head ^2
    void add_cplexsoc_constraint(ROCPPProdTerm_Ptr coneHead, const vector<ROCPPProdTerm_Ptr> &otherVars);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    problemType getType() const {return cplexmisocpType;}
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Base variable name
    string m_baseVarNme;
};



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% Uncertain Multi Stage Optimization Model %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Multi stage uncertain optimization model
class UncertainMultiStageOptimizationModel : public UncertainOptimizationModel
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of UncertainSingleStageOptimizationModel
    UncertainMultiStageOptimizationModel(uint numTimeStages = 1, uncOptModelObjType objType = robust) :
    UncertainOptimizationModel(numTimeStages,objType)
    {}
    
    /// Destructor of UncertainSingleStageOptimizationModel
    ~UncertainMultiStageOptimizationModel(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual problemType getType() const {return uncertainmsType;}
};




//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% DDU Optimization Model %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Optimization model with decision-dependent information discovery
class MultiStageOptModelDDID : public UncertainMultiStageOptimizationModel
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of DDUOptimizationModel
    MultiStageOptModelDDID(uint numTimeStages = 1, uncOptModelObjType objType = robust);
    
    /// Destructor of DDUOptimizationModel
    ~MultiStageOptModelDDID(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator for measurement variables
    typedef map<string, measPair>::const_iterator measVarsIterator;
    
    /// Constant iterator for decision-dependent uncertainties
    typedef uncMapType::const_iterator dduIterator;
    
    /// Constant iterator for non decision-dependent uncertainties
    typedef uncMapType::const_iterator nondduIterator;
    
    /// Constant iterator for ddu to measurement map
    typedef map< pair<string,uint>, measPair>::const_iterator dduToMeasMapIterator;
    
    /// Return a constant iterator pointing to the beginning of the measurement variable map (m_measVars)
    measVarsIterator measVarsBegin() const {return m_measVars.begin();}
    
    /// Return a constant iterator pointing to the end of the measurement variable map (m_measVars)
    measVarsIterator measVarsEnd() const {return m_measVars.end();}
    
    /// Return a constant iterator pointing to the beginning of the ddu container (m_dduContainer)
    dduIterator dduBegin() const;
    
    /// Return a constant iterator pointing to the end of the ddu container (m_dduContainer)
    dduIterator dduEnd() const;
    
    /// Return a constant iterator pointing to the beginning of the non ddu container (m_nondduContainer)
    nondduIterator nondduBegin() const;
    
    /// Return a constant iterator pointing to the end of the non ddu container (m_nondduContainer)
    nondduIterator nondduEnd() const;
    
    /// Return a constant iterator pointing to the beginning of the ddu to measurement map (m_dduToMeasMap)
    dduToMeasMapIterator dduToMeasMapBegin() const;
    
    /// Return a constant iterator pointing to the end of the ddu to measurement map (m_dduToMeasMap)
    dduToMeasMapIterator dduToMeasMapEnd() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Add decision-dependent uncertainty to the model
    /// @param pUncertainty given uncertainty
    /// @param firstTimeStageObservable first time stage when this uncertainty can be observed
    /// @param lastTimeStageObservable last time stage when this uncertainty can be observed
    /// @param obsCosts map from time stage to the cost of observing the uncertainty at that stage
    void add_ddu(ROCPPUnc_Ptr pUncertainty, uint firstTimeStageObservable, uint lastTimeStageObservable, const map<uint, double> &obsCosts);
    
    /// @note Automatically adds the observation cost to objective function of the ddu model
    void set_objective(ROCPPObjectiveIF_Ptr pObj);
    
    using OptimizationModelIF::set_objective;
    
    /// Pair the given uncertainties: make their associated measurement variables equal (one is observed if and only if the other one is)
    void pair_uncertainties(ROCPPUnc_Ptr u1, ROCPPUnc_Ptr u2);
    
    void set_ddu(ROCPPOptModelIF_Ptr pIn);
    
    void set_ddu(const map< pair<string,uint>, measPair> &dduToMeasMap, const map< string, pair<uint,uint> > &dduStagesObs);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return an iterator pointing to the measurement pair corresponding to the uncertain parameter uncName and to the time stage timeStage
    dduToMeasMapIterator find(string uncName, uint timeStage) const;
    
    /// Return the measurement pair associated with the measurement variable varName
    measVarsIterator find(string varName) const;
    
    /// Return true if and only if the variable called varName is a measurement variable
    bool isMeasVar(string varName) const;
    
    /// Return true if and only if the uncertainty called uncName has a time of revelation that is decision-dependent
    bool isDDU(string uncName) const;
    
    bool isMultiStageOptModelDDID() const {return true;}
    
    /// Return the first time stage when uncertainty called uncName can be observed
    uint getFirstStageObservable(string uncName) const;
    
    /// Return the last time stage when uncertainty called uncName can be observed
    uint getLastStageObservable(string uncName) const;
    
    /// Return the number of uncertain parameters whose of revelation is decision-dependent in this model
    size_t getNumDDUncertainties() const;
    
    /// Return the measurement variable associated with the uncertain parameter dduncName at time timeStage
    ROCPPVarIF_Ptr getMeasVar(string dduncName, uint timeStage) const;
    
    /// Return the uncertain parameter associated with a given measurement variable
    ROCPPUnc_Ptr getAssociatedUncertainty(string measVar) const;
    
    /// Return the map m_dduToMeasMap
    map< pair<string,uint>, measPair> getDDUToMeasMap() const {return m_dduToMeasMap;}
    
    /// Return the map m_dduStageObs
    map<string, pair<uint,uint> > getdduStagesObs() const {return m_dduStagesObs;}
    
    problemType getType() const {return dduType;}
    
    //using UncertainOptimizationModel::hasDecisionDependentUncertaintySet();
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual void WriteToFile(string folderName, string fileName) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPOptModelIF_Ptr Clone() const;
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Protected Functions %%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return the ddu container of the model
    ROCPPuncContainer_Ptr getDDUContainer() const {return m_dduContainer;}
    
    /// Return the non ddu container of the model
    ROCPPuncContainer_Ptr getnonDDUContainer() const {return m_nondduContainer;}
    
    /// Return the map of all measurement variables
    map<string, measPair> getMeasVars() const {return m_measVars;}
    
    /// Add uncertainty pUnc to this model; also add the uncertainty either in ddu container or in non ddu container depending on its type
    void add_uncertainty( ROCPPUnc_Ptr pUnc);
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Map from name of the ddu to pair of measurement variable and observation cost
    map<string, pair<ROCPPVarIF_Ptr, double> > obsCost;
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Tool function %%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPOptModelIF_Ptr InstanciateModel( problemType type, uint numTimeStages=1, uncOptModelObjType objType = robust );

bool approximatelyEqual(double a, double b, double epsilon);


#endif /* OptimizationModel_hpp */
