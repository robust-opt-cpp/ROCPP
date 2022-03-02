/*
 * ROCPP/DecisionVariable.hpp
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


#ifndef DecisionVariable_hpp
#define DecisionVariable_hpp

#include "HeaderIncludeFiles.hpp"
#include <map>
#include <math.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% DECISION VARIABLE ENUMERATED TYPE %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Enumerator listing the possible types for the decision variables
enum decVariableType {
    /// integer decision variable
    intDV,
    /// real valued decision variable
    contDV,
    /// binary decision variable
    boolDV
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% DECISION VARIABLE INTERFACE %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Decision variable interface class
class DecisionVariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the DecisionVariableIF class
    /// @param name name of the decision variable
    /// @param type type of the decision variable
    /// @param lb lower bound of the variable
    /// @param ub upper bound of the variable
    /// @note The time-stage of an adaptive variable must greater than 1 (strictly).
    DecisionVariableIF(string name, decVariableType type, double lb, double ub);
    
    /// Destructor of the DecisionVariableIF class
    virtual ~DecisionVariableIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the variable type
    /// @return type of the decision variable
    decVariableType getType() const {return m_type;}
    
    /// Return the name of the decision variable
    /// @return name of the decision variable, string type
    string getName() const {return m_name;}
    
    /// Return the time-stage when the decision is made
    /// @return time-stage, uint type
    ///
    /// An adaptive decision taken at time-stage t can depend on all uncertain parameters revealed up to and including time t
    virtual uint getTimeStage() const = 0;
    
    /// Return indicator of adaptability, return value equals true if and only if the decision variable can adapt to the history of observations
    /// @return adaptability indicator, boolean type
    virtual bool isAdaptive() const = 0;

    /// Return true if and only if the decision variable is of boolean type
    virtual bool isBooleanVar() const = 0;
    
    /// Return true if and only if the decision variable is of integer type
    virtual bool isIntegerVar() const = 0;
    
    /// Return true if and only if the decision variable is of real-valued type
    virtual bool isRealVar() const = 0;
    
    /// Return the lower bound of the decision variable
    /// @return lower bound, double type
    double getLB() const {return m_lb;}
    
    /// Return the upper bound of the decision variable
    /// @return upper bound, double type
    double getUB() const {return m_ub;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Set lower bound of the variable
    virtual void setLB(double lb);
    
    /// Set upper bound of the variable
    virtual void setUB(double ub);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return a copy of the decision variable
    virtual ROCPPVarIF_Ptr Clone() = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return the lower bound of the decision variable as a string
    /// @return lower bound, string type
    string writeLB() const;
    
    /// Return the upper bound of the decision variable as a string
    /// @return upper bound, string type
    string writeUB() const;
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Name of the decision variable
    string m_name;
    
    /// Type of the decision variable
    const decVariableType m_type;
    
    /// Lower bound of the decision variable
    double m_lb;
    
    /// Upper bound of the decision variable
    double m_ub;
    
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% STATIC VARIABLES INTERFACE %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Static decision variable interface class
class VariableIF : public DecisionVariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the VariableIF class
    /// @param name name of the decision variable
    /// @param type type of the decision variable
    /// @param lb lower bound of the variable
    /// @param ub upper bound of the variable
    VariableIF(string name, decVariableType type, double lb, double ub):
        DecisionVariableIF(name, type, lb, ub){};
    
    /// Destructor of the VariableIF class
    virtual ~VariableIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    bool isAdaptive() const {return false;}
    uint getTimeStage() const {return 1;}
    
    
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% STATIC BOOLEAN VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Static boolean variable
class VariableBool : public VariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the VariableBool class
    VariableBool(string name, double lb = 0., double ub = 1.0);
    
    /// Destructor of the VariableBool class
    ~VariableBool(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    /// Always returns true for VariableBool
    bool isBooleanVar() const {return true;}
    
    bool isIntegerVar() const {return false;}
    bool isRealVar() const {return false;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Set lower bound of the boolean variable.
    /// If the input value is greater than 0.0, then the lower bound will be set to 1.0, else it will be set to 0.0
    void setLB(double lb);
    
    /// Set upper bound of the boolean variable.
    /// If the input value less than 1.0, then the upper bound will be set to 0.0, else it will be set to 1.0
    void setUB(double ub);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPVarIF_Ptr Clone();
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% STATIC INTEGER VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Static integer variable
class VariableInt : public VariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the VariableInt class
    VariableInt(string name, double lb = -INFINITY, double ub = INFINITY);
    
    /// Destructor of the VariableInt class
    ~VariableInt(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Always returns true for VariableInt
    bool isIntegerVar() const {return true;}
    
    bool isBooleanVar() const {return false;}
    bool isRealVar() const {return false;}

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return a copy of the decision variable
    ROCPPVarIF_Ptr Clone();
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% STATIC DOUBLE VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Static real-valued variable
class VariableDouble : public VariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the VariableDouble class
    VariableDouble(string name, double lb = -INFINITY, double ub = INFINITY);
    
    /// Destructor of the VariableInt class
    ~VariableDouble(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Always returns true for VariableDouble
    bool isRealVar() const {return true;}
    
    bool isIntegerVar() const {return false;}
    bool isBooleanVar() const {return false;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Return a copy of the decision variable
    ROCPPVarIF_Ptr Clone();
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% ADAPTIVE VARIABLES INTERFACE %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Adaptive variable interface class
class AdaptiveVariableIF : public DecisionVariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the AdaptiveVariableIF class
    /// @param timeStage time-stage when the decision is made
    AdaptiveVariableIF(string name, decVariableType type, uint timeStage, double lb, double ub);
    
    /// Destructor of the AdaptiveVariableIF class
    virtual ~AdaptiveVariableIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    bool isAdaptive() const {return true;}
    uint getTimeStage() const {return m_timeStage;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return a copy of this variable
    virtual ROCPPVarIF_Ptr Clone() = 0;
    
private:
    
    uint m_timeStage;
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% ADAPTIVE BOOLEAN VARIABLES %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Adaptive boolean variable
class AdaptVarBool : public AdaptiveVariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the AdaptVarBool class
    AdaptVarBool(string name, uint timeStage, double lb = 0., double ub = 1.0);
    
    /// Destructor of the AdaptVarBool class
    ~AdaptVarBool(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Always returns true for AdaptVarBool
    bool isBooleanVar() const {return true;}
    
    bool isIntegerVar() const {return false;}
    bool isRealVar() const {return false;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @see VariableBool::setLB()
    void setLB(double lb);
    /// @see VariableBool::setUB()
    void setUB(double ub);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return a copy of this variable
    ROCPPVarIF_Ptr Clone();
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% ADAPTIVE INTEGER VARIABLES %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Adaptive integer variable
class AdaptVarInt : public AdaptiveVariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the AdaptVarInt class
    AdaptVarInt(string name, uint timeStage, double lb = -INFINITY, double ub = INFINITY);
    
    /// Destructor of the AdaptVarInt class
    ~AdaptVarInt(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Always returns true for AdaptVarInt
    bool isIntegerVar() const {return true;}
    
    bool isBooleanVar() const {return false;}
    bool isRealVar() const {return false;}

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return a copy of this variable
    ROCPPVarIF_Ptr Clone();
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% ADAPTIVE REAL-VALUED VARIABLES %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Adaptive real-valued variable
class AdaptVarDouble : public AdaptiveVariableIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the AdaptVarDouble class
    AdaptVarDouble(string name, uint timeStage, double lb = -INFINITY, double ub = INFINITY);
    
    /// Destructor of the AdaptVarDouble class
    ~AdaptVarDouble(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Always returns true for AdaptVarDouble
    bool isRealVar() const {return true;}
    
    bool isIntegerVar() const {return false;}
    bool isBooleanVar() const {return false;}

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return a copy of this variable
    ROCPPVarIF_Ptr Clone();
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% DECISION VARIABLE CONTAINER %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Decision variable container
/*!
 Used to store decision variables for classes ConstraintTermIF, LHSExpression, etc
 */
class dvContainer
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the dvContainer class
    ///
    /// Builds an empty container
    dvContainer() :
        m_numContVars(0),
        m_numIntVars(0),
        m_numBoolVars(0),
        m_adaptiveContVarsCnt(0),
        m_adaptiveVarsCnt(0),
        m_timeStage(0) {}
   
    
    /// Destructor of the dvContainer class
    ~dvContainer(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Add non-overlapping decision variables from container a to this container
    void operator+=(const dvContainer &a);
    
    /// Add another decision variable to this container
    /// If the variable already exist, it will be ignored
    /// @note Adding different variables with same name is not allowed
    void operator+=(const ROCPPVarIF_Ptr& dv);
    
    /// Add integer type variables from this container into the dvs object
    void add_int_vars(dvContainer &dvs) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator for decision variable map
    typedef dvMapType::const_iterator const_iterator;
    
    /// Return a constant iterator pointing to the beginning of the decision variable map
    const_iterator begin() const {return m_DVMap.begin();}
    
    /// Return a constant iterator pointing to the end of the decision variable map
    const_iterator end() const {return m_DVMap.end();}
    
    /// Return a constant iterator pointing the variable called varName or to the end of the decision variable map if the variable does not exist
    const_iterator find(string varName) const;
    
    /// Find the variable with name varName in this object. If there is no such variable, throw an exception
    const_iterator findthrow(string varName) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the number of real-valued static variables in this container
    uint getNumContVars() const {return m_numContVars;}
    
    /// Get the number of integer static variables in this container
    uint getNumIntVars() const {return m_numIntVars;}
    
    /// Get the number of boolean static variables in this container
    uint getNumBoolVars() const {return m_numBoolVars;}
    
    /// Get the number of real-valued adaptive variables in this container
    uint getNumAdaptiveContVars() const {return m_adaptiveContVarsCnt;}
    
    /// Get the number of adaptive variables (of any type) in this container
    uint getNumAdaptiveVars() const {return m_adaptiveVarsCnt;}
    
    /// Get the number of decision variables in this container
    size_t getNumVars() const {return m_DVMap.size();}
    
    /// Get the maximum time-stage of any decision-variable in this container
    uint getTimeStage() const {return m_timeStage;}
    
    /// Return true if any decision variable in the container dvs exists in this object
    bool AnyVarIsInvolved(dvContainer& dvs) const;
    
    /// Find if the decision variable dv exists in this object
    ///
    /// Returns true if a variable with the same name exists in this container
    bool varIsInvolved(ROCPPVarIF_Ptr dv) const;
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Number of continuous variables in this container
    uint m_numContVars;
    
    /// Number of integer variables in this container
    uint m_numIntVars;
    
    /// Number of boolean variables in this container
    uint m_numBoolVars;
    
    /// Number of adaptive continuous variables in this container
    uint m_adaptiveContVarsCnt;
    
    /// Number of adaptive variables in this container
    uint m_adaptiveVarsCnt;
    
    /// Max time stage of variables in this container
    uint m_timeStage;
    
    /// Map from name to the decision variable
    dvMapType m_DVMap;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% CREATE VAR FUNCTION %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Function used to instantiate a decision variable of a particular type
/// @param name name of the decision variable
/// @param type type of the decision variable
/// @param lb lower bound of the variable
/// @param ub upper bound of the variable
/// @param isAdaptive equals true if and only if the decision variable can adapt to the history of observations
/// @param timeStage time-stage when the decision is made
/// @note The time-stage of an adaptive variable must greater than 1 (strictly).
ROCPPVarIF_Ptr createVariable(string name, decVariableType type, bool isAdaptive = false, uint timeStage = 1, double lb = -INFINITY, double ub = INFINITY);


#endif /* DecisionVariable_hpp */

