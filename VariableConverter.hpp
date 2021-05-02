//
//  VariableConverter.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef VariableConverter_hpp
#define VariableConverter_hpp

#include <stdio.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% VARIABLE CONVERTER INTERFACE %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Variable converter interface class
/// Use to map variables to other variables or to expressions
class VariableConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of VariableConverterIF class
    VariableConverterIF(){}
    
    /// Destructor of VariableConverterIF class
    ~VariableConverterIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator into inverse map
    typedef map<string, ROCPPVarIF_Ptr>::const_iterator const_iterator_inv;
    
    /// Return a constant iterator pointing to the pair of the given variable and the old variable associated with it
    const_iterator_inv findInv(string varName) const {return m_inverseMap.find(varName);}
    
    /// Return a constant iterator pointing to the beginning of the inverse map (m_inverseMap)
    const_iterator_inv beginInv() const {return m_inverseMap.begin();}
    
    /// Return a constant iterator pointing to the end of the inverse map (m_inverseMap)
    const_iterator_inv endInv() const {return m_inverseMap.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the objective and constraints in the input model and then convert the variables
    /// @param pIn Model to be approximated
    /// @param resetAndSave Whether to set the translation map in class OneToExprVariableConverterIF
    virtual ROCPPOptModelIF_Ptr convertVar(ROCPPOptModelIF_Ptr pIn, bool resetAndSave=false);
    
    /// Approximate each variable in the given objective and contraints
    /// @param first Constant iterator pointing to the beginning of the vector of constraints to be mapped
    /// @param last Constant iterator pointing to the end of the vector of constraints to be mapped
    /// @param obj Objective to be mapped
    /// @param origDVContainer Container of decision variables to be approximated
    /// @param toAdd Constraints after mapping
    /// @param toSet Objective after mapping
    virtual void convertVar(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, const dvContainer& origDVContainer, vector<ROCPPConstraintIF_Ptr> &toAdd, ROCPPObjectiveIF_Ptr &toSet,bool resetAndSave = false) = 0;
    
    /// Find the variable in the given ocnstraints and objective to be mapped and store them in the container
    virtual void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container) = 0;
    
    /// Create the map m_inverseMap
    virtual void createInverseMap(const dvContainer &origDVContainer) = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Evaluate and return the value of the given variable after converting it
    virtual double evaluateVariableValue( string nme,  const map<string,double>& binValuesMap ) const = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Print the solution of the given decision variable
    virtual void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &variableValue, ROCPPVarIF_Ptr dv) {};
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Map from the new variable name to old variable
    map<string, ROCPPVarIF_Ptr> m_inverseMap; // maps new variable(for example variables in LHS) name to old dv that it helps parameterize
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% ONE TO ONE VARIABLE CONVERTER %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Variable to variable converter interface
/*!
 Class for converting a variable to another variable
*/
 class OneToOneVariableConverterIF : public VariableConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of OneToOneVariableConverterIF class
    OneToOneVariableConverterIF(){}
    
    /// Destructor of OneToOneVariableConverterIF class
    ~OneToOneVariableConverterIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    typedef map<string,ROCPPVarIF_Ptr>::const_iterator const_iterator;
    
    const_iterator find(string varName) const {return m_translationMap.find(varName);}
    
    const_iterator begin() const {return m_translationMap.begin();}
    
    const_iterator end() const {return m_translationMap.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    using VariableConverterIF::convertVar;
    
    /// Approximate the original variable with a new variable or a constant for all variables in the input model
    /// @param resetAndSave Indicates whether to reset the translation map
    void convertVar(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, const dvContainer& origDVContainer, vector<ROCPPConstraintIF_Ptr> &toAdd, ROCPPObjectiveIF_Ptr &toSet, bool resetAndSave = false);
    
    /// Create translation map for the variables in the given container
    /// @param tmpContainer Container contains variables to be mapped
    /// @param translationMap Map from the name of the variable to the new variable it will be mapped to
    /// @param toAdd Contains the upper and lower bounds of the original variables as constraints
    virtual void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPVarIF_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd) = 0; // translationMap: map from dv name in original problem to pair of coeff , int var so that dv = sum coeff * int var; //tmpMap: identifies the integer, non-binary variables involved in bilinear terms in pMI_BLP
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void createInverseMap(const dvContainer &origDVContainer);
    
    double evaluateVariableValue( string nme,  const map<string,double>& binValuesMap ) const;
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Map from name of the original variable to the new variable
    map<string, ROCPPVarIF_Ptr> m_translationMap;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%% ONE TO EXPRESSION VARIABLE CONVERTER %%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Variable to expression converter interface
/*!
 Class for converting a variable to an expression
*/
class OneToExprVariableConverterIF : public VariableConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of OneToExprVariableConverterIF class
    OneToExprVariableConverterIF(){}
    
    /// Destructor of OneToExprVariableConverterIF class
    ~OneToExprVariableConverterIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    typedef map<string,ROCPPExpr_Ptr>::const_iterator const_iterator;
    
    const_iterator find(string varName) const {return m_translationMap.find(varName);}
    
    const_iterator begin() const {return m_translationMap.begin();}
    
    const_iterator end() const {return m_translationMap.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    using VariableConverterIF::convertVar;
    
    /// Map the original variable to an expression for all variables in the input model
    void convertVar(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, const dvContainer& origDVContainer, vector<ROCPPConstraintIF_Ptr> &toAdd, ROCPPObjectiveIF_Ptr &toSet, bool resetAndSave = false);
    
    /// Create m_translationMap for the variables in the given container
    /// @param tmpContainer Container contains the variables to be approximated
    /// @param translationMap Map from the name of the variable to the expression to which it will be mapped
    /// @param toAdd Contains the upper and lower bounds of the original variables as constraints
    virtual void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd) = 0; // translationMap: map from dv name in original problem to pair of coeff , int var so that dv = sum coeff * int var; //tmpMap: identifies the integer, non-binary variables involved in bilinear terms in pMI_BLP
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual void createInverseMap(const dvContainer &origDVContainer);
    
    double evaluateVariableValue( string nme,  const map<string,double>& binValuesMap ) const;
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Map from name of the original variable to the expression
    map<string, ROCPPExpr_Ptr> m_translationMap;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% BILINEAR MI TO MB CONVERTER %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Converts bilinear mixed integer terms to bilinear mixed binary terms
class Bilinear_MItoMB_Converter : public OneToExprVariableConverterIF, public ReformulationStrategyIF
{
public:
    
    /// Constructor of Bilinear_MItoMB_Converter class
    Bilinear_MItoMB_Converter(){}
    
    /// Destructor of Bilinear_MItoMB_Converter class
    ~Bilinear_MItoMB_Converter(){}
    
    void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
    
    
    ROCPPOptModelIF_Ptr Reformulate(ROCPPOptModelIF_Ptr pIn){return convertVar(pIn);}
    bool isApplicable(ROCPPOptModelIF_Ptr pIn) const {return true;}
    string getName() const {return "bilinear term mixed-integer to mixed-binary converter";}
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% UNARY CONVERTER %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class UnaryConverter : public Bilinear_MItoMB_Converter
{
public:
    
    UnaryConverter(){}
    ~UnaryConverter(){}
    
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd);//vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last,
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% BINARY CONVERTER %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


class BinaryConverter : public Bilinear_MItoMB_Converter
{
public:
    
    BinaryConverter(){}
    
    ~BinaryConverter(){}
    
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd);
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% REAL VARIABLE BILINEAR POS REFORMULATOR %%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// convert any real-valued variable involved in bilinear terms that does not have a lower bound to the difference between two non-negative real-valued variables
// and any real-valued variable that has a non-zero lower bound to a variable with zero lower bound
class RealVarBilinearPosReformulator : public OneToExprVariableConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    RealVarBilinearPosReformulator(){}
    
    ~RealVarBilinearPosReformulator(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
    
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPVarIF_Ptr getPosPart(string origVarNme) const;
    
    ROCPPVarIF_Ptr getNegPart(string origVarNme) const;

private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    map<string, ROCPPVarIF_Ptr> m_posPartMap;
    map<string, ROCPPVarIF_Ptr> m_negPartMap;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% REAL VARIABLE DISCRETIZER %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Approximate real-valued decisions
/*!
 Approximates real-valued decisions by a weighted sum of binaries
*/
class RealVarDiscretizer : public OneToExprVariableConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the RealVarDiscretizer class
    RealVarDiscretizer(uint numBitsPerVariable) : m_numBitsPerVariable(numBitsPerVariable) {}
    
    /// Destructor of the RealVarDiscretizer class
    ~RealVarDiscretizer(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd);
    
    virtual void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    uint getNumBitsPerVar() const {return m_numBitsPerVariable;}
    
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    uint m_numBitsPerVariable;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%% UNCERTAINTY SET REAL VARIABLE APPROXIAMATOR %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Approximate the real-valued decisions affecting the uncertainty set
/*!
 Approximates the real-valued decisions affecting the uncertainty set by weighted sums of binary variables
*/
class UncertaintySetRealVarApproximator : public RealVarDiscretizer, public ReformulationStrategyIF
{
public:
    
    /// Constructor of the UncertaintySetRealVarApproximator class
    UncertaintySetRealVarApproximator(uint numBitsPerVariable) : RealVarDiscretizer(numBitsPerVariable) {}
    
    /// Destructor of the UncertaintySetRealVarApproximator class
    ~UncertaintySetRealVarApproximator(){}
    
    void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
    
    ROCPPOptModelIF_Ptr Reformulate(ROCPPOptModelIF_Ptr pIn){return convertVar(pIn);}
    bool isApplicable(ROCPPOptModelIF_Ptr pIn) const {return true;}
    string getName() const {return "uncertainty set real variable approximator";}
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% PREDEF O2O VARIABLE CONVERTER %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Pre-defined variable to variable converter
/*!
 Class for converting a variable to another variable with a pre-defined translation map
*/
class PredefO2OVariableConverter : public OneToOneVariableConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the PredefO2OVariableConverter class
    PredefO2OVariableConverter(const map<string,ROCPPVarIF_Ptr>  &translationMap){m_translationMap=translationMap;}//, const dvContainer& origDVs);
    
    /// Destructor of the PredefO2OVariableConverter class
    ~PredefO2OVariableConverter(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    using VariableConverterIF::convertVar;
    
    ROCPPConstraintIF_Ptr convertVar(ROCPPConstraintIF_Ptr pCstr) const;
    
    void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
    
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPVarIF_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd);
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPdvContainer_Ptr m_origDVs;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% PREDEF O2E VARIABLE CONVERTER %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//!Pre-defined variable to expression converter
/*!
 Class for convert a variable to an expression with a pre-defined translation map
*/
 class PredefO2EVariableConverter : public OneToExprVariableConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the PredefO2EVariableConverter class
    PredefO2EVariableConverter(const map<string,ROCPPExpr_Ptr>  &translationMap){m_translationMap=translationMap;}//, const dvContainer& origDVs) : m_origDVs(origDVs) {m_translationMap=translationMap;}
    
    /// Destructor of the PredefO2EVariableConverter class
    ~PredefO2EVariableConverter(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    using VariableConverterIF::convertVar;
    
    ROCPPConstraintIF_Ptr convertVar(ROCPPConstraintIF_Ptr pCstr) const;
    
    void findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
    
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd);
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPdvContainer_Ptr m_origDVs;
};
#endif /* VariableConverter_hpp */
