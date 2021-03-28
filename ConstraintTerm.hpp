//
//  ConstraintTerm.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef ConstraintTerm_hpp
#define ConstraintTerm_hpp

#include "HeaderIncludeFiles.hpp"
#include <map>
#include <vector>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% CONSTRAINT TERM ENUMERATED TYPE %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Enumerator listing the possible types for the constraint terms
enum constraintTermType {
    /// Product term.
    /// This corresponds to a monomial, i.e., to a term of the type a * x_1^{a_1} * x_2^{a_2} * .... * x_n^{a_n}
    prodTerm,
    /// Norm term.
    /// This corresponds to the two-norm of a LHSExpression, i.e., it can model terms of the type || f(x) ||_2 where f is a vector of LHSExpressions of x
    normTerm
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% CONSTRAINT TERM INTERFACE %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Constraint term interface
/*!
 Used to define a term in a constraint or in the objective function
*/
 class ConstraintTermIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the ConstraintTermIF class
    ConstraintTermIF();
    
    /// Destructor of the ConstraintTermIF class
    virtual ~ConstraintTermIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Operators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    /// Check it two terms are equal
    /// @note only valid in ProductTerm
    virtual bool operator==(const ProductTerm &other) const;
    
    /// Multiply this object with the given term
    /// @note only valid in ProductTerm
    virtual void operator*=(ROCPPconstCstrTerm_Ptr term);
    
    /// Multiply this object with the given decision variable
    /// @note only valid in ProductTerm
    virtual void operator*=(ROCPPVarIF_Ptr var);
    
    /// Multiply this object with the given uncertainty
    /// @note only valid in ProductTerm
    virtual void operator*=(ROCPPUnc_Ptr unc);
    
    /// Multiply this object with the given coefficient
    /// @note only valid in ProductTerm
    virtual void operator*=(double a);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator for product term decision variable map
    typedef dvMapType::const_iterator dvIterator;
    
    /// Constant iterator for product term uncertain parameter map
    typedef uncMapType::const_iterator uncIterator;
    
    /// Return a constant iterator pointing to the beginning of the decision variable map (m_pDVContainer)
    dvIterator varsBegin() const {return m_pDVContainer->begin();}
    
    /// Return a constant iterator pointing to the end of the decision variable map (m_pDVContainer)
    dvIterator varsEnd() const {return m_pDVContainer->end();}
    
    /// Return a constant iterator pointing to the beginning of the uncertain parameter map (m_pUncContainer)
    uncIterator uncBegin() const {return m_pUncContainer->begin();}
    
    /// Return a constant iterator pointing to the end of the uncertain parameter map (m_pUncContainer)
    uncIterator uncEnd() const {return m_pUncContainer->end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Map the old decision variables in this term to new variables
    /// @param mapFromOldToNewVars map from old variable name to new variable pointer
    virtual ROCPPCstrTerm_Ptr mapTermVars(const map<string,ROCPPVarIF_Ptr > &mapFromOldToNewVars) const = 0;
    
    /// Map the old uncertain parameters in this term to new uncertainties
    /// @param mapFromOldToNewUnc map from old uncertainty name to new uncertainty pointer
    virtual ROCPPCstrTerm_Ptr mapTermUnc(const map<string,ROCPPUnc_Ptr > &mapFromOldToNewUnc) const = 0;
    
    /// Map the old variables in this term to expressions
    /// @param mapFromVarToExpression map from old variables names to pointers to expressions
    virtual ROCPPExpr_Ptr mapVars(const map<string, ROCPPExpr_Ptr > &mapFromVarToExpression) const = 0;
    
    /// Map the old uncertainties in this term to expressions
    /// @param mapFromUncToExpression map from uncertain parameter names to pointers to expressions
    virtual ROCPPExpr_Ptr mapUncs(const map<string, ROCPPExpr_Ptr > &mapFromUncToExpression) const = 0;
    
    /// Replace the given term in this term with the given decision variable
    /// @param term map including the decision variables in the term to be replaced
    /// @param var variable used to replace the term
    /// @note Only replace the nonlinear term with variable
    virtual ROCPPCstrTerm_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr > &term, ROCPPVarIF_Ptr var) const = 0;
    
    /// Calculate the coeffiecient of the given uncertainty
    /// @note Only valid in class ProductTerm
    virtual pair<bool,ROCPPCstrTerm_Ptr> factorOut(ROCPPUnc_Ptr unc) const;
    
    /// Add a given constraint term to this term
    /// @note Only valid in class ProductTerm. Throws an exception if the two terms are not identical
    virtual void add(ROCPPconstCstrTerm_Ptr other);
    
    /// Add the decisions variables involved in a product in this term to the given container dvs
    virtual void add_vars_involved_in_prod(dvContainer &dvs) const = 0;
    
    /// Add the integer variables in this term to the given container
    void add_int_vars(dvContainer &dvs) const;
    
    /// Calculate the value of this term
    /// @param valuesMap a map from name of the variable to its value
    virtual double evaluate(const map<string,double>& valuesMap) const = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the products of any two variables in this term
    /// @param freqMap map from the pair of the variable names in each product to the frequency of this product
    /// @param termMap map from the pair of the variable names in each product to the map of the name to the variable involved in the product
    /// @note The calculated results are stored in the two inputs of this method
    virtual void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr > > &termMap) const = 0;
    
    /// Return true if and only if the term is of type prodTerm
    virtual bool isProductTerm() const {return false;}
    
    /// Return true if and only if the term is a product term and is nonlinear, i.e., if it involves products of decision variables
    virtual bool isNonlinearProdTerm() const;
    
    /// Return true if and only if the term is of type normTerm
    virtual bool isNormTerm() const {return false;}
    
    /// Return the type of the term
    /// @return enumerated type, term's type
    virtual constraintTermType getType() const = 0;
    
    /// Return true if and only if there is a product between two decision variables in this term
    virtual bool hasNonlinearities() const = 0;
    
    /// Return true if and only if there is a product between two uncertainties in this term
    virtual bool hasProdsUncertainties() const = 0;
    
    /// Return true if and only if there is a product between two continuous variables in this term
    virtual bool hasProdsContVars() const = 0;
    
    /// Return true if and only if the term is just a constant
    /// @note Only valid in class ProductTerm
    virtual bool isConstant() const;
    
    /// Return true if and only if the term is deterministic
    /// @note Only valid in class ProductTerm
    virtual bool isDeterministic() const;
    
    /// Return true if and only if the term is linear
    /// @note Only valid in class ProductTerm
    virtual bool isLinear() const;
    
    /// Return true if and only if the term is quadratic
    /// @note Only valid in class ProductTerm
    virtual bool isQuadratic() const;
    
    /// Return true if and only if the given constraint term is the same as this object
    virtual bool is_same(ROCPPconstCstrTerm_Ptr other) const = 0;
    
    /// Return true if and only if this object is not empty
    virtual bool isWellDefined() const = 0;
    
    /// Return the decision variable container (m_dvContainer) of this object
    ROCPPconstdvContainer_Ptr getDVContainer() const {return m_pDVContainer;}
    
    /// Return the uncertainty container (m_uncContainer) in this object
    ROCPPconstuncContainer_Ptr getUncContainer() const {return m_pUncContainer;}
    
    /// Return the coefficient (a double type) of this object
    /// @note Only valid in class ProductTerm
    virtual double getCoeff() const;
    
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
    
    /// Return the number of uncertain parameters in this term
    size_t getNumUncertainties() const;
    
    /// Return the number of decision variables in this term
    size_t getNumVars() const;
    
    /// Return true if and only if all integer decision variables in this term are bounded
    bool allIntVarsBounded() const;
    
    /// Get the number of times the given term appears in this object
    /// @param term map from name of the variable to the pointer of the variable involved in the term
    virtual uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr > &term) const = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Clone this term and return a pointer to the clone
    virtual ROCPPCstrTerm_Ptr Clone() const = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Print this term to the stream ofs
    /// @param ofs output file stream
    virtual void WriteToStream(ofstream &ofs) const = 0;
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Decision variable container
    ROCPPdvContainer_Ptr m_pDVContainer;
    
    /// Uncertain parameter container
    ROCPPuncContainer_Ptr m_pUncContainer;

};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% PRODUCT TERM %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Product term
/*!
 Term for product of coefficient, decision variables, and uncertainties (monomial)
 */
class ProductTerm : public ConstraintTermIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the ProductTerm class
    /// @param c coefficient
    ProductTerm(double c);
    
    /// Constructor of the ProductTerm class
    /// @param c coefficient
    /// @param pVariable decision variable involved in the product term
    ProductTerm(double c, ROCPPVarIF_Ptr pVariable);
    
    /// Constructor of the ProductTerm class
    /// @param c coefficient
    /// @param pUncertainty uncertain parameter involved in the product term
    /// @param pVariable decision variable involved in the product term
    ProductTerm(double c, ROCPPUnc_Ptr pUncertainty,  ROCPPVarIF_Ptr pVariable);
    
    /// Constructor of the ProductTerm class
    /// @param c coefficient
    /// @param pUncertainty uncertain parameter involved in the product term
    ProductTerm(double c, ROCPPUnc_Ptr pUncertainty);
    
    /// Constructor of the ProductTerm class
    /// @param c coefficient
    /// @param pVariable1 first decision variable involved in the product term
    /// @param pVariable2 second decision variable involved in the product term
    ProductTerm(double c, ROCPPVarIF_Ptr pVariable1, ROCPPVarIF_Ptr pVariable2);
    
    /// Constructor of the ProductTerm class
    /// @param c coefficient
    /// @param pUncertainty uncertain parameter involved in the product term
    /// @param pVariable1 first decision variable involved in the product term
    /// @param pVariable2 second decision variable involved in the product term
    ProductTerm(double c, ROCPPUnc_Ptr pUncertainty, ROCPPVarIF_Ptr pVariable1, ROCPPVarIF_Ptr pVariable2);
    
    /// Constructor of the ProductTerm class
    /// @param c coefficient
    /// @param uncVec vector of uncetainties involved in the product term
    /// @param varVec vector of decision variables involved in the product term
    ProductTerm(double c, const vector<ROCPPUnc_Ptr > &uncVec, const vector<ROCPPVarIF_Ptr > &varVec);
    
    /// Destructor of the ProductTerm class
    ~ProductTerm(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Operators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    bool operator==(const ProductTerm &other) const;
    void operator*=(ROCPPconstCstrTerm_Ptr term);
    void operator*=(ROCPPVarIF_Ptr var){addVariable(var);}
    void operator*=(ROCPPUnc_Ptr unc){addUncertainty(unc);}
    void operator*=(double a){m_coeff *= a;}
    
    /// Multimap from string to decision variable
    typedef multimap<string, ROCPPVarIF_Ptr > varMapType;
    
    /// Multimap from string to uncertain parameter
    typedef multimap<string, ROCPPUnc_Ptr > uncMapType;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator for decision variable map (m_DVMap)
    typedef multimap<string, ROCPPVarIF_Ptr >::const_iterator varsIterator;
    
    /// Constant iterator for uncertain parameter map (m_UncMap)
    typedef multimap<string, ROCPPUnc_Ptr >::const_iterator uncIterator;
    
    /// Return a pointer to the begining of the decision variable map (m_DVMap)
    varsIterator varsBegin() const {return m_DVMap.begin();}
    
    /// Return a pointer to the end of the decision variable map (m_DVMap)
    varsIterator varsEnd() const {return m_DVMap.end();}
    
    /// Return a pointer to the begining of the uncertain parameter map (m_UncMap)
    uncIterator uncBegin() const {return m_UncMap.begin();}
    
    /// Return a pointer to the end of the uncertain parameter map (m_UncMap)
    uncIterator uncEnd() const {return m_UncMap.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Calculate the coeffiecient of the given uncertainty
    /// @return The first element of the pair indicates whether the given uncertainty exists in this object, the second element stores the coefficient if the uncertianty exist
    pair<bool,ROCPPCstrTerm_Ptr> factorOut(ROCPPUnc_Ptr unc) const;
    
    ROCPPCstrTerm_Ptr mapTermVars(const map<string,ROCPPVarIF_Ptr > &mapFromOldToNewVars) const;
    ROCPPCstrTerm_Ptr mapTermUnc(const map<string,ROCPPUnc_Ptr > &mapFromOldToNewUnc) const;
    
    ROCPPExpr_Ptr mapVars(const map<string, ROCPPExpr_Ptr > &mapFromVarToExpression) const;
    ROCPPExpr_Ptr mapUncs(const map<string, ROCPPExpr_Ptr > &mapFromUncToExpression) const;
    
    ROCPPCstrTerm_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr > &term, ROCPPVarIF_Ptr var) const;
    
    void add(ROCPPconstCstrTerm_Ptr other);
    void add_vars_involved_in_prod(dvContainer &dvs) const;
    double evaluate(const map<string,double>& valuesMap ) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    constraintTermType getType() const;
    bool isProductTerm() const {return true;}
    bool hasNonlinearities() const;
    bool hasProdsUncertainties() const;
    bool hasProdsContVars() const;
    bool isConstant() const {return ( (m_DVMap.size()==0) && (m_UncMap.size()==0) );}
    bool isDeterministic() const {return (m_UncMap.size()==0);}
    bool isLinear() const {return (m_DVMap.size()<=1);}
    bool isQuadratic() const {return (m_DVMap.size()==2);}
    double getCoeff() const {return m_coeff;}
    bool isWellDefined() const {return true;}//( (!m_DVMap.empty()) || (!m_UncMap.empty()) );}
    bool is_same(ROCPPconstCstrTerm_Ptr other) const;
    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr > &term) const;
    void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr > > &termMap) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPCstrTerm_Ptr Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void WriteToStream(ofstream &ofs) const;

private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Coefficient of this term
    double m_coeff;
    
    /// Multimap from decision variable name to decision variable. The variable appears in the map as many times as it is present in the term.
    multimap<string, ROCPPVarIF_Ptr > m_DVMap;
    
    /// Multimap from uncertain parameter name to the uncertain parameter. The uncertain parameter appears in the map as many times as it is present in the term.
    multimap<string, ROCPPUnc_Ptr > m_UncMap;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Fuctions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Multiply the term by pVariable
    /// @param pVariable decision variable that will multiply this term
    void addVariable(ROCPPVarIF_Ptr pVariable);
    
    /// Multiply the term by pUncertainty
    /// @param pUncertainty uncertain parameter that will multiply this term
    void addUncertainty(ROCPPUnc_Ptr pUncertainty);
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%% NORM TERM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Norm term (norm of vector of left hand-side expressions)
class NormTerm : public ConstraintTermIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of NormTerm
    /// @param pExpressionVec vector of expressions, every expression is an element in the norm term
    NormTerm(const vector<ROCPPExpr_Ptr > &pExpressionVec);
    
    /// Destructor of NormTerm
    ~NormTerm(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator in vector of LHSExpression
    typedef vector<ROCPPExpr_Ptr >::const_iterator const_iterator;
    
    /// Return a const iterator pointing to the begin of the m_pExpressionVec
    const_iterator begin() const {return m_pExpressionVec.begin();}
    
    /// Return a const iterator pointing to the end of the m_pExpressionVec
    const_iterator end() const {return m_pExpressionVec.end();}
    
     
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPCstrTerm_Ptr mapTermVars(const map<string,ROCPPVarIF_Ptr > &mapFromOldToNewVars) const;
    
    ROCPPCstrTerm_Ptr mapTermUnc(const map<string,ROCPPUnc_Ptr > &mapFromOldToNewUnc) const;
    
    ROCPPExpr_Ptr mapVars(const map<string, ROCPPExpr_Ptr > &mapFromVarToExpression) const;
    
    ROCPPExpr_Ptr mapUncs(const map<string, ROCPPExpr_Ptr > &mapFromUncToExpression) const;
    
    ROCPPCstrTerm_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr > &term, ROCPPVarIF_Ptr var) const;
    
    void add_vars_involved_in_prod(dvContainer &dvs) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    bool isNormTerm() const {return true;}
    constraintTermType getType() const;

    /// Return the number of elements in the norm term
    size_t getNumTerms() const {return m_pExpressionVec.size();}
    bool hasNonlinearities() const;
    bool hasProdsUncertainties() const;
    bool hasProdsContVars() const;
    bool isWellDefined() const;
    bool is_same(ROCPPconstCstrTerm_Ptr other) const;
    
    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr > &term) const;
    
    void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr > > &termMap) const;
    
    double evaluate(const map<string,double>& valuesMap ) const;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPCstrTerm_Ptr Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void WriteToStream(ofstream &ofs) const;

private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Vector of elements(expressions) involved in this norm term
    vector<ROCPPExpr_Ptr > m_pExpressionVec;
};



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% LEFT-HAND SIDE EXPRESSION %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Left hand-side expression class
class LHSExpression
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of LHSExpression
    LHSExpression() : m_hasNormTerm(false),  m_pDVContainer ( new dvContainer() ), m_pUncContainer( new uncContainer() ) {}
    
    /// Destructor of LHSExpression
    ~LHSExpression(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator in vector of constraint terms
    typedef vector<ROCPPCstrTerm_Ptr>::const_iterator ConstraintLHS_const_iterator;
    
    /// Iterator in vector of constraint terms
    typedef vector<ROCPPCstrTerm_Ptr>::iterator ConstraintLHS_iterator;
    
    /// Constant iterator in vector of constraint terms
    typedef ConstraintLHS_const_iterator const_iterator;
    
    /// Iterator in vector of constraint terms
    typedef ConstraintLHS_iterator iterator;
    
    /// Return a constant iterator pointing to the beginning of m_terms in the expression
    const_iterator begin() const {return m_terms.begin();}
    
    /// Return a constant iterator pointing to the end of m_terms in the expression
    const_iterator end() const {return m_terms.end();}
    
    /// Constant iterator in decision variable map
    typedef dvMapType::const_iterator dvIterator;
    
    /// Constant iterator in uncertain parameter map
    typedef uncMapType::const_iterator uncIterator;
    
    /// Return a constant iterator pointing to the beginning of the decision variable container (m_pDVContainer)
    dvIterator varsBegin() const {return m_pDVContainer->begin();}
    
    /// Return a constant iterator pointing to the end of the decision variable container m_pDVContainer
    dvIterator varsEnd() const {return m_pDVContainer->end();}
    
    /// Return a constaint iterator pointing to the beginning of the uncertain parameter container (m_pUncContainer)
    uncIterator uncBegin() const {return m_pUncContainer->begin();}
    
    /// Return a constant iterator pointing to the end of the uncertain parameter container (m_pUncContainer)
    uncIterator uncEnd() const {return m_pUncContainer->end();}
    
    /// Find a term in the expression
    /// @param term term to add to the expression
    const_iterator find(ROCPPconstCstrTerm_Ptr term) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Operators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Multiply expression by given term
    /// @param term term to multiply this expression by
    void operator*=(ROCPPconstCstrTerm_Ptr term);
    
    /// Multiply expression by given expression
    /// @param other expression to multiply this expression by
    void operator*=(ROCPPconstExpr_Ptr other);
    
    /// Multiply expression by given uncertain parameter
    /// @param unc uncertain parameter to multiply this expression by
    void operator*=(ROCPPUnc_Ptr unc);
    
    /// Multiply expression by given decision variable
    /// @param var decision variable to multiply this expression by
    void operator*=(ROCPPVarIF_Ptr var);
    
    /// Multiply expression by given constant
    /// @param a constant to multiply this expression by
    void operator*=(double a);
    
    /// Add uncertain parameter to expression
    /// @param unc uncertain parameter to add to this expression
    void operator+=(ROCPPUnc_Ptr unc) {add(1.,unc);}
    
    /// Add decision variable to expression
    /// @param var decision variable to add to this expression
    void operator+=(ROCPPVarIF_Ptr var) {add(1.,var);}
    
    /// Add term to expression
    /// @param term term to add to this expression
    void operator+=(ROCPPconstCstrTerm_Ptr term) {add(term);}
    
    /// Add expression to expression
    /// @param other expression to add to this expression
    void operator+=(ROCPPconstExpr_Ptr other){add(other);}
    
    /// Add constant to expression
    /// @param a constants to add to this expression
    void operator+=(double a){add(a);}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Create a new ProductTerm using the given input and add it to the expression
    /// @param c constant to add to the expression
    void add(double c);
    
    /// Create a new ProductTerm using the given inputs and add it to the expression
    /// @param c coefficient of the term to add
    /// @param pVariable decision variable of the term to add
    void add(double c, ROCPPVarIF_Ptr pVariable);
    
    /// Create a new ProductTerm using the given inputs and add it to the expression
    /// @param c coefficient of the term to add
    /// @param pUncertainty uncertain parameter of the term to add
    /// @param pVariable decision variable of the term to add
    void add(double c, ROCPPUnc_Ptr pUncertainty,  ROCPPVarIF_Ptr pVariable);
    
    /// Create a new ProductTerm using the given inputs and add it to the expression
    /// @param c coefficient of the term to add
    /// @param pUncertainty uncertain parameter of the term to add
    void add(double c, ROCPPUnc_Ptr pUncertainty);
    
    /// Create a new ProductTerm using the given inputs and add it to the expression
    /// @param c coefficient of the term to add
    /// @param pVariable1 first decision variable of the term to add
    /// @param pVariable2 second decision variable of the term to add
    void add(double c, ROCPPVarIF_Ptr pVariable1, ROCPPVarIF_Ptr pVariable2);
    
    /// Create a new ProductTerm using the given inputs and add it to the expression
    /// @param c coefficient of the term to add
    /// @param pUncertainty uncertain parameter of the term to add
    /// @param pVariable1 first decision variable of the term to add
    /// @param pVariable2 second decision variable of the term to add
    void add(double c, ROCPPUnc_Ptr pUncertainty, ROCPPVarIF_Ptr pVariable1, ROCPPVarIF_Ptr pVariable2);
    
    /// Add the given term to the expression
    /// @note If the term already exists, change the coefficient
    /// @warning One expression can only have one norm term with coefficient equal to 1
    void add(ROCPPconstCstrTerm_Ptr term); // should only have single norm term!!
    
    /// Add the given expression to this expression
    /// @param expr expression to add to this expression
    /// @note Call LHExpression::add(ROCPPconstCstrTerm_Ptr) for every term in this expression
    void add(ROCPPconstExpr_Ptr expr);
    
    /// Add the product of the given inputs into the expression
    /// @param c coefficient
    /// @param term term to add to expression after multiplying it by coefficient
    /// @note Call LHExpression::add(ROCPPconstCstrTerm_Ptr) after multiplying term by c
    void add(double c, ROCPPconstCstrTerm_Ptr term); // should only have single norm term!!
    
    /// Add the given expression multiplied by c to this expression
    /// @param c coefficient
    /// @param expr expression to add to this expression after multiplying it by coefficient c
    /// @note Call LHExpression::add(ROCPPconstExpr_Ptr) after scaled
    void add(double c, ROCPPconstExpr_Ptr expr);
    
    /// Add the product of the given inputs to the expression
    /// @param c coefficient
    /// @param pVariable decision variable
    /// @param pExpression expression to add to this expression after multiplying it by coefficient c and by decision variable pVariable
    /// @note Call LHExpression::add(ROCPPconstExpr_Ptr)
    void add(double c, ROCPPconstExpr_Ptr pExpression,  ROCPPVarIF_Ptr pVariable);
    
    /// Add the product of the given inputs to the expression
    /// @param c coefficient
    /// @param pUnc uncertain parameter
    /// @param pExpression expression to add to this expression after multiplying it by coefficient c and by uncertain parameter pUnc
    /// Call LHExpression::add(ROCPPconstExpr_Ptr)
    void add(double c, ROCPPconstExpr_Ptr pExpression,  ROCPPUnc_Ptr pUnc);
    
    /// Map the old decision variables in this expression to new decision variables
    /// @note Call ProductTerm::mapTermsVars() or NormTerm::mapTermsVars() for every term based on its type in this expression
    ROCPPExpr_Ptr mapExprVars(const map<string,ROCPPVarIF_Ptr > &mapFromOldToNewVars) const;
    
    /// Map the old uncertain parameters in this expression to new uncertain parameters
    /// @note Call ProductTerm::mapTermsUnc() or NormTerm::mapTermsUnc() for every term based on its type in this expression
    ROCPPExpr_Ptr mapExprUnc(const map<string,ROCPPUnc_Ptr > &mapFromOldToNewUnc) const;

    /// Map the old variables in this expression to some expressions
    /// @note Call ProductTerm::mapVars() or NormTerm::mapVars() for every term based on its type in this expression
    ROCPPExpr_Ptr mapVars(const map<string, ROCPPExpr_Ptr > &mapFromVarToExpression) const;
    
    /// Map the old uncertain parameters in this expression to some expressions
    /// @note Call ProductTerm::mapUnc() or NormTerm::mapUnc() for every term based on its type in this expression
    ROCPPExpr_Ptr mapUncs(const map<string, ROCPPExpr_Ptr > &mapFromUncToExpression) const;

    /// Replace the given term in this expression with the given decision variable
    /// @note Call ProductTerm::replaceTermWithVar() or NormTerm::replaceTermWithVar() for every term based on its type in this expression
    ROCPPExpr_Ptr replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr > &term, ROCPPVarIF_Ptr var) const;
    
    /// Replace the bilinear term in this object with the given decision variable
    /// @param allTerms map from pair of decision variable names that define the bilinear term to the decision variable used to replace the term
    /// @param count number of different bilinear terms
    /// @note The function will search and replace all the bilinear terms in this expression and store the information in the two input parameters
    /// @note If the same bilinear term appears later, it will be replaced by the same variable
    /// @note The time stage of the new variable is decided by the maximum time-stage of the decision variables in the bilinear term
    /// @warning The bilinear term should include at least one boolean varaible
    ROCPPExpr_Ptr replaceBilinearTerm(map< pair<string,string>, ROCPPVarIF_Ptr > &allTerms, uint &count) const;
    
    /// Calculate the coeffiecient of the given uncertain parameter
    /// @note Call ProductTerm::factorOut for every ProductTerm in this expression
    pair<bool,ROCPPExpr_Ptr > factorOut(ROCPPUnc_Ptr unc) const;
    
    /// Add the variable involved in product in this expression into the given container
    /// @note Call ConstraintTermIF::add_vars_involved_in_prod() based on the type for each term in this expression
    void add_vars_involved_in_prod(dvContainer &dvs) const;
    
    /// Add the integer variable in this expression into the given container
    /// @note Call ConstraintTermIF::add_int_vars() based on the type for each term in this expression
    void add_int_vars(dvContainer &dvs) const;
    
    /// Calculate the value of this expression at the given solution
    /// @note Call ConstraintTermIF::evaluate() based on the type for each term in this expression
    double evaluate(const map<string,double>& valuesMap) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Calculate the sum of the constant terms in this expression
    double getSumConstantTerms() const;
    
    /// Get the number of terms in this experssion
    size_t getNumTerms() const {return m_terms.size();}
    
    /// Returns true if and only if the expression has a norm term
    bool hasNormTerm() const {return m_hasNormTerm;}
    
    /// Returns true if and only if the expression involves nonlinear terms (i.e., terms involving products of decision variables)
    bool hasNonlinearities() const;
    
    /// Returns true if and only if the expression involves products of uncertain parameters
    bool hasProdsUncertainties() const;
    
    /// Returns true if and only if the expression involves products of real-valued decision variables
    bool hasProdsContVars() const;
    
    /// Returns true if and only if the expression does not involve any uncertain parameters
    bool isDeterministic() const;
    
    /// Returns true if and only if the expression involves only constant terms
    bool isConstant() const;
    
    /// Returns true if and only if the expression is linear
    bool isLinear() const;
    
    /// Returns true if and only if the expression is quadratic
    bool isQuadratic() const;
    
    /// Return true if and only if this expression is not empty
    bool isWellDefined() const {return (!m_terms.empty());}
    
    /// Return the part without norm term (i.e., the sum of all product terms) in this expression
    ROCPPExpr_Ptr getLinearPart() const;
    
    /// Return the norm term in this expression
    ROCPPNormTerm_Ptr getNormTerm() const;
    
    /// Get the number of real-valued decision variables in the expression
    uint getNumContVars() const {return m_pDVContainer->getNumContVars();}
    
    /// Get the number of integer-valued decision variables in the expression
    uint getNumIntVars() const {return m_pDVContainer->getNumIntVars();}
    
    /// Get the number of boolean decision variables in the expression
    uint getNumBoolVars() const {return m_pDVContainer->getNumBoolVars();}
    
    /// Get the number of adaptive real-valued decision variables in the expression
    uint getNumAdaptiveContVars() const {return m_pDVContainer->getNumAdaptiveContVars();}
    
    /// Get the number of adaptive decision variables in the expression
    uint getNumAdaptiveVars() const {return m_pDVContainer->getNumAdaptiveVars();}
    
    /// Get the number of decision variables in the expression
    size_t getNumVars() const {return m_pDVContainer->getNumVars();}
    
    /// Get the maximum time-stage of any decision variable in the expression
    uint getTimeStage() const {return m_pDVContainer->getTimeStage();}
    
    /// Get the number of uncertain parameters in the expression
    size_t getNumUncertainties() const {return m_pUncContainer->getNumUncertainties();}
    
    /// Return the sum of all deterministic product terms in this expression
    ROCPPExpr_Ptr getDeterministicLinearPart() const;
    
    /// Get the number of times the given term appears in this expression
    /// @note Call ProductTerm::getNumTimesTermAppears() or NormTerm::getNumTimesTermAppears() based on the type for each term in this expression
    uint getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr > &term) const;
    
    /// Get products of any two variables in this expression
    /// @param freqMap map from the pair of the decision variable names in each product to the frequency of this product
    /// @param termMap map from the pair of the decision variable names in each product to the map of the name to the variable involved in the product
    /// @note The calculated results are stored in the two inputs of this method
    void getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr > > &termMap) const;
    
    /// Get the decision variable container
    ROCPPconstdvContainer_Ptr getDVContainer() const;
    
    /// Get the uncertain parameter container
    ROCPPconstuncContainer_Ptr getUncContainer() const;
    
    /// Get the variable by name
    ROCPPVarIF_Ptr getVar(string varName) const;
    
    /// Check whether the given variable is involved in this expression
    /// Call dvContainer::varIsInvolved() for the dvContainer of this expression
    bool varIsInvolved(ROCPPVarIF_Ptr dv) const;
    
    /// Check whether any decision variable in the given container is involved in this expression
    /// Call dvContainer::AnyVarIsInvolved() for the dvContainer of this expression
    bool AnyVarIsInvolved(dvContainer& dvs) const;
    
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Clone this expression
    ROCPPExpr_Ptr Clone() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Print this term to the stream ofs
    /// @param ofs output file stream
    void WriteToStream(ofstream &ofs) const;
 
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Vector of terms in this expression
    vector<ROCPPCstrTerm_Ptr> m_terms;
    
    /// Indicates whether this expression has a norm term or not
    bool m_hasNormTerm;

    /// Find a term in this expression
    iterator find(ROCPPCstrTerm_Ptr term);
    
    /// Decision variable container for this expression
    ROCPPdvContainer_Ptr m_pDVContainer;
    
    /// Uncertain parameter container for this expression
    ROCPPuncContainer_Ptr m_pUncContainer;
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% TOOL FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Given a term, find all possible products of 2 variables in the term and add them to the map passed by reference
bool GetAllProductsOf2Variables(const multimap<string, ROCPPVarIF_Ptr > &term, map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr > > &termMap);

/// Get the number of times that a term appears in a multimap
uint getNTTermAppears(const multimap<string, ROCPPVarIF_Ptr > &sup, const multimap<string, ROCPPVarIF_Ptr > &sub);



#endif /* ConstraintTerm_hpp */
