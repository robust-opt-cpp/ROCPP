//
//  DecisionRule.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef DecisionRule_hpp
#define DecisionRule_hpp

#include <stdio.h>
#include <vector>
#include "HeaderIncludeFiles.hpp"


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%% DECISION RULE APPROXIMATOR INTERFACE %%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    void findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
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
class LinearDecisionRule: public ContinuousVarsDRIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the LinearDecisionRule class
    /// @param UC Container that contains all uncertainty being used to approximate the decision variable in this problem
    LinearDecisionRule(ROCPPuncContainer_Ptr UC, uint memory=1000) : ContinuousVarsDRIF(memory), m_UC(UC), m_uncContSet(true) {}
    
    /// Constructor of the LinearDecisionRule class
    LinearDecisionRule(uint memory=1000) : ContinuousVarsDRIF(memory), m_uncContSet(false) {}
    
    /// Destructor of the LinearDecisionRule class
    ~LinearDecisionRule(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Approximated the real-valued adaptive decisions in the given model by linear desicion rules
    /// First set the uncertainty container and then do the approximation
    /// @param pIn Model to be approximated
    /// @param resetAndSave Indicates whether to reset the translation map in class OneToExprVariableConverterIF
    /// @see VariableConverterIF::doMyThing(ROCPPOptModelIF_Ptr, bool)
    ROCPPOptModelIF_Ptr doMyThing(ROCPPOptModelIF_Ptr pIn, bool resetAndSave=false);
    
    /// Create the map from the original decision variable to decisions that are affine in the history of observations
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd);
    
    /// Set the uncertainty container of this class
    void setUncContainer(ROCPPuncContainer_Ptr UC){m_UC=UC; m_uncContSet=true;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the coefficient decision variable associated with the given uncertain parameter in the approximation for the given decision variable
    ROCPPVarIF_Ptr getCoeffDV(string dvName, string uncName) const;
    
    /// Return map m_mapOrigDVUncPairToCoeffDV
    map< pair<string,string>, ROCPPVarIF_Ptr > getLDRCoeff() const{return m_mapOrigDVUncPairToCoeffDV;}
    
    /// Return map m_mapOrigDVToUncAndCoeffDV
    multimap<string, pair<string, ROCPPVarIF_Ptr > > getLDRExpr() const{return m_mapOrigDVToUncAndCoeffDV;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Print the solution (in the form of an expression) for the given variable
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &variableValue, ROCPPVarIF_Ptr dv);
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Uncertain parameter container of this approximator
    ROCPPuncContainer_Ptr m_UC;
    
    bool m_uncContSet;
    
    /// Map of pair of original variable name and uncertainty name to the coefficient variable
    map< pair<string,string>, ROCPPVarIF_Ptr > m_mapOrigDVUncPairToCoeffDV;
    
    /// Map of original variable name to the pair of uncertainty name and the coefficient variable
    multimap<string, pair<string, ROCPPVarIF_Ptr > > m_mapOrigDVToUncAndCoeffDV;
    
    /// Map of original variable name to the name of decision variable representing the constant term in the linear decision rule
    map<string, string> m_cst;
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
    void findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container);
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
class ConstantDecisionRule : public DiscreteVarsDRIF
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
    
    void createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPVarIF_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd);
    
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &variableValue, ROCPPVarIF_Ptr dv);
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% PARTITION CONVERTER %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Partition converter
/*!
 Class for converting partition to a string
*/
class PartitionConverter
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the PartitionConverter class
    PartitionConverter(uint numEls) :
    m_numEls(numEls)
    {}
    
    /// Destructor of the PartitionConverter class
    ~PartitionConverter(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    string convertPartitionToString(uint partition) const;
    
    /// Convert the partition of the uncertainty in the input map to a string based on the order of each uncertainty in the uncertainty container in the input model
    string convertPartitionToString(const map<string,uint> &partitionIn, ROCPPOptModelIF_Ptr pModel) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the basic partition to reduce the amount of euqal constraints
    string getBasicPartition(const map<string,uint> &partitionIn, uint t, ROCPPOptModelIF_Ptr pModel, uint memory) const;
    
    uint getNumEls() const {return m_numEls;}
    
private:
    
    uint m_numEls;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% PARTITION CONSTRUCTOR INTERFACE %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Partition constructor interface
/*!
 Class for constructing partitions of uncertainty set
*/
class PartitionConstructorIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// @param numPartitionsMap Map from uncertainty name to number of break points in this direction
    PartitionConstructorIF(const map<string,uint> &numPartitionsMap);
    
    ~PartitionConstructorIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator of the partition map
    typedef map<string, map<string,uint> >::const_iterator const_iterator;
    
    /// Return a constant iterator pointing to the beginning of m_partitionsMap
    const_iterator begin() const {return m_partitionsMap.begin();}
    
    /// Return a constant iterator pointing to the end of m_partitionsMap
    const_iterator end() const {return m_partitionsMap.end();}
    
    /// Constant iterator of the vector of uncertainty set constraints
    typedef vector< ROCPPConstraint_Ptr >::const_iterator usconstraints_iterator;
    
    /// Return a constant iterator pointing to the beginning of constraints defining the uncertainty set on the given partition
    usconstraints_iterator USCbegin(string partition) const;
    
    /// Return a constant iterator pointing to the end of constraints defining the uncertainty set on the given partition
    usconstraints_iterator USCend(string partition) const;
    
    /// Constant iterator of the vector of additional constraints
    typedef vector< ROCPPConstraint_Ptr >::const_iterator addconstraints_iterator;
    
    /// Return a constant iterator pointing to the beginning of m_additionalConstraints
    addconstraints_iterator ACbegin() const {return m_additionalConstraints.begin();}
    
    /// Return a constant iterator pointing to the end of m_additionalConstraints
    addconstraints_iterator ACend() const {return m_additionalConstraints.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Calculate the upper and lower bounds for each uncertainty
    void getReady(ROCPPOptModelIF_Ptr pIn, ROCPPParConverter_Ptr pPartConverter, ROCPPMItoMB_Ptr pMIMBConverter, map<string, pair<double,double> > &margSupp, const map<string,pair<double,double> >& OAmargSupp, string solver = "gurobi"); // i.e. build all maps
    
    /// Reset all maps in this container
    void Reset() {m_numPartitionsMap.clear(); m_partitionsMap.clear(); m_partitionUSconstraints.clear(); m_uncToBreakpointMap.clear();}
    
    /// Decide on the breakpoint of each uncertainty and store the information in the maps
    virtual void constructUncToBreakpointMap(const map<string,pair<double,double> > &margSupp) = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPconstdvContainer_Ptr getBPDVContainer() const {return m_bpdvs; }
    
    /// Get the total number of subsets
    size_t getNumSubsets() const {return m_partitionsMap.size();}
    
    /// Get the number of subsets for the given uncertainty
    uint getNumSubsets(string uncNme) const;//return ri
    
    uint getPos(string partition, string uncNme) const;
    
    bool hasPartition(string uncNme) const;
    
    ROCPPExpr_Ptr getBp(pair<string, uint> uncOnPartition) const;
    
    map<string,uint> getNumPartitionsMap() const {return m_numPartitionsMap;}
    
protected:
    
    void constructPartitionsMap(ROCPPOptModelIF_Ptr pIn, ROCPPParConverter_Ptr pPartConverter);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Map from uncertainty to number of breakpoints in this direction
    map<string,uint> m_numPartitionsMap; // map from uncertainty to number of bps in this direction
    
    /// Map from partition_name to map from unc name to element of parition associated with this uncertainty
    map<string, map<string,uint> > m_partitionsMap; // map from partition_name to map from unc name to element of parition associated with this uncertainty
    
    /// Map from partition name to the vector of constraints specific to this partition
    map< string, vector< ROCPPConstraint_Ptr > > m_partitionUSconstraints; // uncertainty set on this partition: constraints specific to this partition
    
    /// Map from pair<unc name,breakpoint number> to dv modeling the breakpoint
    map< pair<string,uint>, ROCPPExpr_Ptr > m_uncToBreakpointMap; // map from pair<unc name,breakpoint number> to dv modeling the breakpoint
    
    /// Vector of additional constraints
    vector<ROCPPConstraint_Ptr > m_additionalConstraints;
    
    /// Container of breakpoint variables
    ROCPPdvContainer_Ptr m_bpdvs;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% STATIC PARTITION CONSTRUCTOR %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Static partition constructor
class StaticPartitionConstructor : public PartitionConstructorIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    StaticPartitionConstructor(const map<string,uint> &numPartitionsMap) : PartitionConstructorIF(numPartitionsMap) {}
    
    ~StaticPartitionConstructor(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Use constant as breakpoint of the partition
    void constructUncToBreakpointMap(const map<string,pair<double,double> > &margSupp);
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% ADAPTIVE PARTITION CONSTRUCTOR %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Adaptive partition constructor
class AdaptivePartitionConstructor : public PartitionConstructorIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    AdaptivePartitionConstructor(const map<string,uint> &numPartitionsMap) : PartitionConstructorIF(numPartitionsMap) {}
    
    ~AdaptivePartitionConstructor(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Use variable as breakpoint of the partition
    void constructUncToBreakpointMap(const map<string,pair<double,double> > &margSupp);
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
