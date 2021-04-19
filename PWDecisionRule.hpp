//
//  PWDecisionRule.hpp
//  ROCPP
//
//  Created by Phebe Vayanos on 4/3/21.
//

#ifndef PWDecisionRule_hpp
#define PWDecisionRule_hpp

#include <stdio.h>
#include <vector>
#include "HeaderIncludeFiles.hpp"


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
    typedef vector<ROCPPConstraint_Ptr>::const_iterator usconstraints_iterator;
    
    /// Return a constant iterator pointing to the beginning of constraints defining the uncertainty set on the given partition
    usconstraints_iterator USCbegin(string partition) const;
    
    /// Return a constant iterator pointing to the end of constraints defining the uncertainty set on the given partition
    usconstraints_iterator USCend(string partition) const;
    
    /// Get the uncertain parameter on the the given subset of the partition for the given original uncertain parameter
    ROCPPUnc_Ptr getUncOnPartition(string partition, string origUncName) const;
    
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
    
    map<string,ROCPPUnc_Ptr> getMapFromOriginalUncToSubsetUnc(string subset) const;
    
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
    
    /// Map from subset of partition to map from uncertain parameter in original problem to uncertainty on partition
    map<string, map<string,ROCPPUnc_Ptr> > m_UncMap;
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
//%%%%%%%%%%%%%%%%%%%% PIECEWISE DECISION RULE %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Piecewise constant and piecewise linear decision rule approximator
/// @warning: only applies to problems where the uncertainty set only depends on binary variables

class PiecewiseDecisionRule : public DecisionRuleIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the PiecewiseDecisionRule class
    /// @param pIn Model to be approximated
    /// @param numPartitionsStr String to encode the number of subsets for each observable uncertainty, in alphabetical order
    /// @param numBits Number of bits used for discretizing any real-valued variables affecting the uncertainty set
    /// @param bigM Big-M value used for linearizing the bilinear terms
    /// @param useExplicitNACs If true, then two meansurement variables are equal, otherwise use the same measurement variables for the NACs
    /// @param folder Name of folders where to store the log file
    
    /// Constructor of the PiecewiseApproximator class
    /// @param numPartitionsMap map from the name of the uncertain parameter to the number of subsets along that direction (number of breakpoints + 1)
    PiecewiseDecisionRule(const map<string,uint> &numPartitionsMap = map<string, uint>(), double bigM = 100., bool useExplicitNACs = false, string folder=" ");
    
    /// Constructor of the PiecewiseApproximator class
    /// Directly initialize with the predefined private members (for greater customization)
//    PiecewiseDecisionRule(ROCPPContinuousVarsDR_Ptr pCVA, ROCPPDiscreteVarsDR_Ptr pDVA,
//                          ROCPPParConstructor_Ptr pPartConstructor, ROCPPParConverter_Ptr pPartConverter,
//                          double bigM = 100., bool useExplicitNACs = false, string folder=" ") :
//    m_pCVA(pCVA), m_pDVA(pDVA), m_pPartConstructor(pPartConstructor), m_pPartConverter(pPartConverter), m_bigM(bigM), m_useExplicitNACs(useExplicitNACs) {}
    
    /// Destructor of the PiecewiseApproximator class
    ~PiecewiseDecisionRule(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    /// Approximate the input model
    /// First construct the partition maps, then calculate and robustity the model on each subset, finally add non-anticipativity constraints
    ROCPPOptModelIF_Ptr approximate(ROCPPOptModelIF_Ptr pIn);
    
    bool isApplicable(ROCPPOptModelIF_Ptr pIn) const;
    
    
    
    /// Return a new model after fixing the binary variables to the value provided in the input map
    ROCPPOptModelIF_Ptr fixBinaryVariableValues(ROCPPOptModelIF_Ptr pKadaptModel, const map<string,bool> &varValues) const;
    
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    string getSolutionApproachParameters(string delimiter) const;
    
    /// Find the variable that new decision variable originated from after LDR and CDR
    pair<bool,ROCPPVarIF_Ptr > findOrigVariable(ROCPPVarIF_Ptr newdv) const; // find the time stage of the variable that newdv originated from after LDR and CDR
    
    /// Get the decision variable on the the given subset of the partition for the given original variable
    ROCPPVarIF_Ptr getVarOnPartition(string partition, string origVarName) const;
    
    /// Calculate solution for the given variable on the given subset of the partition
    void calculateSolution(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv, string partition);
    
    /// Calculate the measurement variable value for the given uncertainty on the given subset of the partition
    void calculateSolution(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc, string partition);
    
    /// Return the warm start solution of this problem by setting the variables which exist in the warm start problem to their solutions and assigning the variables which do not exist in the warm start problem to the solutions on the nearest partition
    /// @param pIn Model before approximating
    /// @param warmStartResults Results of the warm start problem
    /// @param wsMap Map from time stage to number of k in the warm start problem
    /// @param wsSolutions Map from variable name to its warm start solution for all variables in the problem to be solved
    void getWsSolutions(ROCPPOptModelIF_Ptr pIn, const map<string, double> &warmStartResults, const map<string,uint> &wsMap, map<string, double> &wsSolutions);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void printParametersToScreen() const;
    
    /// Print out the solution for the given variable on the given subset of the partition
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv,  map<ROCPPUnc_Ptr, uint> partitionIn);
    
    /// Print out the solution for the given variable on all subsets of the the partition
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv);
    
    /// Print out whether observe or not the given uncertainty on the given subset of the partition
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc,  map<ROCPPUnc_Ptr, uint> partitionIn);
    
    /// Print out whether we observe or not the given uncertainty on all subsets of the partition
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc);
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// String encoding the number of subsets in the partition for every observable uncertainty in the problem, in alphabetical order
    string m_numPartitionsStr;
    
    /// Linear decision rule converter
    ROCPPContinuousVarsDR_Ptr m_pCVA;
    
    /// Constant decision rule converter
    ROCPPDiscreteVarsDR_Ptr m_pDVA;
    ROCPPMItoMB_Ptr m_pMItoMB_Bilinear;
    ROCPPUncSetRealVarApprox_Ptr m_pBPA;
    
    /// Partition constructor of this approximator
    ROCPPParConstructor_Ptr m_pPartConstructor;
    
    /// Partition converter of this approximator
    ROCPPParConverter_Ptr m_pPartConverter;
    
    /// Map from subset of partition to map from variable in original problem to variable on subset of the partition
    map<string, map<string,ROCPPVarIF_Ptr > > m_VariableMap;  // m_VariableMap: map from partition to map from dv in original problem to dv on partition

    
    /// Use constraints to describe the non anticipativity constraints
    bool m_useExplicitNACs;
    
    /// Value of the big M constant
    double m_bigM;
    
    map<string,uint> m_numPartitionsMap;
    
    /// Name of the folder where to write the text file
    string m_folder;
    
    /// Number of bits used to approximate the real valued variables affecting the uncertainty set
    uint m_numBits;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%% Private Doer Functions %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Initialize the class with the given parameters
    void initialize(ROCPPOptModelIF_Ptr pIn);
    
    
    /// Create the map from partition to map from variable in original problem to variable on partition
    void createVariableMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pMiddle, vector<ROCPPConstraint_Ptr >& vecNACs);
    
    /// Create the map from partition to map from uncertain parameter in original problem to uncertainty on partition
    void createUncertaintyMap(ROCPPOptModelIF_Ptr pIn);
    
    /// Calculate the probability of the given subset of the partition and the means of the uncertain parameters on it
    pair<double, map<string, ROCPPExpr_Ptr > > calculateMeanAndProb(const ROCPPOptModelIF_Ptr pModel, string partition, const map<string,pair<double,double> > &allMap, double allArea);
    
    /// Calculate the value of the objective on the given subset of the partition
    void getStochasticObj(const pair<double, map<string, ROCPPExpr_Ptr > > meanAndProb, const ROCPPObjectiveIF_Ptr oldObj, vector<ROCPPExpr_Ptr > &newObj);
    
    
};

#endif /* PWDecisionRule_hpp */
