//
//  DDUApproximatorPW.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef DDUApproximatorPW_hpp
#define DDUApproximatorPW_hpp

#include <stdio.h>


#include "HeaderIncludeFiles.hpp"

class MISOCP;
class DDUOptimizationModel;
class DDUApproximatorIF;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% PIECEWISE APPROXIMATOR %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Piecewise approximator
/*!
 Class for approximating the adaptive decisions in a model by piecewise linear and piecewise constant decision rules
*/
class PiecewiseApproximator : public DDUApproximatorIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the PiecewiseApproximator class
    /// @param pIn Model to be approximated
    /// @param numPartitionsStr String to encode the number of subsets for each observable uncertainty, in alphabetical order
    /// @param numBits Number of bits used for discretizing any real-valued variables affecting the uncertainty set
    /// @param bigM Big-M value used for linearizing the bilinear terms
    /// @param useExplicitNACs If true, then two meansurement variables are equal, otherwise use the same measurement variables for the NACs
    /// @param folder Name of folders where to store the log file
    PiecewiseApproximator(boost::shared_ptr<OptimizationModelIF> pIn, string numPartitionsStr, uint numBits = 5, double bigM = 100., bool useExplicitNACs = false, string folder=" ");
    
    /// Constructor of the PiecewiseApproximator class
    /// @param numPartitionsMap map from the name of the uncertain parameter to the number of subsets along that direction (number of breakpoints + 1)
    PiecewiseApproximator(boost::shared_ptr<OptimizationModelIF> pIn, const map<string,uint> &numPartitionsMap = map<string, uint>(),  uint numBits = 5, double bigM = 100., bool useExplicitNACs = false, string folder=" ");
    
    /// Constructor of the PiecewiseApproximator class
    /// Directly initialize with the predefined private members (for greater customization)
    PiecewiseApproximator(boost::shared_ptr<ContinuousVarsDRIF> pCVA, boost::shared_ptr<DiscreteVarsDRIF> pDVA,
                          boost::shared_ptr<PartitionConstructorIF> pPartConstructor, boost::shared_ptr<PartitionConverter> pPartConverter,
                          boost::shared_ptr<Bilinear_MItoMB_Converter> pMItoMB_Bilinear, boost::shared_ptr<BilinearTermReformulatorIF> pBTR,
                          boost::shared_ptr<UncertaintySetRealVarApproximator> pUSRVA, boost::shared_ptr<UncertaintySetRealVarApproximator> pBPA,
                          uint numBits = 5, double bigM = 100., bool useExplicitNACs = false, string folder=" ") :
    m_pCVA(pCVA), m_pDVA(pDVA), m_pPartConstructor(pPartConstructor), m_pPartConverter(pPartConverter), m_pBTR(pBTR), m_pMItoMB_Bilinear(pMItoMB_Bilinear), m_pUSRVA(pUSRVA), m_pBPA(pBPA), m_numBits(numBits), m_bigM(bigM), m_useExplicitNACs(useExplicitNACs), m_folder(folder) {}
    
    /// Destructor of the PiecewiseApproximator class
    ~PiecewiseApproximator(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Initialize the class with the given parameters
    void InitializeMe(boost::shared_ptr<OptimizationModelIF> pIn, const map<string,uint> &numPartitionsMap, uint numBits, double bigM, bool useExplicitNACs, string folder);
    
    /// Create the map from partition to map from variable in original problem to variable on partition
    void createVariableMap(boost::shared_ptr<OptimizationModelIF> pIn, boost::shared_ptr<OptimizationModelIF> pMiddle, vector<boost::shared_ptr<ConstraintIF> >& vecNACs);
    
    /// Approximate the input model
    /// First construct the partition maps, then calculate and robustity the model on each subset, finally add non-anticipativity constraints
    boost::shared_ptr<MISOCP> DoMyThing(boost::shared_ptr<OptimizationModelIF> pIn);
    
    
    /// Return a new model after fixing the binary variables to the value provided in the input map
    boost::shared_ptr<OptimizationModelIF> fixBinaryVariableValues(boost::shared_ptr<OptimizationModelIF> pKadaptModel, const map<string,bool> &varValues) const;
    
    /// Calculate the probability of the given subset of the partition and the means of the uncertain parameters on it
    pair<double, map<string, boost::shared_ptr<LHSExpression> > > calculateMeanAndProb(const boost::shared_ptr<OptimizationModelIF> pModel, string partition, const map<string,pair<double,double> > &allMap, double allArea);
    
    /// Calculate the value of the objective on the given subset of the partition
    void getStochasticObj(const pair<double, map<string, boost::shared_ptr<LHSExpression> > > meanAndProb, const boost::shared_ptr<ObjectiveFunctionIF> oldObj, vector<boost::shared_ptr<LHSExpression> > &newObj);
    
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    string getSolutionApproachParameters(string delimiter) const;
    
    /// Find the variable that new decision variable originated from after LDR and CDR
    pair<bool,boost::shared_ptr<DecisionVariableIF> > findOrigVariable(boost::shared_ptr<DecisionVariableIF> newdv) const; // find the time stage of the variable that newdv originated from after LDR and CDR
    
    /// Get the decision variable on the the given subset of the partition for the given original variable
    boost::shared_ptr<DecisionVariableIF> getVarOnPartition(string partition, string origVarName) const;
    
    /// Calculate solution for the given variable on the given subset of the partition
    void calculateSolution(boost::shared_ptr<OptimizationModelIF> pIn, const map<string, double> &resultIn, boost::shared_ptr<DecisionVariableIF> dv, string partition);
    
    /// Calculate the measurement variable value for the given uncertainty on the given subset of the partition
    void calculateSolution(boost::shared_ptr<OptimizationModelIF> pIn, const map<string, double> &resultIn, boost::shared_ptr<UncertaintyIF> unc, string partition);
    
    /// Return the warm start solution of this problem by setting the variables which exist in the warm start problem to their solutions and assigning the variables which do not exist in the warm start problem to the solutions on the nearest partition
    map<string, double> getWsSolutions(boost::shared_ptr<OptimizationModelIF> pIn, const map<string, double> &warmStartResults, const map<string,uint> &wsMap) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void printParametersToScreen() const;
    
    /// Print out the solution for the given variable on the given subset of the partition
    void printOut(const boost::shared_ptr<OptimizationModelIF> pIn, const map<string, double> &resultIn, boost::shared_ptr<DecisionVariableIF> dv,  map<boost::shared_ptr<UncertaintyIF>, uint> partitionIn);
    
    /// Print out the solution for the given variable on all subsets of the the partition
    void printOut(const boost::shared_ptr<OptimizationModelIF> pIn, const map<string, double> &resultIn, boost::shared_ptr<DecisionVariableIF> dv);
    
    /// Print out whether observe or not the given uncertainty on the given subset of the partition
    void printOut(const boost::shared_ptr<OptimizationModelIF> pIn, const map<string, double> &resultIn, boost::shared_ptr<UncertaintyIF> unc,  map<boost::shared_ptr<UncertaintyIF>, uint> partitionIn);
    
    /// Print out whether we observe or not the given uncertainty on all subsets of the partition
    void printOut(const boost::shared_ptr<OptimizationModelIF> pIn, const map<string, double> &resultIn, boost::shared_ptr<UncertaintyIF> unc);
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// String encoding the number of subsets in the partition for every observable uncertainty in the problem, in alphabetical order
    string m_numPartitionsStr;
    
    /// Linear decision rule converter
    boost::shared_ptr<ContinuousVarsDRIF> m_pCVA;
    
    /// Constant decision rule converter
    boost::shared_ptr<DiscreteVarsDRIF> m_pDVA;
    
    /// Partition constructor of this approximator
    boost::shared_ptr<PartitionConstructorIF> m_pPartConstructor;
    
    /// Partition converter of this approximator
    boost::shared_ptr<PartitionConverter> m_pPartConverter;
    boost::shared_ptr<BilinearTermReformulatorIF> m_pBTR;
    boost::shared_ptr<Bilinear_MItoMB_Converter> m_pMItoMB_Bilinear;
    boost::shared_ptr<UncertaintySetRealVarApproximator> m_pUSRVA;
    boost::shared_ptr<UncertaintySetRealVarApproximator> m_pBPA;
    
    /// Map from subset of partition to map from variable in original problem to variable on subset of the partition
    map<string, map<string,boost::shared_ptr<DecisionVariableIF> > > m_VariableMap;  // m_VariableMap: map from partition to map from dv in original problem to dv on partition
    
    /// Use constraints to describe the non anticipativity constraints
    bool m_useExplicitNACs;
    
    /// Name of the folder where to write the text file
    string m_folder;
    
    /// Value of the big M constant
    double m_bigM;
    
    /// Number of bits used to approximate the real valued variables affecting the uncertainty set
    uint m_numBits;
};



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% DDU TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

typedef PiecewiseApproximator ROCPPPiecewiseApprox;
typedef boost::shared_ptr<ROCPPPiecewiseApprox> ROCPPPiecewiseApprox_Ptr;


#endif /* DDUApproximatorPW_hpp */
