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
    PiecewiseApproximator(ROCPPOptModelIF_Ptr pIn, string numPartitionsStr, uint numBits = 5, double bigM = 100., bool useExplicitNACs = false, string folder=" ");
    
    /// Constructor of the PiecewiseApproximator class
    /// @param numPartitionsMap map from the name of the uncertain parameter to the number of subsets along that direction (number of breakpoints + 1)
    PiecewiseApproximator(ROCPPOptModelIF_Ptr pIn, const map<string,uint> &numPartitionsMap = map<string, uint>(),  uint numBits = 5, double bigM = 100., bool useExplicitNACs = false, string folder=" ");
    
    /// Constructor of the PiecewiseApproximator class
    /// Directly initialize with the predefined private members (for greater customization)
    PiecewiseApproximator(ROCPPContinuousVarsDR_Ptr pCVA, ROCPPDiscreteVarsDR_Ptr pDVA,
                          ROCPPParConstructor_Ptr pPartConstructor, ROCPPParConverter_Ptr pPartConverter,
                          ROCPPMItoMB_Ptr pMItoMB_Bilinear, ROCPPBilinearReform_Ptr pBTR,
                          ROCPPUncSetRealVarApprox_Ptr pUSRVA, ROCPPUncSetRealVarApprox_Ptr pBPA,
                          uint numBits = 5, double bigM = 100., bool useExplicitNACs = false, string folder=" ") :
    m_pCVA(pCVA), m_pDVA(pDVA), m_pPartConstructor(pPartConstructor), m_pPartConverter(pPartConverter), m_pBTR(pBTR), m_pMItoMB_Bilinear(pMItoMB_Bilinear), m_pUSRVA(pUSRVA), m_pBPA(pBPA), m_numBits(numBits), m_bigM(bigM), m_useExplicitNACs(useExplicitNACs), m_folder(folder) {}
    
    /// Destructor of the PiecewiseApproximator class
    ~PiecewiseApproximator(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Initialize the class with the given parameters
    void InitializeMe(ROCPPOptModelIF_Ptr pIn, const map<string,uint> &numPartitionsMap, uint numBits, double bigM, bool useExplicitNACs, string folder);
    
    /// Create the map from partition to map from variable in original problem to variable on partition
    void createVariableMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pMiddle, vector<ROCPPConstraintIF_Ptr>& vecNACs);
    
    /// Approximate the input model
    /// First construct the partition maps, then calculate and robustity the model on each subset, finally add non-anticipativity constraints
    ROCPPMISOCP_Ptr approximate(ROCPPOptModelIF_Ptr pIn);
    
    
    /// Return a new model after fixing the binary variables to the value provided in the input map
    ROCPPOptModelIF_Ptr fixBinaryVariableValues(ROCPPOptModelIF_Ptr pKadaptModel, const map<string,bool> &varValues) const;
    
    /// Calculate the probability of the given subset of the partition and the means of the uncertain parameters on it
    pair<double, map<string, ROCPPExpr_Ptr> > calculateMeanAndProb(const ROCPPOptModelIF_Ptr pModel, string partition, const map<string,pair<double,double> > &allMap, double allArea);
    
    /// Calculate the value of the objective on the given subset of the partition
    void getStochasticObj(const pair<double, map<string, ROCPPExpr_Ptr> > meanAndProb, const ROCPPObjectiveIF_Ptr oldObj, vector<ROCPPExpr_Ptr> &newObj);
    
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    string getSolutionApproachParameters(string delimiter) const;
    
    /// Find the variable that new decision variable originated from after LDR and CDR
    pair<bool,ROCPPVarIF_Ptr> findOrigVariable(ROCPPVarIF_Ptr newdv) const; // find the time stage of the variable that newdv originated from after LDR and CDR
    
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
    
    /// Partition constructor of this approximator
    ROCPPParConstructor_Ptr m_pPartConstructor;
    
    /// Partition converter of this approximator
    ROCPPParConverter_Ptr m_pPartConverter;
    ROCPPBilinearReform_Ptr m_pBTR;
    ROCPPMItoMB_Ptr m_pMItoMB_Bilinear;
    ROCPPUncSetRealVarApprox_Ptr m_pUSRVA;
    ROCPPUncSetRealVarApprox_Ptr m_pBPA;
    
    /// Map from subset of partition to map from variable in original problem to variable on subset of the partition
    map<string, map<string,ROCPPVarIF_Ptr> > m_VariableMap;  // m_VariableMap: map from partition to map from dv in original problem to dv on partition
    
    /// Use constraints to describe the non anticipativity constraints
    bool m_useExplicitNACs;
    
    /// Name of the folder where to write the text file
    string m_folder;
    
    /// Value of the big M constant
    double m_bigM;
    
    /// Number of bits used to approximate the real valued variables affecting the uncertainty set
    uint m_numBits;
};


#endif /* DDUApproximatorPW_hpp */
