//
//  KADecisionRule.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 The University of Southern California. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef KADecisionRule_hpp
#define KADecisionRule_hpp

#include <stdio.h>
#include <vector>
#include "HeaderIncludeFiles.hpp"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% K-ADAPTABILITY PARTITION ENCODER %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! K-adapatability partition encorder class
/*!
 Class for encoding the contingency plans in k-adaptability
*/
class KadaptabilityPartitionEncoderMS // used to encode the choice of xi e.g, (k_1,k_2,...,k_T) \in \mathcal K^T
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Construct encoder with different values of K_t in each time stage
    /// @param numPartitionsStr String encoding the number of contingency plans for each time period from 1 to T
    KadaptabilityPartitionEncoderMS(ROCPPOptModelIF_Ptr pIn, string numPartitionsStr);
    
    /// Construct encoder with different values of K_t in each time stage
    /// @param numPartitionsMap Map from each time period to a value of K for that period
    KadaptabilityPartitionEncoderMS(const map<uint,uint> &numPartitionsMap); // maps each time period to a value of K
    
    /// Construct encoder with same K across all time stages
    /// @param T Total number of time stages
    /// @param K Value of K
    KadaptabilityPartitionEncoderMS(uint T, uint K); // same value of K for all time periods
    
    /// Destructor of K-Adaptability partition encoder
    ~KadaptabilityPartitionEncoderMS(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constant iterator for m_mapPartitionNme_to_mapTime_to_k
    typedef map<string, map<uint, uint> >::const_iterator Kmap_iterator;
    
    /// Return the constant iterator pointing to the beginning of the m_mapPartitionNme_to_mapTime_to_k
    Kmap_iterator Kmap_iteratorBegin() const {return m_mapPartitionNme_to_mapTime_to_k.begin();}
    
    /// Return the constant iterator pointing to the end of the m_mapPartitionNme_to_mapTime_to_k
    Kmap_iterator Kmap_iteratorEnd() const {return m_mapPartitionNme_to_mapTime_to_k.end();}
    
    /// Constant iterator for m_mapt_mapPartitionNme_to_mapTime_to_k
    typedef map< uint , map< string, map<uint, uint> > >::const_iterator Klargemap_iterator;
    
    /// Return the constant iterator pointing to the beginning of m_mapt_mapPartitionNme_to_mapTime_to_k
    Klargemap_iterator Klargemap_iteratorBegin() const {return m_mapt_mapPartitionNme_to_mapTime_to_k.begin();}
    
    /// Return the constant iterator pointing to the end of m_mapt_mapPartitionNme_to_mapTime_to_k
    Klargemap_iterator Klargemap_iteratorEnd() const {return m_mapt_mapPartitionNme_to_mapTime_to_k.end();}
    
    /// Constant iterator for m_numPartitionsMap
    typedef map<uint,uint>::const_iterator numPartitionMap_iterator;
    
    /// Return the constant iterator pointing to the beginning of the m_numPartitionsMap
    numPartitionMap_iterator numPartitionMap_iteratorBegin() const {return m_numPartitionsMap.begin();}
    
    /// Return the constant iterator pointing to the end of the m_numPartitionsMap
    numPartitionMap_iterator numPartitionMap_iteratorEnd() const {return m_numPartitionsMap.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Construct the encoder, build the maps
    void getReady();
    
    /// Convert the contingency plan in the input vector to a string
    /// @param partition Vector of number of case for each time stage
    string convertPartitionToString(const vector<uint> &partition) const;
    
    /// Convert the contingency plan in the input map to a string
    /// @param partitionMap Map of time stage to number of case on that partition
    string convertPartitionToString(const map<uint,uint> &partitionMap) const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return the total number of contingency plans for the given time stage
    uint getKt(uint t) const;
    
    /// Return the contingency plan for this time stage for the given contigency plan for the entire planning horizon
    uint getkt(string partition, uint t) const;
    
    /// Return the length of the planning horizon
    uint getT() const {return m_numPartitionsMap.rbegin()->first;};
    
    /// Return the total number of contingency plans
    size_t getKmapSize() const {return m_mapPartitionNme_to_mapTime_to_k.size();}
    
    /// Get the subset of the given partition map up to the given time stage
    string getPartitionSubset(const map<uint,uint> &partitionMap, uint t) const;
    
    /// Get the first subset in this encoder
    string getBasicPartition() const {return m_mapPartitionNme_to_mapTime_to_k.begin()->first;}
    
    string getMaxPartitionString() const {return convertPartitionToString(m_numPartitionsMap);}
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Map from time period to value of K for that time-period
    map<uint,uint> m_numPartitionsMap; // map from time period to value of K for that time-period
    
    /// Map from partition name to map from time period to value of k for that period
    map<string, map<uint, uint> > m_mapPartitionNme_to_mapTime_to_k; // map from partition name to map from time period to value of k for that period
    
    /// Map from time t sub-partition name to map from time period to value of k for that period
    map< uint , map< string, map<uint, uint> > > m_mapt_mapPartitionNme_to_mapTime_to_k; // map from time t sub-partition name to map from time period to value of k for that period
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%% K-adaptability APPROXIMATOR MULTI-STAGE %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// K-adaptability approximator
class KadaptabilityDecisionRule : public DecisionRuleIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of K-Adaptability approximator
    /// @param pIn Model to be approximated
    /// @param K Value of K in each time stage
    /// @param bigM Big-M constant value for linearizing the bilinear terms
    /// @param epsilon Value of epsilon to build the strictly inequality in the constraint uncertainty case
    /// @param folder Name of the folders where to save the log file
    KadaptabilityDecisionRule(ROCPPOptModelIF_Ptr pIn, uint K, double bigM = 100.0, double epsilon = 0.0001, string folder=" ");
    
    /// Constructor of K-Adaptability approximator
    /// @param pPartitionEncoder predefined partition encoder for the model to be approximated
    KadaptabilityDecisionRule(ROCPPKadaptEncoder_Ptr pPartitionEncoder, double bigM = 100.0, double epsilon = 0.0001, string folder=" ");
    
    KadaptabilityDecisionRule(ROCPPKadaptEncoder_Ptr pPartitionEncoder, ROCPPBilinearReform_Ptr pBTR, double epsilon = 0.0001, string folder=" ") : m_pPartitionEncoder(pPartitionEncoder), m_pBTR(pBTR), m_folder(folder), m_epsilon(epsilon) {}
    
    /// Destructor of K-Adaptability approximator
    ~KadaptabilityDecisionRule(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Check the compatability of the given model
    /// @warning Only binary variables are allowed in the K-adaptability approximator
    //void checkCompatability(ROCPPOptModelIF_Ptr pIn);
    
    /// Check the compatability of the given warm start map
    /// @note the value of K in each time stage must be less than or equal to that in the original approximator; at least one K must be strictly less than the one in the original problem
    void checkWsCompatability(const map<uint,uint> &wsMap);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Create the map from the original decision variable and uncertainty to the new variable and uncertainty for each plan and pair of partition and time stage respectively
    void createVariableAndUncMap(dvContainer::const_iterator varsBegin, dvContainer::const_iterator varsEnd, uncContainer::const_iterator uncBegin, uncContainer::const_iterator uncEnd);
    
    /// Check the compatability of the input model and check whether the model has constraint uncertainty to choose the appropriate method
    ROCPPOptModelIF_Ptr approximate(ROCPPOptModelIF_Ptr pIn);
    
    bool isApplicable(ROCPPOptModelIF_Ptr pIn) const;
    
    /// Return a new model after fixing second stage variables which are solved by the warm start problem to the value provided in the input map
    /// @param pIn Model before approximating
    /// @param pKadaptModel Model after approximating
    /// @param warmStartResults Results of the warm start problem
    /// @param wsMap Map from time stage to number of k in the warm start problem
    ROCPPOptModelIF_Ptr fixSecondStageVariablesToWarmStart(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pKadaptModel, const map<string,double> &warmStartResults, const map<uint,uint> &wsMap);
    
    /// Return a new model after fixing second stage variables to the value provided in the input map
    ROCPPOptModelIF_Ptr fixBinaryVariableValues(ROCPPOptModelIF_Ptr pKadaptModel, const map<string,bool> &varValues) const;
    
    virtual ROCPPOptModelIF_Ptr addProblemSpecificConstraints(ROCPPOptModelIF_Ptr pKadapt) const {return pKadapt;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    string getSolutionApproachParameters(string delimiter) const;
    
    /// Get the new variable associated with the given original variable on the given contingency plan
    ROCPPVarIF_Ptr getDVonPartition(string partition, string OrigDVname) const;
    
    /// Get the new uncertain parameter associated with the given original uncertainty on the given contingency plan at time stage t
    ROCPPUnc_Ptr getUncOnPartition(string OrigUncName, string partition, uint t) const;
    
    //map<string, double> getWsSolutions(ROCPPOptModelIF_Ptr pIn, const map<string, double> &warmStartResults, uint K) const;
    
    /// Return the warm start solution of this problem by setting the variables which exist in the warm start problem to their solutions and assigning the variables which don't exist in the warm start problem to the biggest solved kth solutions
    /// @param pIn Model before approximating
    /// @param warmStartResults Results of the warm start problem
    /// @param wsMap Map from time stage to number of k in the warm start problem
    /// @param wsSolutions Map from variable name to its warm start solution for all variables in the problem to be solved
    void getWsSolutions(ROCPPOptModelIF_Ptr pIn, const map<string, double> &warmStartResults, const map<uint,uint> &wsMap, map<string, double> &wsSolutions);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void printParametersToScreen() const;
    
    /// Print out the solution for the given variable for the given plan
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv, string partition);
    
    /// Print out the solution for the given variable for all plans
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv);
    
    /// Print out whether observe or not the given uncertainty for the given plan
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc, string partition);
    
    /// Print out whether we observe or not the given uncertainty for all cases
    void printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc);
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Protected Methods %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Method for multistage model with only objective uncertainty
    /// @warning No uncertainty in constraint is allowed here
    ROCPPOptModelIF_Ptr approxObjUnc(ROCPPOptModelIF_Ptr pIn);
    
    /// Method for two stage model with both objective and constraint uncertainty
    /// @warning Only two stage model is allowed here
    ROCPPOptModelIF_Ptr approxCstrUnc(ROCPPOptModelIF_Ptr pIn);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPKadaptEncoder_Ptr m_pPartitionEncoder;
    
    /// Map from original decision variable name to partition string to new decision variable
    map<string, map<string, ROCPPVarIF_Ptr> > m_DVmap; // map from original decision variable name to partition string to new decision variable
    
    /// Map from partition to map from original variable name to new variable on that partition
    map< string, map<string, ROCPPVarIF_Ptr> > m_mapPartitionEnc_mapOrigDVtoDVonPartition;
    
    /// Map from pair of partition and time stage to the map from uncertainty name to the uncertainty on that partition at specific time stage
    map< pair<string,uint> , map<string, ROCPPUnc_Ptr> > m_mapPartitionEncandt_mapOrigUnctoUnconPartition;
    
    ROCPPBilinearReform_Ptr m_pBTR;
    
    /// Small values used to deal with the infeasible constraint in constraint uncertainty
    double m_epsilon;
    
    string m_folder;
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% TOOL FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void createAllPartitionsKadapt(vector<vector<uint> > &BPconfigs, uint T, const map<uint, uint> &numPartitionsMap);

void appendPartitionsKadapt(vector<vector<uint> > &BPconfigs, uint t, const map<uint, uint> &numPartitionsMap);
#endif /* KADecisionRule_hpp */
