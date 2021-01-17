//
//  SolverModeller.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef SolverModeller_hpp
#define SolverModeller_hpp

class OptimizationModelIF;


#include "HeaderIncludeFiles.hpp"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% SOLVER PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Class to store the solver parameters
class SolverParams
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the SolverParams class
    /// @param useLazyNACs indicates if the decision-dependent non-anticipativity constraints will be added as lazy constraints
    /// @param verbose indicates if the detail of the solver iterations will be displayed
    /// @param epIntLimit first parameter indicates if an integer feasibility tolerance is set, second parameter indicates the value of the limit
    /// @param timeLimit first parameter indicates if a time limit is set, second parameter indicates the value of the limit
    /// @param epGapLimit first parameter indicates if an MIP gap is set, second parameter indicates the value of the gap
    /// @param epAGapLimit first parameter indicates if an MIP absolute gap is set, second parameter indicates the value of the gap
    /// @param epOptLimit first parameter indicates if an optimality tolerance is set, second parameter indicates the value of the tolerance
    /// @param epRHSLimit first parameter indicates if a feasibility tolerance is set, second parameter indicates the value of the tolerance
    SolverParams(bool useLazyNACs=false,bool verbose = true, pair<bool,double> epIntLimit = make_pair(false,0.), pair<bool,double> timeLimit = make_pair(false,0.), pair<bool,double> epGapLimit = make_pair(false,1.e-4), pair<bool,double> epAGapLimit = make_pair(false,1.e-10),
                 pair<bool,double> epOptLimit = make_pair(false,1.e-6), pair<bool,double> epRHSLimit = make_pair(false,1.e-9), double SOSeps = 1.e-20 ):
    m_useLazyNACs(useLazyNACs),
    m_verbose(verbose),
    m_timeLimit(timeLimit),
    m_epGapLimit(epGapLimit),
    m_epAGapLimit(epAGapLimit),
    m_epOptLimit(epOptLimit),
    m_epRHSLimit(epRHSLimit),
    m_epIntLimit(epIntLimit),
    m_SOSeps(SOSeps)
    {}
    ~SolverParams(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    void setTimeLimit(pair<bool,double> timeLimit){m_timeLimit=timeLimit;}
    void setEpGapLimit(pair<bool,double> epGapLimit){m_epGapLimit=epGapLimit;}
    void setEpIntLimit(pair<bool,double> epIntLimit){m_epIntLimit=epIntLimit;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    void printMainSolverParamsToScreen() const;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    string getColumnTitlesOfOutputDataFile() const;
    string getParams(string delimitter) const;
    
    pair<bool,double> getTimeLimit() const {return m_timeLimit;}
    pair<bool,double> getEpGapLimit() const {return m_epGapLimit;}
    pair<bool,double> getEpAGapLimit() const {return m_epAGapLimit;}
    pair<bool,double> getEpOptLimit() const {return m_epOptLimit;}
    pair<bool,double> getEpRHSLimit() const {return m_epRHSLimit;}
    pair<bool,double> getEpIntLimit() const {return m_epIntLimit;}
    double getSOSeps() const {return m_SOSeps;}
    bool getVerbose() const {return m_verbose;}
    bool useLazyNACs() const {return m_useLazyNACs;}
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Private Member %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    bool m_useLazyNACs;
    pair<bool,double> m_timeLimit;
    pair<bool,double> m_epGapLimit;
    pair<bool,double> m_epAGapLimit;
    pair<bool,double> m_epOptLimit;
    pair<bool,double> m_epRHSLimit;
    pair<bool,double> m_epIntLimit;
    double m_SOSeps;
    bool m_verbose;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% RESULT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Class to store the retults of a solver
struct resultParams
{
    /// Constructor of the resultParams structure
    resultParams() : m_optStatus(0), m_optValue(0.0), m_isMIP(true), m_MIPGap(0.0), m_solveTime(0.0) {}
    
    /// Optimality status
    uint m_optStatus;
    
    /// Optimal value
    double m_optValue;
    
    /// Indicates whether the problem is an MIP or not
    bool m_isMIP;
    
    /// Value of the MIP gap
    double m_MIPGap;
    
    /// Solver time
    double m_solveTime;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% SOLVER MODELLER INTERFACE %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Class used to interface with deterministic optimization solvers
class SolverModellerIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the SolverModellerIF class
    SolverModellerIF(const SolverParams &pSParams):
    m_pSParams(pSParams), m_problemSolved(false), m_results(resultParams() ) {}
    
    /// Destructor of the SolverModellerIF class
    virtual ~SolverModellerIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Solver Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    virtual void solve(boost::shared_ptr<OptimizationModelIF> pModelIn, bool writeSlnToFile = false, string fileName = "", bool writeSlnToConsole = true, const map<string, double>& WSvars = (map<string,double>()), const map<string,int>& priorities = (map<string,int>()), bool deleteModel=false) = 0;
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    pair<bool,double> getTimeLimit() const {return m_pSParams.getTimeLimit();}
    pair<bool,double> getEpGapLimit() const {return m_pSParams.getEpGapLimit();}
    pair<bool,double> getEpAGapLimit() const {return m_pSParams.getEpAGapLimit();}
    pair<bool,double> getEpOptLimit() const {return m_pSParams.getEpOptLimit();}
    pair<bool,double> getEpRHSLimit() const {return m_pSParams.getEpRHSLimit();}
    pair<bool,double> getEpIntLimit() const {return m_pSParams.getEpIntLimit();}
    double getSOSeps() const {return m_pSParams.getSOSeps();}
    map<string, double> getSolution() const {return m_solution;}
    double getSolution(string dvName) const;
    
    uint getOptStatus() const;
    double getOptValue() const;
    double getMIPGap() const;
    double getSolvingTime() const;
    
protected:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    bool m_problemSolved;
    SolverParams m_pSParams;
    map<string, double> m_solution;
    resultParams m_results;
    
    int getThreadsToRun() const;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% CREATE SOLVER FUNCTION %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<SolverModellerIF> InstantiateSolverModeller( string solver, const SolverParams &pSParams, bool useLazyNACs=false  );

boost::shared_ptr<SolverModellerIF> InstantiateSolverModeller( string solver, bool useLazyNACs=false );



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% SOLVER MODELLER TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

typedef SolverModellerIF ROCPPSolver;
typedef boost::shared_ptr<SolverModellerIF> ROCPPSolver_Ptr;

typedef SolverParams ROCPPSolverParams;
typedef boost::shared_ptr<ROCPPSolverParams> ROCPPSolverParams_Ptr;

#endif /* SolverModeller_hpp */
