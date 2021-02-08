//
//  SolverModeller.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "IncludeFiles.hpp"
#include "SolverModeller.hpp"
#include <boost/thread.hpp>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% SOLVER PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void SolverParams::printMainSolverParamsToScreen() const
{
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================== SOLVER PARAMS ================================= " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    
    
    cout << "Lazy NACs: "  << m_useLazyNACs << endl;
    if (m_timeLimit.first)
        cout << "Time Limit:" << m_timeLimit.second << endl;
    if (m_epGapLimit.first)
        cout << "Gap Limit:" << m_epGapLimit.second << endl;
    if (m_epAGapLimit.first)
        cout << "AGap Limit:" << m_epAGapLimit.second << endl;
    if (m_epOptLimit.first)
        cout << "Opt Limit:" << m_epOptLimit.second << endl;
    if (m_epRHSLimit.first)
        cout << "RHS Limit:" << m_epRHSLimit.second << endl;
    if (m_epIntLimit.first)
        cout << "Int Limit:" << m_epIntLimit.second << endl;
    cout << "SOS eps:" << m_SOSeps << endl;
    
    cout << "=========================================================================== " << endl;
    
}

string SolverParams::getColumnTitlesOfOutputDataFile() const
{
    string out("LazyNACs, ");
    out += "TimeLimit";
    return out;
}

string SolverParams::getParams(string delimitter) const
{
    string out(boost::lexical_cast<string>(m_useLazyNACs));
    out += delimitter;
    out += boost::lexical_cast<string>(m_timeLimit.second);
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% SOLVER MODELLER INTERFACE %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double SolverModellerIF::getSolution(string dvName) const
{
    if(m_solution.find(dvName) == m_solution.end())
        throw MyException("Decision variable does not exist");
    
    return m_solution.find(dvName)->second;
}

int SolverModellerIF::getThreadsToRun() const
{
    int numThreads(static_cast<int>( boost::thread::hardware_concurrency() ));

    if (numThreads>2)
        numThreads -= 1;
    else if (numThreads<1)
        numThreads = 1;
    
    
    return numThreads;
}

uint SolverModellerIF::getOptStatus() const
{
    if(!m_problemSolved)
        throw MyException("No problem is solved now");
    
    return m_results.m_optStatus;
}

double SolverModellerIF::getSolvingTime() const
{
    if(!m_problemSolved)
        throw MyException("No problem is solved now");
    
    return m_results.m_solveTime;
}



