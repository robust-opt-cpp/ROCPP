/*
 * ROCPP/SolverModeller.cpp
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

#include "IncludeFiles.hpp"
#include "OptimizationModel.hpp"
#include "SolverModeller.hpp"
#include <thread>

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
    string out(to_string(m_useLazyNACs));
    out += delimitter;
    out += to_string(m_timeLimit.second);
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


bool SolverModellerIF::isApplicable(ROCPPOptModelIF_Ptr pModelIn) const
{
    // basically need to check that this problem can be converted to a deterministic single stage optimization problem
    if (pModelIn->getNumAdaptiveVars()>0)
    {
        cout << "Solver modeller not applicable to problem with adaptive decision variables" << endl;
        return false;
    }
    
    if (pModelIn->getNumUncertainties()>0)
    {
        cout << "Solver modeller not applicable to problem with uncertain parameters" << endl;
        return false;
    }
    
    if (pModelIn->hasNonlinearities())
    {
        cout << "Solver modeller not applicable to problems involving products of decision variables" << endl;
        return false;
    }
    
    return true;
}



double SolverModellerIF::getSolution(string dvName) const
{
    if (m_solution.find(dvName) == m_solution.end())
        throw MyException("Decision variable does not exist");
    
    return m_solution.find(dvName)->second;
}

int SolverModellerIF::getThreadsToRun() const
{
    int numThreads(static_cast<int>( thread::hardware_concurrency() ));

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



