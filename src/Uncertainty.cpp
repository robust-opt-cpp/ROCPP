/*
 * ROCPP/Uncertainty.cpp
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

#include "Uncertainty.hpp"
#include "IncludeFiles.hpp"


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% UNCERTAIN PARAMETER CLASS %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

UncertaintyIF::UncertaintyIF(string name, uint timeStage, bool isObservable) :
m_name(name),
m_timeStage(timeStage),
m_isObservable(isObservable)
{
    if (name.length()==0)
        throw MyException("unc name cannot be empty");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPUnc_Ptr UncertaintyIF::Clone()
{
    ROCPPUnc_Ptr out( new UncertaintyIF( this->getName(), this->getTimeStage(), this->isObservable()) );
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% UNCERTAINTY CONTAINER %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void uncContainer::operator+=(const ROCPPUnc_Ptr& pUncertainty)
{
    // check for uncertain parameter
    string uncName(pUncertainty->getName());
    uncMapType::const_iterator unc_it(m_UncMap.find(uncName));
    
    if ( (unc_it != m_UncMap.end()) && (unc_it->second != pUncertainty) )
        throw MyException("attempted to add two different uncertainties with same name to the same container");
    
    
    if ( unc_it == m_UncMap.end() )
        m_UncMap[uncName]=pUncertainty;
    
    if ( pUncertainty->isObservable() )
        m_ObsUncMap[uncName]=pUncertainty;
    
}

void uncContainer::operator+=(const uncContainer &u)
{
    // iterate through the uncertainties in u
    for (uncMapType::const_iterator u_it = u.m_UncMap.begin(); u_it != u.m_UncMap.end(); u_it++)
        *this += u_it->second;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

uncContainer::const_iterator uncContainer::find(string uncName) const
{
    const_iterator it ( m_UncMap.find( uncName ) );
    return it;
}

uncContainer::const_iterator uncContainer::findInObs(string uncName) const
{
    const_iterator it ( m_ObsUncMap.find( uncName ) );
    return it;
}


uncContainer::const_iterator uncContainer::findthrow(string uncName) const
{
    const_iterator it ( find( uncName ) );
    if (it==end())
        throw MyException("uncertainty not found");
    
    return it;
}


uint uncContainer::getAlphabeticalLocation(string uncName) const
{
    uncMapType::const_iterator m_it( find(uncName) );
    
    if (m_it==end())
        throw MyException("unknown uncertainty");
    
    uncMapType::difference_type diff(0);
    diff = distance(begin(), m_it);
    return (static_cast<uint>(diff));
    
}


uint uncContainer::getObservableAlphabeticalLocation(string uncName) const
{
    uncMapType::const_iterator m_it( findInObs(uncName) );
    
    if (m_it==m_ObsUncMap.end())
        throw MyException("unknown observable uncertainty");
    
    uncMapType::difference_type diff(0);
    diff = distance(m_ObsUncMap.begin(), m_it);
    return (static_cast<uint>(diff));
    
}

bool uncContainer::isObservable(string uncName) const
{
    uncMapType::const_iterator obs_it( findInObs(uncName) );
    uncMapType::const_iterator all_it( find(uncName) );
    
    if (all_it == m_UncMap.end())
        throw MyException("uncertain parameter not found");
    
    if (obs_it== m_ObsUncMap.end())
        return false;
    
    return true;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% CREATE UNC FUNCTION %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPUnc_Ptr createUncertainty(string uncName, uint timeStage, bool isObservable)
{
    ROCPPUnc_Ptr newUnc(new UncertaintyIF(uncName, timeStage, isObservable));
    
    return newUnc;
}
