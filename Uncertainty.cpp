//
//  Uncertainty.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

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

boost::shared_ptr<UncertaintyIF> UncertaintyIF::Clone()
{
    boost::shared_ptr<UncertaintyIF> out( new UncertaintyIF( this->getName(), this->getTimeStage(), this->isObservable()) );
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

void uncContainer::operator+=(const boost::shared_ptr<UncertaintyIF>& pUncertainty)
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
    
    if (m_it==end())
        throw MyException("unknown uncertainty");
    
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

boost::shared_ptr<UncertaintyIF> createUncertainty(string uncName, uint timeStage, bool isObservable)
{
    boost::shared_ptr<UncertaintyIF> newUnc(new UncertaintyIF(uncName, timeStage, isObservable));
    
    return newUnc;
}
