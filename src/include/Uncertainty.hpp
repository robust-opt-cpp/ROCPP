/*
 * ROCPP/Uncertainty.hpp
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

#ifndef Uncertainty_hpp
#define Uncertainty_hpp

#include "HeaderIncludeFiles.hpp"
#include <map>



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% UNCERTAIN PARAMETER CLASS %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/// Uncertain parameter class
class UncertaintyIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Constructor of the UncertaintyIF class
    /// @param name name of the uncertain parameter
    /// @param timeStage time-stage of the uncertain parameter (time when it is revealed)
    /// @param isObservable true if and only if the uncertain parameter is observable
    UncertaintyIF(string name, uint timeStage = 1, bool isObservable = true);
    
    /// Destructor of the UncertaintyIF class
    virtual ~UncertaintyIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the uncertain parameter name
    /// @return name of the uncertain parameter, string type
    string getName() const {return m_name;}
    
    /// Return the time-stage when the uncertain parameter is revealed
    /// @return time-stage, uint type
    uint getTimeStage() const {return m_timeStage;}
    
    /// Return indicator of observability, return value equals 1 if and only if the uncertain parameter can be observed
    /// @return observability indicator, boolean type
    /// @note If an uncertain parameter is observable and its time-stage is t, then all decisions taken on or after time t will be allowed to depend on this uncertain parameter
    bool isObservable() const {return m_isObservable;}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Return a copy of the uncertain parameter
    ROCPPUnc_Ptr Clone();
    
private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Name of the uncertain parameter
    string m_name;
    
    /// Time-stage of the uncertain parameter
    /// @note The time-stage only matters in the case of observable uncertain parameters. If an uncertain parameter is not observable, then its time-stage is not used.
    uint m_timeStage;
    
    /// Indicator of observability. It is equal to true if and only if the uncertain parameter can be observed
    bool m_isObservable;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% UNCERTAINTY CONTAINER %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//! Uncertainty container
/*!
 Used to store uncertain parameters for class ConstraintTermIF, LHSExpression, etc.
*/
 class uncContainer
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    uncContainer(){}
    ~uncContainer(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Add uncertain parameters from uncertainty container a into this container
    ///
    /// @note Uncertain parameters that already exist will be ignored. If an uncertain parameter with the same name already exists in this container, an error will be thrown.
    void operator+=(const uncContainer &a);
    
    /// Add uncertain parameter unc to this container
    ///
    /// @note If the uncertainty already exists, it will be ignored. If an uncertain parameter with the same name already exists in this container, an error will be thrown.
    void operator+=(const ROCPPUnc_Ptr& unc);
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /// Constant iterator for decision variable map
    typedef uncMapType::const_iterator const_iterator;
    
    /// Return a constant iterator pointing to the beginning of the uncertain parameter map
    const_iterator begin() const {return m_UncMap.begin();}
    
    /// Return a constant iterator pointing to the end of the uncertain parameter map
    const_iterator end() const {return m_UncMap.end();}
    
    /// Return a constant iterator pointing to the uncertain parameter called uncName
    const_iterator find(string uncName) const;
    
    /// Find the uncertain parameter called uncName in the map for observable uncertain parameters
    const_iterator findInObs(string uncName) const;
    
    /// Find the uncertain parameter called uncName in this container.
    /// If there is no such uncertainty, an exception will be thrown.
    const_iterator findthrow(string uncName) const;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Get the number of uncertain parameters in this container
    size_t getNumUncertainties() const {return m_UncMap.size();}
    
    /// Get the number of observable uncertain parameters in this container
    size_t getNumObsUncertainties() const {return m_ObsUncMap.size();}
    
    /// Get the alphabetical location of this uncertain parameter in this container
    uint getAlphabeticalLocation(string uncName) const;
    
    /// Get the alphabetical location of this uncertain parameter in this container among the observable uncertain parameters
    uint getObservableAlphabeticalLocation(string uncName) const;
    
    /// Return true if and only if the uncertain parameter is observable
    bool isObservable(string uncName) const;
    
    /// Return true if and only if this container is empty
    bool empty() const {return m_UncMap.empty();}
    

private:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%% Private Members %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    /// Map from uncertain parameter name to uncertain parameter
    uncMapType m_UncMap;
    
    /// Map from uncertain parameter name to observable uncertain parameter
    uncMapType m_ObsUncMap;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% CREATE UNC FUNCTION %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


/// Function used to instanciate an uncertain parameter
/// @param uncName name of the uncertain parameter
/// @param timeStage time-stage of the uncertain parameter
/// @param isObservable true if and only if the uncertain parameter is observable
ROCPPUnc_Ptr createUncertainty(string uncName, uint timeStage = 1, bool isObservable = true);


#endif /* Uncertainty_hpp */
