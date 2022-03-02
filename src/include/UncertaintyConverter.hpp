/*
 * ROCPP/UncertaintyConverter.hpp
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

#ifndef UncertaintyConverter_hpp
#define UncertaintyConverter_hpp

#include "HeaderIncludeFiles.hpp"
#include <map>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% UNCERTAINTY CONVERTER INTERFACE %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class UncertaintyConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    UncertaintyConverterIF(){}
    
    ~UncertaintyConverterIF(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    typedef map<string,ROCPPExpr_Ptr>::const_iterator const_iterator;
    
    const_iterator find(string varName) const {return m_translationMap.find(varName);}
    
    const_iterator begin() const {return m_translationMap.begin();}
    
    const_iterator end() const {return m_translationMap.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPOptModelIF_Ptr convert(ROCPPOptModelIF_Ptr pIn, bool resetAndSave=false);
    
    virtual void findUncsToTranslate(ROCPPOptModelIF_Ptr pIn, uncContainer &container) = 0;
    
    virtual void createTranslationMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pOut, const uncContainer &tmpContainer, map<string,ROCPPExpr_Ptr> &translationMap) const = 0; // translationMap: map from dv name in original problem to pair of coeff , int var so that dv = sum coeff * int var; //tmpMap: identifies the integer, non-binary variables involved in bilinear terms in pMI_BLP
    
protected:
    
    map<string, ROCPPExpr_Ptr> m_translationMap;
};


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% UNCERTAINTY TO VARIABLE CONVERTER %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class UncToVariableConverter : public UncertaintyConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    UncToVariableConverter(const map<string,ROCPPVarIF_Ptr>  &translationMap);
    
    ~UncToVariableConverter(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPConstraintIF_Ptr convert(ROCPPConstraintIF_Ptr pCstr) const;
    
    void findUncsToTranslate(ROCPPOptModelIF_Ptr pIn, uncContainer &container);
    
    void createTranslationMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pOut, const uncContainer &tmpContainer, map<string,ROCPPExpr_Ptr> &translationMap) const;
};



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% UNCERTAINTY TO UNCERTAINTY CONVERTER %%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class UncToUncConverter : public UncertaintyConverterIF
{
public:
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    UncToUncConverter(const map<string,ROCPPUnc_Ptr>  &translationMap);
    
    ~UncToUncConverter(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ROCPPConstraintIF_Ptr convert(ROCPPConstraintIF_Ptr pCstr) const;
    
    void findUncsToTranslate(ROCPPOptModelIF_Ptr pIn, uncContainer &container);
    
    void createTranslationMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pOut, const uncContainer &tmpContainer, map<string,ROCPPExpr_Ptr> &translationMap) const;
};



#endif /* UncertaintyConverter_hpp */
