//
//  UncertaintyConverter.hpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifndef UncertaintyConverter_hpp
#define UncertaintyConverter_hpp

#include "HeaderIncludeFiles.hpp"
#include <map>

class uncContainer;
class LHSExpression;
class ConstraintIF;
class OptimizationModelIF;

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
    
    typedef map<string,boost::shared_ptr<LHSExpression> >::const_iterator const_iterator;
    
    const_iterator find(string varName) const {return m_translationMap.find(varName);}
    
    const_iterator begin() const {return m_translationMap.begin();}
    
    const_iterator end() const {return m_translationMap.end();}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    boost::shared_ptr<OptimizationModelIF> doMyThing(boost::shared_ptr<OptimizationModelIF> pIn, bool resetAndSave=false);
    
    virtual void findUncsToTranslate(boost::shared_ptr<OptimizationModelIF> pIn, uncContainer &container) = 0;
    
    virtual void createTranslationMap(boost::shared_ptr<OptimizationModelIF> pIn, boost::shared_ptr<OptimizationModelIF> pOut, const uncContainer &tmpContainer, map<string,boost::shared_ptr<LHSExpression> > &translationMap) const = 0; // translationMap: map from dv name in original problem to pair of coeff , int var so that dv = sum coeff * int var; //tmpMap: identifies the integer, non-binary variables involved in bilinear terms in pMI_BLP
    
protected:
    
    map<string, boost::shared_ptr<LHSExpression> > m_translationMap;
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
    
    UncToVariableConverter(const map<string,boost::shared_ptr<DecisionVariableIF> >  &translationMap);
    
    ~UncToVariableConverter(){}
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    boost::shared_ptr<ConstraintIF> doMyThing(boost::shared_ptr<ConstraintIF> pCstr) const;
    
    void findUncsToTranslate(boost::shared_ptr<OptimizationModelIF> pIn, uncContainer &container);
    
    void createTranslationMap(boost::shared_ptr<OptimizationModelIF> pIn, boost::shared_ptr<OptimizationModelIF> pOut, const uncContainer &tmpContainer, map<string,boost::shared_ptr<LHSExpression> > &translationMap) const;
};


#endif /* UncertaintyConverter_hpp */
