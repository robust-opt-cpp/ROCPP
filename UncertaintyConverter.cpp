//
//  UncertaintyConverter.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "UncertaintyConverter.hpp"
#include "IncludeFiles.hpp"
#include "Exceptions.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "OptimizationModel.hpp"
#include "UncertaintyConverter.hpp"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% UNCERTAINTY CONVERTER INTERFACE %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<OptimizationModelIF> UncertaintyConverterIF::doMyThing(boost::shared_ptr<OptimizationModelIF> pIn, bool resetAndSave)
{
    boost::shared_ptr<OptimizationModelIF> pOut = InstanciateModel(pIn->getType(),pIn->getNumTimeStages(),pIn->getObjType());
    
    uncContainer tmpContainer;
    findUncsToTranslate(pIn, tmpContainer);
    
    map<string,boost::shared_ptr<LHSExpression> > translationMap;
    
    createTranslationMap(pIn,pOut,tmpContainer,translationMap);
    
    if (resetAndSave)
        m_translationMap=translationMap;
    
    // now do actual conversion
    for (OptimizationModelIF::constraintIterator c_it = pIn->constraintBegin(); c_it != pIn->constraintBegin(); c_it++ )
    {
        pOut->add_constraint( (*c_it)->mapUncs( translationMap ) );
    }
    
    pOut->set_objective(pIn->getObj() );
    pOut->set_ddu(pIn);
    
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% UNCERTAINTY TO VARIABLE CONVERTER %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

UncToVariableConverter::UncToVariableConverter(const map<string,boost::shared_ptr<DecisionVariableIF> >  &translationMap)
{
    // must convert decision variable to expression
    for (map<string,boost::shared_ptr<DecisionVariableIF> >::const_iterator it = translationMap.begin(); it != translationMap.end(); it++)
    {
        boost::shared_ptr<LHSExpression> expr ( new LHSExpression() );
        (*expr) += it->second;
        
        m_translationMap[it->first] = expr;
    }
}

boost::shared_ptr<ConstraintIF> UncToVariableConverter::doMyThing(boost::shared_ptr<ConstraintIF> pCstr) const
{
    return pCstr->mapUncs( m_translationMap );
}

void UncToVariableConverter::findUncsToTranslate(boost::shared_ptr<OptimizationModelIF> pIn, uncContainer &container)
{
    if (!pIn->isUncertainOptimizationModel())
        throw MyException("would not work");
    
    boost::shared_ptr<UncertainOptimizationModel> pInUnc = boost::static_pointer_cast<UncertainOptimizationModel>(pIn);
    
    
    for (map<string, boost::shared_ptr<LHSExpression> >::const_iterator it = m_translationMap.begin(); it != m_translationMap.end(); it++)
        container += pInUnc->getUnc( it->first );
}

void UncToVariableConverter::createTranslationMap(boost::shared_ptr<OptimizationModelIF> pIn, boost::shared_ptr<OptimizationModelIF> pOut, const uncContainer &tmpContainer, map<string,boost::shared_ptr<LHSExpression> > &translationMap) const
{
    translationMap = m_translationMap;
}
