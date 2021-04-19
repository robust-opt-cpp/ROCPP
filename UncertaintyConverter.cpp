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

ROCPPOptModelIF_Ptr UncertaintyConverterIF::convert(ROCPPOptModelIF_Ptr pIn, bool resetAndSave)
{
    ROCPPOptModelIF_Ptr pOut = InstanciateModel(pIn->getType(),pIn->getNumTimeStages(),pIn->getObjType());
    
    uncContainer tmpContainer;
    findUncsToTranslate(pIn, tmpContainer);
    
    map<string,ROCPPExpr_Ptr > translationMap;
    
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

UncToVariableConverter::UncToVariableConverter(const map<string,ROCPPVarIF_Ptr >  &translationMap)
{
    // must convert decision variable to expression
    for (map<string,ROCPPVarIF_Ptr >::const_iterator it = translationMap.begin(); it != translationMap.end(); it++)
    {
        ROCPPExpr_Ptr expr ( new LHSExpression() );
        (*expr) += it->second;
        
        m_translationMap[it->first] = expr;
    }
}

ROCPPConstraint_Ptr UncToVariableConverter::convert(ROCPPConstraint_Ptr pCstr) const
{
    return pCstr->mapUncs( m_translationMap );
}

void UncToVariableConverter::findUncsToTranslate(ROCPPOptModelIF_Ptr pIn, uncContainer &container)
{
    if (!pIn->isUncertainOptimizationModel())
        throw MyException("would not work");
    
    ROCPPUncOptModel_Ptr pInUnc = static_pointer_cast<UncertainOptimizationModel>(pIn);
    
    
    for (map<string, ROCPPExpr_Ptr >::const_iterator it = m_translationMap.begin(); it != m_translationMap.end(); it++)
        container += pInUnc->getUnc( it->first );
}

void UncToVariableConverter::createTranslationMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pOut, const uncContainer &tmpContainer, map<string,ROCPPExpr_Ptr > &translationMap) const
{
    translationMap = m_translationMap;
}



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% UNCERTAINTY TO UNCERSTAINTY CONVERTER %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

UncToUncConverter::UncToUncConverter(const map<string,ROCPPUnc_Ptr >  &translationMap)
{
    // must convert decision variable to expression
    for (map<string,ROCPPUnc_Ptr >::const_iterator it = translationMap.begin(); it != translationMap.end(); it++)
    {
        ROCPPExpr_Ptr expr ( new LHSExpression() );
        (*expr) += it->second;
        
        m_translationMap[it->first] = expr;
    }
}

ROCPPConstraint_Ptr UncToUncConverter::convert(ROCPPConstraint_Ptr pCstr) const
{
    return pCstr->mapUncs( m_translationMap );
}

void UncToUncConverter::findUncsToTranslate(ROCPPOptModelIF_Ptr pIn, uncContainer &container)
{
    if (!pIn->isUncertainOptimizationModel())
        throw MyException("would not work");
    
    ROCPPUncOptModel_Ptr pInUnc = static_pointer_cast<UncertainOptimizationModel>(pIn);
    
    
    for (map<string, ROCPPExpr_Ptr >::const_iterator it = m_translationMap.begin(); it != m_translationMap.end(); it++)
        container += pInUnc->getUnc( it->first );
}

void UncToUncConverter::createTranslationMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pOut, const uncContainer &tmpContainer, map<string,ROCPPExpr_Ptr > &translationMap) const
{
    translationMap = m_translationMap;
}
