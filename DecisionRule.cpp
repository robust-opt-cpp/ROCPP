//
//  DecisionRule.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "IncludeFiles.hpp"
#include "Uncertainty.hpp"
#include "DecisionVariable.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "ObjectiveFunction.hpp"
#include "OptimizationModel.hpp"
#include "VariableConverter.hpp"
#include "OptModelConverters.hpp"
#include "RobustifyEngine.hpp"
#include "helpersOpt.hpp"
#include "FileRelations.hpp"
#include "IndexSetCreator.hpp"
#include "DecisionRule.hpp"
#include <iomanip>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% CONTINOUS VARIABLE APPROXIMATOR INTERFACE %%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ContinuousVarsDRIF::findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
    for (ObjectiveFunctionIF::varsIterator vit = obj->varsBegin(); vit != obj->varsEnd(); vit++)
    {
        if ( (vit->second->isAdaptive()) && (vit->second->isRealVar() ) )
            container += vit->second;
    }
    
    for (vector<ROCPPConstraint_Ptr >::const_iterator cit = first; cit != last; cit++)
    {
        for (ConstraintIF::varsIterator vit = (*cit)->varsBegin(); vit != (*cit)->varsEnd(); vit++)
        {
            if ( (vit->second->isAdaptive()) && (vit->second->isRealVar() ) )
                container += vit->second;
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% LINEAR DECISION RULE APPROXIMATOR %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPOptModelIF_Ptr LinearDecisionRule::convertVar(ROCPPOptModelIF_Ptr pIn, bool resetAndSave)
{
    ROCPPUncOptModel_Ptr pInUnc = static_pointer_cast<UncertainOptimizationModel>(pIn);
    setUncContainer(pInUnc->getUncContainer());
    
    return VariableConverterIF::convertVar(pIn,resetAndSave);
}

void LinearDecisionRule::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd)
{
    if (!m_uncContSet)
        throw MyException("uncertainty container in LDR is not set");
    
    // iterate through variables to convert
    for (dvContainer::const_iterator vit = tmpContainer.begin(); vit != tmpContainer.end(); vit++)
    {
        
        // ---- create expression for LDR
        ROCPPExpr_Ptr ldr ( new LHSExpression() );
        
        // constant term
        ROCPPVarIF_Ptr cst ( new VariableDouble( vit->second->getName() + "_cst" ) );
        (*ldr) += cst;
        
        m_cst.insert(make_pair(vit->second->getName(), cst->getName()));
        
        uint cnt(0);
        
        // iterate through uncertainties in the problem
        for ( uncContainer::const_iterator uit = m_UC->begin(); uit != m_UC->end(); uit++)
        {
            
            if ( (uit->second->isObservable() ) && (uit->second->getTimeStage() <= vit->second->getTimeStage() ) && (uit->second->getTimeStage() + getMemory() > vit->second->getTimeStage() ) )
            {
                ROCPPVarIF_Ptr dv ( new VariableDouble( vit->second->getName() + "_" + uit->second->getName() ) );
                (*ldr) += ROCPPCstrTerm_Ptr (new ProductTerm( 1., uit->second, dv ) );
                cnt++;
                
                m_mapOrigDVUncPairToCoeffDV.insert( make_pair( make_pair(vit->second->getName(), uit->second->getName() ), dv) );
                m_mapOrigDVToUncAndCoeffDV.insert(make_pair(vit->second->getName(), make_pair(uit->second->getName(), dv) ) );
            }
        }
        
        // add expression to translation map
        
        translationMap[vit->second->getName()] = ldr;
        
        // add the bounds (if any) as constraints
        
        double lb ( vit->second->getLB() );
        double ub ( vit->second->getUB() );
        
        if (ub < INFINITY)
        {
            ROCPPClassicConstraint_Ptr cstr ( new IneqConstraint() );
            cstr->add_lhs(ldr->Clone());
            
            bool isZero = (ub >= -10e-4)&&(ub <= 10e-4);
            cstr->set_rhs( make_pair(ub, isZero) );
            
            toAdd.push_back(cstr);
        }
        if (lb > -INFINITY)
        {
            ROCPPClassicConstraint_Ptr cstr ( new IneqConstraint() );
            ROCPPExpr_Ptr expr( ldr->Clone() );
            (*expr) *= -1.;
            cstr->add_lhs(expr);
            
            bool isZero = (lb >= -10e-4)&&(ub <= 10e-4);
            cstr->set_rhs( make_pair(-1.*lb, isZero) );
            
            toAdd.push_back(cstr);
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPVarIF_Ptr LinearDecisionRule::getCoeffDV(string dvName, string uncName) const
{
    map< pair<string,string>, ROCPPVarIF_Ptr >::const_iterator it ( m_mapOrigDVUncPairToCoeffDV.find(make_pair(dvName,uncName)) );
    
    if (it==m_mapOrigDVUncPairToCoeffDV.end())
        throw MyException("this pair " + dvName + " " + uncName + " is not available in the linear decision rule");
    
    return it->second;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void LinearDecisionRule::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &variableValue, ROCPPVarIF_Ptr dv)
{
    string name = dv->getName();
    if(!dv->isAdaptive()){
        cout << "non adaptive: " << name <<" = " << variableValue.find(name)->second << setprecision(4) << endl;
        return;
    }
    
    map<string, ROCPPVarIF_Ptr > expression;
    multimap<string, pair<string, ROCPPVarIF_Ptr > > copy = getLDRExpr();
    
    cout << name <<" = ";
    double value;
    multimap<string, pair<string, ROCPPVarIF_Ptr > >::iterator term = copy.find(name);
    string sign;
    
    while(term != copy.end())
    {
        value = variableValue.find(term->second.second->getName())->second;
        
        if(value >= -10e-4 && value <= 10e-4)
            value = 0.;
        
        if(value >= 0.)
            sign = " + ";
        else
            sign = " - ";
        
        cout << sign << abs(value) << setprecision(4) <<"*"<< term->second.first;
        copy.erase(term);
        term = copy.find(name);
    }
    
    value = variableValue.find(m_cst.find(name)->second)->second;
    
    if(value >= -10e-4 && value <= 10e-4)
        value = 0.;
    
    if(value >= 0.)
        sign = " + ";
    else
        sign = " - ";
    
    cout << sign << abs(value) << setprecision(4) << endl;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% DISCRETE VARIABLE APPROXIMATOR INTERFACE %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void DiscreteVarsDRIF::findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
    for (ObjectiveFunctionIF::varsIterator vit = obj->varsBegin(); vit != obj->varsEnd(); vit++)
    {
        if ( (vit->second->isAdaptive()) && (! (vit->second->isRealVar()) ) )
            container += vit->second;
    }
    
    for (vector<ROCPPConstraint_Ptr >::const_iterator cit = first; cit != last; cit++)
    {
        for (ConstraintIF::varsIterator vit = (*cit)->varsBegin(); vit != (*cit)->varsEnd(); vit++)
        {
            if ( (vit->second->isAdaptive()) && (! (vit->second->isRealVar() ) ) )
                container += vit->second;
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% CONSTANT VARIABLE APPROXIMATOR INTERFACE %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ConstantDecisionRule::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPVarIF_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd)
{
    // iterate through variables to convert
    for (dvContainer::const_iterator vit = tmpContainer.begin(); vit != tmpContainer.end(); vit++)
    {
        //// ---- create expression for CDR (consists of a variable only)
        
        // --- convert adaptive variable to constant
        ROCPPVarIF_Ptr cst;
        
        if (!vit->second->isAdaptive())
            throw MyException("variable should be adaptive");
        
        if (vit->second->isBooleanVar())
            cst = ROCPPVarIF_Ptr( new VariableBool( vit->second->getName(), vit->second->getLB(), vit->second->getUB() ) );
        else if (vit->second->isIntegerVar())
            cst = ROCPPVarIF_Ptr( new VariableInt( vit->second->getName(), vit->second->getLB(), vit->second->getUB() ) );
        
        // add variable to translation map
        translationMap[vit->second->getName()] = cst;
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ConstantDecisionRule::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &variableValue, ROCPPVarIF_Ptr dv)
{
    string name = dv->getName();
    string sign;
    double value = variableValue.find(name)->second;
    
    if (dv->isIntegerVar() || dv->isBooleanVar())
        cout << name << " = " << (int)(round(value)) << endl;
    else
        cout << name << " = " << value << endl;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% TOOL FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


bool allPositive(vector<uint> element)
{
    bool allPos = true;
    
    vector<uint>::const_iterator entry = element.begin();
    for(; entry != element.end(); entry++)
    {
        if ( !( (*entry)-1) ){
            allPos = false;
            break;
        }
    }
    
    return allPos;
}


template <class InputIterator>
double product(InputIterator first, InputIterator last)
{
    double init(1.);
    while (first!=last) {
        init *= *first;  // or: init=binary_op(init,*first) for the binary_op version
        ++first;
    }
    return init;
}

