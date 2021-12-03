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
#include "ReformulationOrchestrator.hpp"
#include "VariableConverter.hpp"
#include "OptModelConverters.hpp"
#include "RobustifyEngine.hpp"
#include "helpersOpt.hpp"
#include "FileRelations.hpp"
#include "IndexSetCreator.hpp"
#include "DecisionRule.hpp"
#include <iomanip>
#include <chrono>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% CONTINOUS VARIABLE APPROXIMATOR INTERFACE %%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ContinuousVarsDRIF::findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
    for (ObjectiveFunctionIF::varsIterator vit = obj->varsBegin(); vit != obj->varsEnd(); vit++)
    {
        if ( (vit->second->isAdaptive()) && (vit->second->isRealVar() ) )
            container += vit->second;
    }
    
    for (vector<ROCPPConstraintIF_Ptr>::const_iterator cit = first; cit != last; cit++)
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

bool LinearDecisionRule::isApplicable(ROCPPOptModelIF_Ptr pIn) const
{
    if (!pIn->isUncertainOptimizationModel())
    {
        cout << "Cannot apply LinearDecisionRule to deterministic model" << endl;
        return false;
    }
    
    if ((pIn->getObjType() == stochastic) && ( (!pIn->hasRectangularUncertaintySet()) || (pIn->hasDecisionDependentUncertaintySet()) ) )
    {
        cout << "LinearDecisionRule approximator is only applicable to stochastic problems with rectangular and decision-independent uncertainty set or to robust problems" << endl;
        return false;
    }
    
    if (pIn->hasRealVarsInUncertaintySet())
    {
        cout << "LinearDecisionRule approximator does not apply to problems with uncertainty set affected by real valued decisions" << endl;
        return false;
    }
    
    vector<ROCPPExpr_Ptr> objs(pIn->getObj()->getObj());
    
    vector<ROCPPExpr_Ptr>::const_iterator obj(objs.begin());
    
    for(; obj != objs.end(); obj++)
    {
        if((*obj)->hasProdsUncertainties())
        {
            cout << "Cannot deal with products of uncertainties at this time" << endl;
            return false;
        }
    }
    
    if ( (pIn->hasNonlinearities()) ) // here also will need to check if we have real valued decisions affecting the uncertainty set
    {
        cout << "Cannot handle nonlinear problem at present" << endl;
        return false;
    }
    
    return true;
}

ROCPPOptModelIF_Ptr LinearDecisionRule::convertVar(ROCPPOptModelIF_Ptr pIn, bool resetAndSave)
{
    ROCPPUncOptModel_Ptr pInUnc = static_pointer_cast<UncertainOptimizationModel>(pIn);
    setUncContainer(pInUnc->getUncContainer());
    
    return VariableConverterIF::convertVar(pIn,resetAndSave);
}

ROCPPOptModelIF_Ptr LinearDecisionRule::approximate(ROCPPOptModelIF_Ptr pIn)
{
    cout << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "======================== APPROXIMATING USING LDR ========================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    auto start = chrono::high_resolution_clock::now();
    
    ROCPPOptModelIF_Ptr pModel(pIn->Clone());
    
    if (pIn->getObjType() == robust)
        pModel->add_epigraph();
    
    // do linear decision rule
    ROCPPOptModelIF_Ptr pLDRModel(convertVar(pModel, true) );


    if ( pLDRModel->getNumAdaptiveContVars() != 0 )
        throw MyException("Adaptive continous variables should have been eliminated by now");
    
    if (pIn->getObjType() == stochastic)
        pLDRModel->getExpectation();

    
    if(pIn->isMultiStageOptModelDDID() )
    {
        ROCPPOptModelDDID_Ptr pIn_DDU( static_pointer_cast<MultiStageOptModelDDID>(pIn));
        
        // ---------------- DECISION-DEPENDENT NON-ANTICIPATIVITY CONSTRAINTS -----------------------------------------------
        
        // |Y_{t,ij}| <= M x_{t-1,j} \forall i,j,t
        for (OneToExprVariableConverterIF::const_iterator tmldr_it=begin(); tmldr_it!=end(); tmldr_it++)
        {
            ROCPPVarIF_Ptr odv( pIn_DDU->getVar( tmldr_it->first ) );//Y_{t,i}
            
            for (MultiStageOptModelDDID::dduIterator ddu_it = pIn_DDU->dduBegin(); ddu_it != pIn_DDU->dduEnd(); ddu_it++)
            {
                ROCPPVarIF_Ptr mv( pIn_DDU->getMeasVar(ddu_it->first,odv->getTimeStage()-1) );//x_{t-1, j}
                
                ROCPPVarIF_Ptr ldrCoeff (getCoeffDV( odv->getName(),ddu_it->second->getName()) );//Y_{t, ij}
                
                // add non-anticipativity constraints
                ROCPPConstraintIF_Ptr pConstraint1( new IneqConstraint(false,true) );
                pConstraint1->add_lhs(1.,ldrCoeff);
                pConstraint1->add_lhs(-1.*m_bigM, mv);
                pConstraint1->set_rhs(make_pair(0.,true));
                pLDRModel->add_constraint(pConstraint1);
                
                ROCPPConstraintIF_Ptr pConstraint2( new IneqConstraint(false,true) );
                pConstraint2->add_lhs(-1.,ldrCoeff);
                pConstraint2->add_lhs(-1.*m_bigM, mv);
                pConstraint2->set_rhs(make_pair(0.,true));
                pLDRModel->add_constraint(pConstraint2);
            }
        }
    }
    auto stop = chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<chrono::seconds>(stop - start);
    
    cout << endl;
    cout << "Total time to approximate: " << duration.count() << " seconds" << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    
    return pLDRModel;
}

void LinearDecisionRule::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd)
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
                (*ldr) += ROCPPCstrTermIF_Ptr (new ProductTerm( 1., uit->second, dv ) );
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
    map< pair<string,string>, ROCPPVarIF_Ptr>::const_iterator it ( m_mapOrigDVUncPairToCoeffDV.find(make_pair(dvName,uncName)) );
    
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
    
    map<string, ROCPPVarIF_Ptr> expression;
    multimap<string, pair<string, ROCPPVarIF_Ptr> > copy = getLDRExpr();
    
    cout << name <<" = ";
    double value;
    multimap<string, pair<string, ROCPPVarIF_Ptr> >::iterator term = copy.find(name);
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

void LinearDecisionRule::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc)
{
    uint t;
    string name = unc->getName();
    
    if(!pIn->isMultiStageOptModelDDID()){
        cout << "This is not a decision dependent uncertain model." << endl;
        return;
    }
    
    if (!pIn->isDDU(name))
        throw MyException("Uncertain parameter "+ name + " does not have a time of revelation that is decision-dependent");
    
    ROCPPVarIF_Ptr meas;
    //bool value;
    double value;
    
    for(t = 1; t < pIn->getNumTimeStages(); t++)
    {
        meas = pIn->getMeasVar(name, t);
        value = resultIn.find(meas->getName())->second;
        bool observed;
        if (abs(value - 1.0) <= 0.01)
            observed = true;
        else if (abs(value - 0.0) <= 0.01)
            observed = false;
        else
            throw MyException("Wrong result for boolean variable.");
        
        if(observed){
            cout << "Uncertain parameter " << name << " is observed at stage " << t << endl;
            return;
        }
    }
    
    cout << "Uncertain parameter " << name << " is never observed" << endl;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% DISCRETE VARIABLE APPROXIMATOR INTERFACE %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool ConstantDecisionRule::isApplicable(ROCPPOptModelIF_Ptr pIn) const
{
    if (!pIn->isUncertainOptimizationModel())
    {
        cout << "Cannot apply LinearDecisionRule to deterministic model" << endl;
        return false;
    }
    
    if ((pIn->getObjType() == stochastic) && ( (!pIn->hasRectangularUncertaintySet()) || (pIn->hasDecisionDependentUncertaintySet()) ) )
    {
        cout << "LinearDecisionRule approximator is only applicable to stochastic problems with rectangular and decision-independent uncertainty set or to robust problems" << endl;
        return false;
    }
    
    if (pIn->hasRealVarsInUncertaintySet())
    {
        cout << "LinearDecisionRule approximator does not apply to problems with uncertainty set affected by real valued decisions" << endl;
        return false;
    }
    
    vector<ROCPPExpr_Ptr> objs(pIn->getObj()->getObj());
    
    vector<ROCPPExpr_Ptr>::const_iterator obj(objs.begin());
    
    for(; obj != objs.end(); obj++)
    {
        if((*obj)->hasProdsUncertainties())
        {
            cout << "Cannot deal with products of uncertainties at this time" << endl;
            return false;
        }
    }
    
    if ( (pIn->hasNonlinearities()) ) // here also will need to check if we have real valued decisions affecting the uncertainty set
    {
        cout << "Cannot handle nonlinear problem at present" << endl;
        return false;
    }
    
    return true;
}

void DiscreteVarsDRIF::findVarsToTranslate(vector<ROCPPConstraintIF_Ptr>::const_iterator first, vector<ROCPPConstraintIF_Ptr>::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
    for (ObjectiveFunctionIF::varsIterator vit = obj->varsBegin(); vit != obj->varsEnd(); vit++)
    {
        if ( (vit->second->isAdaptive()) && (! (vit->second->isRealVar()) ) )
            container += vit->second;
    }
    
    for (vector<ROCPPConstraintIF_Ptr>::const_iterator cit = first; cit != last; cit++)
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

void ConstantDecisionRule::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPVarIF_Ptr>  &translationMap, vector<ROCPPConstraintIF_Ptr> &toAdd)
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


ROCPPOptModelIF_Ptr ConstantDecisionRule::approximate(ROCPPOptModelIF_Ptr pIn)
{
    cout << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "======================== APPROXIMATING USING CDR ========================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    auto start = chrono::high_resolution_clock::now();
    
    ROCPPOptModelIF_Ptr pModel(pIn->Clone());
    
    if (pIn->getObjType() == robust)
        pModel->add_epigraph();
    
    // do linear decision rule
    ROCPPOptModelIF_Ptr pCDRModel(convertVar(pModel, true) );


    if ( (pCDRModel->getNumAdaptiveVars() - pCDRModel->getNumAdaptiveContVars()) != 0 )
        throw MyException("Adaptive integer and binary variables should have been eliminated by now");
    
    if (pIn->getObjType() == stochastic)
        pCDRModel->getExpectation();

    
    auto stop = chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<chrono::seconds>(stop - start);
    
    cout << endl;
    cout << "Total time to approximate: " << duration.count() << " seconds" << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    
    return pCDRModel;
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

void ConstantDecisionRule::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc)
{
    uint t;
    string name = unc->getName();
    
    if(!pIn->isMultiStageOptModelDDID()){
        cout << "This is not a decision dependent uncertain model." << endl;
        return;
    }
    
    if (!pIn->isDDU(name))
        throw MyException("Uncertain parameter "+ name + " does not have a time of revelation that is decision-dependent");
    
    ROCPPVarIF_Ptr meas;
    //bool value;
    double value;
    
    for(t = 1; t < pIn->getNumTimeStages(); t++)
    {
        meas = pIn->getMeasVar(name, t);
        value = resultIn.find(meas->getName())->second;
        bool observed;
        if (abs(value - 1.0) <= 0.01)
            observed = true;
        else if (abs(value - 0.0) <= 0.01)
            observed = false;
        else
            throw MyException("Wrong result for boolean variable.");
        
        if(observed){
            cout << "Uncertain parameter " << name << " is observed at stage " << t << endl;
            return;
        }
    }
    
    cout << "Uncertain parameter " << name << " is never observed" << endl;
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

