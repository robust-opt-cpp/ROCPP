//
//  VariableConverter.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include <stdio.h>

//#include "helpers.hpp"
#include "IncludeFiles.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "ObjectiveFunction.hpp"
#include "OptimizationModel.hpp"
#include "VariableConverter.hpp"
#include <fstream>
#include <math.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% VARIABLE CONVERTER INTERFACE %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPOptModelIF_Ptr VariableConverterIF::doMyThing(ROCPPOptModelIF_Ptr pIn, bool resetAndSave)
{
    ROCPPOptModelIF_Ptr pOut;
    if (pIn->isUncertainOptimizationModel())
        pOut = InstanciateModel(pIn->getType(),pIn->getNumTimeStages(),pIn->getObjType());
    else
        pOut = InstanciateModel(pIn->getType(),pIn->getNumTimeStages(),robust);
    
    vector<ROCPPConstraint_Ptr >::const_iterator first(pIn->constraintBegin());
    vector<ROCPPConstraint_Ptr >::const_iterator last(pIn->constraintEnd());
    ROCPPObjectiveIF_Ptr obj(pIn->getObj());
    
    vector<ROCPPConstraint_Ptr > toAdd;
    ROCPPObjectiveIF_Ptr toSet;
    ROCPPconstdvContainer_Ptr pOrigDVContainer(pIn->getDVContainer() );
    
    doMyThing(first,last,obj,*pOrigDVContainer,toAdd,toSet,resetAndSave);
    
    // out of the computed constraints, only add the ones that are useful!
    // a constraint is useless if it says "constant == constant"
    
    for (vector<ROCPPConstraint_Ptr >::const_iterator c_it = toAdd.begin(); c_it != toAdd.end(); c_it++ )
        if ( (*c_it)->isUseful() )
            pOut->add_constraint( *c_it );
    
    pOut->set_objective(toSet);
    pOut->set_ddu(pIn);
    
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% ONE TO ONE VARIABLE CONVERTER %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void OneToOneVariableConverterIF::doMyThing(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, const dvContainer& origDVContainer, vector<ROCPPConstraint_Ptr > &toAdd, ROCPPObjectiveIF_Ptr &toSet, bool resetAndSave)
{
    toAdd.clear();
    
    dvContainer tmpContainer;
    findVarsToTranslate(first,last,obj,tmpContainer);
    
    map<string,ROCPPVarIF_Ptr > translationMap;
    
    createTranslationMap(tmpContainer,translationMap,toAdd);
    
    if (resetAndSave)
    {
        m_translationMap=translationMap;
        createInverseMap(origDVContainer);
    }
    
    // now do actual conversion
    vector<ROCPPConstraint_Ptr >::const_iterator c_it(first);
    for (; c_it != last; c_it++)
        toAdd.push_back( (*c_it)->mapVars( translationMap ) );
    
    toSet = obj->mapObjVars(translationMap);
}

void OneToOneVariableConverterIF::createInverseMap(const dvContainer &origDVContainer)
{
    // iterate through the translation map
    for (const_iterator it = begin(); it != end(); it++)
    {
        // add them to the translation map - throw if they were already there
        pair< map<string, ROCPPVarIF_Ptr >::iterator, bool> ret;
        
        dvContainer::const_iterator ov ( origDVContainer.findthrow( it->first ) );
        
        ret = m_inverseMap.insert( make_pair( it->second->getName(), ov->second ) );
        
        if (ret.second==false)
            throw MyException("element already existed - that's unexpected");
        
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double OneToOneVariableConverterIF::evaluateVariableValue( string nme, const map<string,double>& valuesMap ) const
{
    const_iterator it ( m_translationMap.find( nme ) );
    
    if (it==m_translationMap.end())
        throw MyException("variable not found in translation map");
    
    map<string,double>::const_iterator valit ( valuesMap.find( it->second->getName() ) );
    
    if (valit==valuesMap.end())
        throw MyException("variable not found in valuesMap");
    
    return (valit->second );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%% ONE TO EXPRESSION VARIABLE CONVERTER %%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void OneToExprVariableConverterIF::doMyThing(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, const dvContainer& origDVContainer, vector<ROCPPConstraint_Ptr > &toAdd, ROCPPObjectiveIF_Ptr &toSet, bool resetAndSave)
{
    toAdd.clear();
    
    dvContainer tmpContainer;
    findVarsToTranslate(first,last,obj,tmpContainer);
    
    map<string,ROCPPExpr_Ptr > translationMap;
    
    createTranslationMap(tmpContainer,translationMap,toAdd);
    
    if (resetAndSave)
    {
        m_translationMap=translationMap;
        createInverseMap(origDVContainer);
    }
    
    // now do actual conversion
    vector<ROCPPConstraint_Ptr >::const_iterator c_it(first);
    for (; c_it != last; c_it++)
        toAdd.push_back( (*c_it)->mapVars( translationMap ) );
    
    toSet = obj->mapVars(translationMap);
}

void OneToExprVariableConverterIF::createInverseMap(const dvContainer &origDVContainer)
{
    
    // iterate through the translation map
    for (const_iterator it = begin(); it != end(); it++)
    {
        // iterate through the variables in the expression
        for (LHSExpression::dvIterator vit = it->second->varsBegin(); vit != it->second->varsEnd(); vit++)
        {
            // add them to the translation map - throw if they were already there
            pair< map<string, ROCPPVarIF_Ptr >::iterator, bool> ret;
            
            dvContainer::const_iterator ov ( origDVContainer.findthrow( it->first ) );
            
            ret = m_inverseMap.insert( make_pair( vit->second->getName(), ov->second ) );
            
            if (ret.second==false)
                throw MyException("element already existed - that's unexpected");
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double OneToExprVariableConverterIF::evaluateVariableValue( string nme, const map<string,double>& valuesMap ) const
{
    const_iterator it ( m_translationMap.find( nme ) );
    
    if (it==m_translationMap.end())
        throw MyException("variable not found in translation map");
    
    return (it->second->evaluate(valuesMap ) );
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% BILINEAR MI TO MB CONVERTER %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Bilinear_MItoMB_Converter::findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
    // identifies the integer, non-binary variables involved in bilinear terms in pIn
    dvContainer tmp;
    //obj->add_vars_involved_in_prod(tmp);
    
    vector<ROCPPConstraint_Ptr >::const_iterator c_it(first);
    // iterate through the constraints in pIN
    for (; c_it != last; c_it++)
    {
        (*c_it)->add_vars_involved_in_prod(tmp);
        
        // add to tmpContainer all the integer variables affecting the uncertainty set
        if ( (*c_it)->definesUncertaintySet() )
            (*c_it)->add_int_vars(container);
        
    }
    
    // add to tmpContainer all integer variables involved in products
    for (dvContainer::const_iterator it = tmp.begin(); it != tmp.end(); it++)
        if (it->second->isIntegerVar())
            container += it->second;
    
    if (container.getNumIntVars() != container.getNumVars() )
        throw MyException("should only have to convert integer variables");
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% UNARY CONVERTER %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void UnaryConverter::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd)
{
    translationMap.clear();
    
    // iterate through the decision variables in the tmpContainer
    for (dvContainer::const_iterator it = tmpContainer.begin(); it != tmpContainer.end(); it++)
    {
        if (!it->second->isIntegerVar())
            throw MyException("tmpMap should only contain integer vars");
        
        double lb(it->second->getLB());
        double ub(it->second->getUB());
        
        if ( !(lb > -INFINITY) || !(ub < INFINITY) )
            throw MyException("should have upper and lower bounds");
        
        if (lb > ub)
            throw MyException("we should have lb.second>ub.second");
        
        
        ROCPPClassicConstraint_Ptr pCstrl( new IneqConstraint() );
        
        ROCPPExpr_Ptr cexpr ( new LHSExpression() );
        (*cexpr) +=  lb ;
        
        for (int i=1; i<=(ub -lb); i++)
        {
            string nme;
            if (i<0)
                nme = it->second->getName() + "_m" + to_string(-1*i);
            else
                nme = it->second->getName() + "_" + to_string(i);
            
            ROCPPVarIF_Ptr dv;
            
            if ( it->second->isAdaptive() )
                dv = ROCPPVarIF_Ptr(new AdaptVarBool( nme, it->second->getTimeStage()) );
            else
                dv = ROCPPVarIF_Ptr( new VariableBool( nme ) );
            
            
            (*cexpr) += ROCPPCstrTerm_Ptr ( new ProductTerm(static_cast<double>(i), dv )  );
            pCstrl->add_lhs(1.,dv);
        }
        
        translationMap[it->second->getName()] = cexpr;
        pCstrl->set_rhs( make_pair(1.,false) );
        toAdd.push_back(pCstrl);
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% BINARY CONVERTER %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void BinaryConverter::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd)
{
    translationMap.clear();
    
    // iterate through the decision variables in the tmpContainer
    for (dvContainer::const_iterator it = tmpContainer.begin(); it != tmpContainer.end(); it++)
    {
        if (!it->second->isIntegerVar())
            throw MyException("tmpMap should only contain integer vars");
        
        double lb(it->second->getLB());
        double ub(it->second->getUB());
        
        if ( !(lb > -INFINITY) || !(ub < INFINITY) )
            throw MyException("should have upper and lower bounds");
        
        if (lb > ub)
            throw MyException("we should have lb.second>ub.second");
        
        
        ROCPPClassicConstraint_Ptr pCstrl( new IneqConstraint() );
        
        ROCPPExpr_Ptr cexpr ( new LHSExpression() );
        (*cexpr) +=  lb;
        
        double tmp(floor ( log( static_cast<double>(ub - lb) ) / log(2.) ) );
        
        for (int i=0; i<=static_cast<int>( tmp ); i++)
        {
            string nme;
            if (i<0)
                nme = it->second->getName() + "_m" + to_string(-1*i);
            else
                nme = it->second->getName() + "_" + to_string(i);
            
            ROCPPVarIF_Ptr dv;
            if ( it->second->isAdaptive() )
                dv = ROCPPVarIF_Ptr(new AdaptVarBool( nme, it->second->getTimeStage() ) );
            else
                dv = ROCPPVarIF_Ptr( new VariableBool( nme ) );
            
            double tmp2(pow(2.,static_cast<double>(i)));
            (*cexpr) += ROCPPCstrTerm_Ptr ( new ProductTerm( tmp2, dv )  );
            pCstrl->add_lhs( tmp2,dv);
            
        }
        
        translationMap[it->second->getName()] = cexpr;
        pCstrl->set_rhs( make_pair(static_cast<double>(ub - lb),false) );
        
        // check whether to add constraint? add if \lfloor log2( overline w - underline w ) \rfloor + 1 = 2^\theta for some \theta in \mathbb N
        double e1( tmp + 1. );
        double e2( log( static_cast<double>(ub - lb + 1))/log(2.) );
        if ( static_cast<int>(e1) != static_cast<int>(e2) )
            toAdd.push_back(pCstrl);
    }
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% REAL VARIABLE BILINEAR POS REFORMULATOR %%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void RealVarBilinearPosReformulator::findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
    // identifies the real variables involved in bilinear terms that do not have a lower bound or that have a lower bound that is less than zero
    vector<ROCPPConstraint_Ptr >::const_iterator c_it(first);
    
    dvContainer tmp;

    // iterate through the constraints in pIN
    for (; c_it != last; c_it++)
    {
        // add to tmpContainer all integer variables involved in products
        (*c_it)->add_vars_involved_in_prod(tmp);
    }
    
    for (dvContainer::const_iterator it = tmp.begin(); it != tmp.end(); it++)
        if (it->second->isRealVar())
        {
            //???: Am I right? find the variable which can be negative
            double lb = it->second->getLB();
            if((!(lb > -INFINITY)) || lb <= 0)
                container += it->second;
        }
    
    if (container.getNumContVars() != container.getNumVars() )
        throw MyException("should only have to convert real valued variables");
}

void RealVarBilinearPosReformulator::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd)
{
    translationMap.clear();
    
    // iterate through the decision variables in the tmpContainer
    for (dvContainer::const_iterator it = tmpContainer.begin(); it != tmpContainer.end(); it++)
    {
        if (!it->second->isRealVar())
            throw MyException("tmpMap should only contain real vars");
        
        
        double lb(it->second->getLB());
        double ub(it->second->getUB());
        
        ROCPPExpr_Ptr cexpr ( new LHSExpression() );
        
        ROCPPVarIF_Ptr dvp;
        ROCPPVarIF_Ptr dvn;
        
        if (it->second->isAdaptive())
        {
            // positive part
            dvp = ROCPPVarIF_Ptr ( new AdaptVarDouble( ( it->second->getName() + "_pos" ), it->second->getTimeStage(), 0.) );

            //negative part
            dvn = ROCPPVarIF_Ptr ( new AdaptVarDouble( ( it->second->getName() + "_neg" ), it->second->getTimeStage(), 0.) );
        }
        else
        {
            // positive part
            dvp = ROCPPVarIF_Ptr ( new VariableDouble( ( it->second->getName() + "_pos" ), 0.) );
            //negative part
            dvn = ROCPPVarIF_Ptr ( new VariableDouble( ( it->second->getName() + "_neg" ), 0.) );
        }
        
        (*cexpr) += dvp;
        ROCPPCstrTerm_Ptr tmp( new ProductTerm(-1., dvn) );
        (*cexpr) += tmp;
        
        translationMap[it->second->getName()] = cexpr;
        
        // add the variables to posPartMap and negPartMap
        m_posPartMap.insert(make_pair( it->second->getName(), dvp ) );
        m_negPartMap.insert( make_pair(it->second->getName(), dvn ) );
        
        
        // must add upper and lower bound constraints if any
        
        if ( (lb > -INFINITY) && (lb) )
        {
            ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
            ROCPPExpr_Ptr cexprn ( cexpr->Clone() );
            (*cexprn) *= -1.;
            cstr->add_lhs( cexprn );

            double lbn(lb);
            lbn *= -1.;
            bool isZero = true;
            if ((lb<-10E-4) || (lb>10E-4) )
                isZero = false;
            cstr->set_rhs( make_pair(lbn, isZero) );
            toAdd.push_back(cstr);
        }
        if ( (ub < INFINITY) )
        {
            ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
            cstr->add_lhs( cexpr->Clone() );
            bool isZero = true;
            if ((ub<-10E-4) || (ub>10E-4) )
                isZero = false;
            cstr->set_rhs( make_pair(ub, isZero) );

            toAdd.push_back( cstr);
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPVarIF_Ptr RealVarBilinearPosReformulator::getPosPart(string origVarNme) const
{
    map<string, ROCPPVarIF_Ptr >::const_iterator it (m_posPartMap.find(origVarNme) );
    if (it==m_posPartMap.end())
        throw MyException("variable not found");
    
    return it->second;
}

ROCPPVarIF_Ptr RealVarBilinearPosReformulator::getNegPart(string origVarNme) const
{
    map<string, ROCPPVarIF_Ptr >::const_iterator it (m_negPartMap.find(origVarNme) );
    if (it==m_negPartMap.end())
        throw MyException("variable not found");
    
    return it->second;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% REAL VARIABLE DISCRETIZER %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void RealVarDiscretizer::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd)
{
    
    translationMap.clear();
    
    // iterate through the decision variables in the tmpContainer
    for (dvContainer::const_iterator it = tmpContainer.begin(); it != tmpContainer.end(); it++)
    {
        if (!it->second->isRealVar())
            throw MyException("tmpMap should only contain real vars");

        
        double lb(it->second->getLB());
        double ub(it->second->getUB());
        
        if ( !(lb > -INFINITY) || !(ub < INFINITY) )
            throw MyException(it->second->getName() + " should have upper and lower bounds");
        
        if (lb > ub)
            throw MyException(it->second->getName() + " we should have lb>ub");
        
      
        ROCPPExpr_Ptr cexpr ( new LHSExpression() );
        (*cexpr) +=  lb ;
        
        double coeff( ub - lb );
        
        for (uint i=0; (i+1)<=m_numBitsPerVariable; i++)
        {
            string nme;
            nme = it->second->getName() + "_" + to_string(i);
            
            ROCPPVarIF_Ptr dv;
            
            if (!it->second->isAdaptive())
            {
                dv = createVariable(nme, boolDV, false);
            }
            else
                dv = ROCPPVarIF_Ptr( new AdaptVarBool(nme,it->second->getTimeStage()) );
            
            (*cexpr) += ROCPPCstrTerm_Ptr ( new ProductTerm( coeff * pow( 2., -1.*static_cast<double>(i)) , dv )  );
            
        }
        translationMap[it->second->getName()] = cexpr;
        
        
        // add the bounds (if any) as constraints
        
        if (ub < INFINITY)
        {
            ROCPPClassicConstraint_Ptr cstr ( new IneqConstraint() );
            cstr->add_lhs(cexpr->Clone());
            
            bool isZero = (ub >= -10e-4)&&(ub <= 10e-4);
            cstr->set_rhs( make_pair(ub, isZero) );
            
            toAdd.push_back(cstr);
        }
        if (lb > -INFINITY)
        {
            ROCPPClassicConstraint_Ptr cstr ( new IneqConstraint() );
            ROCPPExpr_Ptr expr( cexpr->Clone() );
            (*expr) *= -1.;
            cstr->add_lhs(expr);
            
            bool isZero = (lb >= -10e-4)&&(lb <= 10e-4);
            cstr->set_rhs( make_pair(-1.*lb, isZero) );
            
            toAdd.push_back(cstr);
        }
    }
}

void RealVarDiscretizer::findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
    vector<ROCPPConstraint_Ptr >::const_iterator c_it(first);
    
    dvContainer dvs;
    
    // iterate through the constraints in pIN
    for (; c_it != last; c_it++)
    {
        (*c_it)->add_vars_involved_in_prod(dvs);
    }
    
    for (dvContainer::const_iterator vit = dvs.begin(); vit != dvs.end(); vit++)
    {
        if ( vit->second->isRealVar() )
            container += vit->second;
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%% UNCERTAINTY SET REAL VARIABLE APPROXIAMATOR %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void UncertaintySetRealVarApproximator::findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
    vector<ROCPPConstraint_Ptr >::const_iterator c_it(first);
    // iterate through the constraints in pIN
    for (; c_it != last; c_it++)
    {
        if ( (*c_it)->definesUncertaintySet() )
        {
            for (ConstraintIF::varsIterator vit = (*c_it)->varsBegin(); vit != (*c_it)->varsEnd(); vit++)
            {
                if (vit->second->isRealVar())
                {
                    container += vit->second;
                    
                }
            }
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% PREDEF O2O VARIABLE CONVERTER %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPConstraint_Ptr PredefO2OVariableConverter::doMyThing(ROCPPConstraint_Ptr pCstr) const
{
    return pCstr->mapVars( m_translationMap );
}

void PredefO2OVariableConverter::findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
}

void PredefO2OVariableConverter::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPVarIF_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd)
{
    translationMap = m_translationMap;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% PREDEF O2E VARIABLE CONVERTER %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPConstraint_Ptr PredefO2EVariableConverter::doMyThing(ROCPPConstraint_Ptr pCstr) const
{
    return pCstr->mapVars( m_translationMap );
}

void PredefO2EVariableConverter::createTranslationMap(const dvContainer &tmpContainer, map<string,ROCPPExpr_Ptr >  &translationMap, vector<ROCPPConstraint_Ptr > &toAdd)
{
    translationMap = m_translationMap;
}

void PredefO2EVariableConverter::findVarsToTranslate(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last, ROCPPObjectiveIF_Ptr obj, dvContainer &container)
{
}
