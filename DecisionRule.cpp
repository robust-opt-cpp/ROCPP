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
//%%%%%%%%%%%%%%%%%%%%%%% PARTITION CONVERTER %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string PartitionConverter::convertPartitionToString(uint partition) const
{
    string tmp2( to_string(partition) );
    string tmp1;
    for (uint i=1; i+tmp2.length() <= m_numEls; i++)
        tmp1 += "0";
    
    string out(tmp1);
    out += tmp2;
    return out;
}

string PartitionConverter::convertPartitionToString(const map<string,uint> &partitionIn, ROCPPOptModelIF_Ptr pModel) const
{
    if (!pModel->isUncertainOptimizationModel())
        throw MyException("only applicable to uncertain models");
    
    ROCPPconstUncOptModel_Ptr pInUnc = static_pointer_cast<const UncertainOptimizationModel>(pModel);
    
    
    string out;
    for (map<string,uint>::const_iterator mit = partitionIn.begin(); mit!=partitionIn.end(); mit++)
    {
        ROCPPUnc_Ptr unc( pInUnc->getUnc( mit->first ) );
        if (unc->isObservable())
            out += convertPartitionToString(mit->second);
    }
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// if it can be observe, then itself is the basic partition; if not then it must be euqal to the first segment so that the variable for all segment will be the same(equal to we do not have partition on that uncertainty in this time stage)

string PartitionConverter::getBasicPartition(const map<string,uint> &partitionIn, uint t, ROCPPOptModelIF_Ptr pModel, uint memory) const
{
    map<string,uint> partitionOut;
    
    if (!pModel->isUncertainOptimizationModel())
        throw MyException("only applicable to uncertain models");
    
    ROCPPconstUncOptModel_Ptr pInUnc = static_pointer_cast<const UncertainOptimizationModel>(pModel);
    
    map<string, pair<uint, uint> > timeStage = pInUnc->getdduStagesObs();
    
    for (map<string,uint>::const_iterator mit = partitionIn.begin(); mit!=partitionIn.end(); mit++)
    {
        ROCPPUnc_Ptr unc( pInUnc->getUnc( mit->first ) );

        //For ordinary uncertain model, the fisrt observable is the time stage of the uncertainty, the last one is the time stage of the model
        if (  (unc->isObservable() ) && (unc->getTimeStage() <= t) && (unc->getTimeStage()+memory > t) )
            partitionOut.insert( make_pair(mit->first, mit->second ) );
        else
            partitionOut.insert( make_pair(mit->first, 1 ) );
    }
    
    return convertPartitionToString(partitionOut,pModel);
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% PARTITION CONSTRUCTOR INTERFACE %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PartitionConstructorIF::PartitionConstructorIF(const map<string,uint> &numPartitionsMap) :
m_numPartitionsMap(numPartitionsMap),
m_bpdvs ( new dvContainer() )
{}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PartitionConstructorIF::usconstraints_iterator PartitionConstructorIF::USCbegin(string partition) const
{
    map< string, vector< ROCPPConstraint_Ptr > >::const_iterator mit( m_partitionUSconstraints.find(partition) );
    
    if (mit == m_partitionUSconstraints.end() )
        throw MyException("partition not found");
    
    return (mit->second.begin());
}

PartitionConstructorIF::usconstraints_iterator PartitionConstructorIF::USCend(string partition) const
{
    map< string, vector< ROCPPConstraint_Ptr > >::const_iterator mit( m_partitionUSconstraints.find(partition) );
    
    if (mit == m_partitionUSconstraints.end() )
        throw MyException("partition not found");
    
    return (mit->second.end());
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void PartitionConstructorIF::getReady(ROCPPOptModelIF_Ptr pIn, ROCPPParConverter_Ptr pPartConverter, ROCPPMItoMB_Ptr pMIMBConverter, map<string, pair<double,double> > &margSupp, const map<string,pair<double,double> >& OAmargSupp, string solver)
{
    
    constructPartitionsMap(pIn,pPartConverter);
    findMarginalSupportUncertaintySet(pIn,margSupp,pMIMBConverter,m_numPartitionsMap,solver,true,false,OAmargSupp);
    
    constructUncToBreakpointMap(margSupp);
    
    if (!pIn->isUncertainOptimizationModel())
        throw MyException("only applicable to uncertain problems");
    
    ROCPPconstUncOptModel_Ptr pInUnc = static_pointer_cast<const UncertainOptimizationModel>(pIn);
    
    
    for (map<string, map<string,uint> >::const_iterator pm_it=m_partitionsMap.begin(); pm_it!=m_partitionsMap.end(); pm_it++)
    {
        vector<ROCPPConstraint_Ptr > tmp;
        
        for (UncertainOptimizationModel::uncertaintiesIterator u_it=pInUnc->uncertaintiesBegin(); u_it!=pInUnc->uncertaintiesEnd();u_it++)
        {
            map<string,uint>::const_iterator loc1( (*pm_it).second.find( u_it->second->getName() ) );
            // break point number (p_i: goes from 1 to r_i)
            uint bpNum(loc1->second);
            // number of partitions in direction of \xi_i (r_i)
            
            map<string,uint>::const_iterator c_loc = m_numPartitionsMap.find( u_it->second->getName() );
            uint r(0);
            if (c_loc == m_numPartitionsMap.end())
                r=1;
            else
                r=(*c_loc).second;
    
            
            if (bpNum+1<=r)
            {
                map<pair<string,uint>, ROCPPExpr_Ptr >::const_iterator loc2( m_uncToBreakpointMap.find( make_pair(u_it->second->getName(), bpNum) ) );
                if (loc2==m_uncToBreakpointMap.end())
                    throw MyException("pair not found in m_uncToBreakpointMap");
                
                ROCPPClassicConstraint_Ptr pOutConstraint(new IneqConstraint(true));
                
                pOutConstraint->add_lhs( 1. , u_it->second );
                ROCPPExpr_Ptr expr( loc2->second->Clone() );
                (*expr) *= -1.;
                pOutConstraint->add_lhs( expr );
                pOutConstraint->set_rhs(make_pair(0.,true));
                
                
                tmp.push_back( pOutConstraint );
            }
            if (bpNum>1)
            {
                map<pair<string,uint>, ROCPPExpr_Ptr >::const_iterator loc2( m_uncToBreakpointMap.find( make_pair(u_it->second->getName(), bpNum-1) ) );
                if (loc2==m_uncToBreakpointMap.end())
                    throw MyException("pair not found in m_uncToBreakpointMap");
                
                ROCPPClassicConstraint_Ptr pOutConstraint(new IneqConstraint(true));
                
                pOutConstraint->add_lhs( -1. , u_it->second );
                ROCPPExpr_Ptr expr( loc2->second->Clone() );
                pOutConstraint->add_lhs( expr );
                pOutConstraint->set_rhs(make_pair(0.,true));
                
                
                tmp.push_back( pOutConstraint );
            }
        }
        m_partitionUSconstraints[pm_it->first] = tmp;
    }
}

uint PartitionConstructorIF::getNumSubsets(string uncNme) const
{
    map<string,uint>::const_iterator it ( m_numPartitionsMap.find(uncNme) );
    if (it==m_numPartitionsMap.end())
        throw MyException("uncertainty not found");
    return it->second;
}

uint PartitionConstructorIF::getPos(string partition, string uncNme) const
{
    map<string, map<string,uint> >::const_iterator pos( m_partitionsMap.find(partition));
    
    if (pos == m_partitionsMap.end())
        throw MyException("Partition not found");
    
    if (pos->second.find(uncNme) == pos->second.end())
        return 1;
    else
        return pos->second.find(uncNme)->second;
}

bool PartitionConstructorIF::hasPartition(string uncNme) const
{
    if (m_numPartitionsMap.find(uncNme) == m_numPartitionsMap.end())
        return false;
    
    else if (m_numPartitionsMap.find(uncNme)->second == 1)
        return false;
    
    return true;
}

ROCPPExpr_Ptr PartitionConstructorIF::getBp(pair<string, uint> uncOnPartition) const
{
    map< pair<string,uint>, ROCPPExpr_Ptr > ::const_iterator bp(m_uncToBreakpointMap.find(uncOnPartition));
    
    if (bp == m_uncToBreakpointMap.end())
        throw MyException("Partition and uncertainty pair not found");
        
    return bp->second;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% Protected Function %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void PartitionConstructorIF::constructPartitionsMap(ROCPPOptModelIF_Ptr pIn, ROCPPParConverter_Ptr pPartConverter)
{
    m_partitionsMap.clear();
    
    if (!pIn->isUncertainOptimizationModel())
        throw MyException("only applicable to uncertain problems");
    
    
    ROCPPconstUncOptModel_Ptr pInUnc = static_pointer_cast<const UncertainOptimizationModel>(pIn);
    
    
    size_t k( pInUnc->getNumUncertainties() );
    
    
    // calculate total number of partitions
    vector<uint> incr_vals(k,0);
    uint numParts(1);
    uint j(0);
    for (UncertainOptimizationModel::uncertaintiesIterator n_it=pInUnc->uncertaintiesBegin(); n_it!=pInUnc->uncertaintiesEnd();n_it++)
    {
        
        map<string,uint>::const_iterator tmp_it( m_numPartitionsMap.find( n_it->first ) );
        if (tmp_it==m_numPartitionsMap.end())
            throw MyException("uncertainty " + n_it->first + " not found in partitions map");
        
        if (j==0)
            incr_vals[j] = tmp_it->second;
        else
            incr_vals[j] = incr_vals[j-1]*tmp_it->second;
        if (!n_it->second->isObservable() && tmp_it->second >1)
            throw MyException("cannot partition along a direction that is not observable");
        
        numParts *= tmp_it->second;
        
        j++;
    }
    
    // populate matrix of partitions
    vector<vector<uint> > mat( vector<vector<uint> >(numParts, vector<uint>( k, 1) ) );
    
    
    
    // for each row of the matrix other than the first row
    for (uint i=1; i<mat.size(); i++)
    {
        uint j(0);
        bool increment_next(true);
        
        // for each column of the matrix
        for (UncertainOptimizationModel::uncertaintiesIterator n_it=pInUnc->uncertaintiesBegin(); n_it!=pInUnc->uncertaintiesEnd();n_it++)
        {
            map<string,uint>::const_iterator tmp_it( m_numPartitionsMap.find( n_it->first ) );
            
            
            // first, set the element equal to its value in the previous row
            uint tmp = increment_next ? 1:0;
            mat[i][j] = mat[i-1][j] + tmp;
            
            // check whether it must be set to 1 or kept the same
            if ( (i) % incr_vals[j] == 0 )
            {
                mat[i][j] = 1;
                increment_next = true;
            }
            else
                increment_next = false;
            
            
            j++;
        }
    }
    
    
    
    for (uint i=0; i<mat.size(); i++)
    {
        string partitionName("");
        uint j(0);
        map<string,uint> subMap;
        for (UncertainOptimizationModel::uncertaintiesIterator n_it=pInUnc->uncertaintiesBegin(); n_it!=pInUnc->uncertaintiesEnd();n_it++)
        {
            if (n_it->second->isObservable())
                partitionName += pPartConverter->convertPartitionToString(mat[i][j]);
            
            pair<map<string,uint>::iterator,bool> subret = subMap.insert(pair<string,uint>( n_it->second->getName(), mat[i][j] ) );
            if (subret.second==false)
                throw MyException("element already existed");
            
            j++;
        }
        
        pair<map<string,map<string,uint> >::iterator,bool> ret = m_partitionsMap.insert(pair<string,map<string,uint> >(partitionName,subMap));
        if (ret.second==false)
            throw MyException("partition already existed");
    }
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% STATIC PARTITION CONSTRUCTOR %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void StaticPartitionConstructor::constructUncToBreakpointMap(const map<string,pair<double,double> > &margSupp)
{
    m_uncToBreakpointMap.clear();
    
    // iterate through margSupp
    map<string,pair<double,double> >::const_iterator ms_it(margSupp.begin());
    for (; ms_it!=margSupp.end(); ms_it++)
    {
        map<string,uint>::const_iterator c_loc = m_numPartitionsMap.find( (*ms_it).first );
        uint cNumP(0);
        if (c_loc == m_numPartitionsMap.end())
            cNumP=1;
        else
            cNumP=(*c_loc).second;
        
        
        double step = ( (*ms_it).second.second ) - ( (*ms_it).second.first );
        step /= static_cast<double>(cNumP);
        
        for (uint i=1; i<cNumP; i++)
        {
            ROCPPExpr_Ptr expr ( new LHSExpression() );
            double tmp( ( (*ms_it).second.first ) + ( (static_cast<double>(i))*step) );
            (*expr) += ROCPPCstrTerm_Ptr( new ProductTerm( tmp ) ) ;
            m_uncToBreakpointMap[make_pair( (*ms_it).first, i)] = expr;
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% ADAPTIVE PARTITION CONSTRUCTOR %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void AdaptivePartitionConstructor::constructUncToBreakpointMap(const map<string,pair<double,double> > &margSupp)
{
    m_uncToBreakpointMap.clear();
    
    // iterate through margSupp
    map<string,pair<double,double> >::const_iterator ms_it(margSupp.begin());
    for (; ms_it!=margSupp.end(); ms_it++)
    {
        map<string,uint>::const_iterator c_loc = m_numPartitionsMap.find( (*ms_it).first );
        uint cNumP(0);
        if (c_loc == m_numPartitionsMap.end())
            cNumP=1;
        else
            cNumP=(*c_loc).second;
        
        ROCPPVarIF_Ptr prevbpdv;
        for (uint i=1; i<cNumP; i++)
        {
            ROCPPExpr_Ptr expr ( new LHSExpression() );
            ROCPPVarIF_Ptr bpdv( new VariableDouble( "bp_"+ms_it->first+"_"+to_string(i), ms_it->second.first , ms_it->second.second) );
            
            *m_bpdvs += bpdv;
            
            (*expr) += bpdv;
            m_uncToBreakpointMap[make_pair( (*ms_it).first, i)] = expr;
            
            if (i>1) // add constraint on breakpoint ordering
            {
                ROCPPClassicConstraint_Ptr cst( new IneqConstraint() );
                cst->add_lhs(1.,bpdv);
                cst->add_lhs(-1.,prevbpdv);
                cst->set_rhs(make_pair(0.,true));
                m_additionalConstraints.push_back(cst);
            }
            
            prevbpdv = bpdv;
        }
    }
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

