//
//  OptModelConverters.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "OptModelConverters.hpp"
#include "IncludeFiles.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "OptimizationModel.hpp"
#include "VariableConverter.hpp"
#include "OptModelConverters.hpp"
#include <time.h>
//#include "helpers.h"

template<class T1, class T2>
bool pairCompare(const pair<T1,T2> &x, const pair<T1,T2> &y) {
    return x.second < y.second;
}


bool pairCompare(const pair< pair<string,string> ,uint> &x, const pair< pair<string,string> ,uint> &y);

//template bool pairCompare(const uint & x, const uint & y);

template<class T>
typename T::iterator map_max_element(T &A)
{
    typedef typename T::value_type pair_type;
    typedef typename pair_type::first_type K;
    typedef typename pair_type::second_type V;
    return std::max_element(A.begin(), A.end(), pairCompare<K,V>);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%% BILINEAR TERM REFORMULATOR INTERFACE %%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPOptModelIF_Ptr BilinearTermReformulatorIF::linearize(ROCPPOptModelIF_Ptr pIn, const map<string,pair<double,double> >& variableBounds)
{
    ROCPPOptModelIF_Ptr pOut;
    
    if (pIn->isUncertainOptimizationModel())
        pOut = InstanciateModel(pIn->getType(), pIn->getNumTimeStages(), pIn->getObjType());
    else
        pOut = InstanciateModel(pIn->getType(), pIn->getNumTimeStages(),robust);
    
    map<string,pair<double,double> > variableBoundsCopy = variableBounds;
    vector<ROCPPConstraint_Ptr> cstrsToAdd;
    
    map<pair<string,string>, ROCPPVarIF_Ptr> allTerm;
    uint count = 0;
    
    for(OptimizationModelIF::constraintIterator cit = pIn->constraintBegin(); cit != pIn->constraintEnd(); cit++)
    {
        if(!(*cit)->hasNonlinearities())
            pOut->add_constraint((*cit));
        else{
            ROCPPConstraint_Ptr newCstr = (*cit)->replaceBilinearTerm(allTerm, count);
            pOut->add_constraint(newCstr);
        }
    }
    
    map<pair<string,string>, ROCPPVarIF_Ptr>::const_iterator term = allTerm.begin();
    for(;term != allTerm.end(); term++)
    {
        ROCPPVarIF_Ptr bindv = pIn->getVar(term->first.first);
        ROCPPVarIF_Ptr otherdv = pIn->getVar(term->first.second);
        ROCPPVarIF_Ptr newdv = term->second;
        getlinearCstr(bindv, otherdv, newdv, cstrsToAdd, variableBoundsCopy);
    }
    
    for (vector<ROCPPConstraint_Ptr>::const_iterator it = cstrsToAdd.begin(); it != cstrsToAdd.end(); it++)
        pOut->add_constraint(*it);
    
    pOut->set_objective(pIn->getObj());
    pOut->set_ddu(pIn);
    
    return pOut;
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%$$%%% BILINEAR TERM REFORMULATOR BIG_M %%%%%%%%%%%%$$%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void BTR_bigM::getlinearCstr(ROCPPVarIF_Ptr bindv, ROCPPVarIF_Ptr otherdv, ROCPPVarIF_Ptr  newdv, vector<ROCPPConstraint_Ptr>& cstrvec, map<string,pair<double,double> >& variableBounds)
{
    
    double M_ub(m_M);
    double M_lb(-1.*m_M);
    
    if (otherdv->isBooleanVar())
    {
        M_ub = (otherdv->getUB() < 1.)?(otherdv->getUB()):1.;
        M_lb = (otherdv->getLB() > 0.)?(otherdv->getLB()):0.;
    }
    else
    {
        if (otherdv->getUB() < INFINITY)
            M_ub = otherdv->getUB();
        else
        {
            // try to find the variable in the bounds map
            if (variableBounds.size()!=0)
            {
                map<string,pair<double,double> >::const_iterator bit ( variableBounds.find(otherdv->getName()) );
                if (bit != variableBounds.end())
                    M_ub = bit->second.second;
            }
            
        }
        
        // lower bound
        if (otherdv->getLB() > -INFINITY)
            M_lb = otherdv->getLB();
        else
        {
            // try to find the variable in the bounds map
            if (variableBounds.size()!=0)
            {
                map<string,pair<double,double> >::const_iterator bit ( variableBounds.find(otherdv->getName()) );
                if (bit != variableBounds.end())
                    M_lb = bit->second.first;
            }
            
        }
        
    }
    
    // w=xy (x is bin) <=> w <= max( M_ub, 0 ) x, w >= min( M_lb, 0 ) x, w <= y - (1-x) M_lb, w >= y - (1-x) M_ub
    {
        ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
        cstr->add_lhs(1.,newdv);
        cstr->add_lhs(-1.* max( M_ub, 0.), bindv);
        cstr->set_rhs( make_pair(0.,true) );
        cstrvec.push_back(cstr);
    }
    {
        ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
        cstr->add_lhs(-1.,newdv);
        cstr->add_lhs(min( M_lb, 0.), bindv);
        cstr->set_rhs( make_pair(0.,true) );
        cstrvec.push_back(cstr);
    }
    {
        ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
        cstr->add_lhs(1.,newdv);
        cstr->add_lhs(-1., otherdv);
        cstr->add_lhs(-M_lb, bindv);
        cstr->set_rhs( make_pair(-M_lb,false) );
        cstrvec.push_back(cstr);
    }
    {
        ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
        cstr->add_lhs(M_ub, bindv);
        cstr->add_lhs(1.,otherdv);
        cstr->add_lhs(-1.,newdv);
        cstr->set_rhs( make_pair(M_ub,false) );
        cstrvec.push_back(cstr);
    }
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% MODEL CONVERT FUNCTION %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPUncSSOptModel_Ptr convertToUSSOM(ROCPPOptModelIF_Ptr pIn)
{
    ROCPPUncSSOptModel_Ptr pModelOut(new UncertainSingleStageOptimizationModel(pIn->getObjType() ));
    
    for (OptimizationModelIF::constraintIterator c_it=pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
        pModelOut->add_constraint(*c_it);
    
    pModelOut->set_objective(pIn->getObj());
    
    return pModelOut;
}

ROCPPMISOCP_Ptr convertToMISOCP(ROCPPOptModelIF_Ptr pIn)
{
    ROCPPMISOCP_Ptr pModelOut(new MISOCP());
    
    for (OptimizationModelIF::constraintIterator c_it=pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
        pModelOut->add_constraint(*c_it);
    
    pModelOut->set_objective(pIn->getObj());
    
    return pModelOut;
}

