//
//  RobustifyEngine.cpp
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
#include "DecisionRule.hpp"
#include "RobustifyEngine.hpp"

boost::shared_ptr<Bilinear_MISOCP> RobustifyEngine::doMyThing(boost::shared_ptr<UncertainSingleStageOptimizationModel> pIn, bool feasible)
{
    boost::shared_ptr<Bilinear_MISOCP> pOut( new Bilinear_MISOCP() );
    calculateUncertaintySetMatrices(pIn);
    
    // iterate through constraints of pModelIn
    OptimizationModelIF::constraintIterator c_it(pIn->constraintBegin() );
    uint ccnt(1);
    size_t numCstr(pIn->getNumProblemConstraints());
    
    uint ratio((uint)numCstr/10);
    
    ratio = ratio>1 ? ratio : 1;
    
    for (; c_it!=pIn->constraintEnd(); c_it++)
    {
        if((*c_it)->definesUncertaintySet())
            continue;
        
        if (ccnt % ratio == 0)
            cout << ccnt << " of " << pIn->getNumProblemConstraints() << " constraints robustified" << endl;
        
        robustifyConstraint(*c_it,pIn,pOut,feasible);
        ccnt++;
    }
    
    if (pIn->getObj()->isDeterministic())
        pOut->set_objective(pIn->getObj() );
    
    return pOut;
}

void RobustifyEngine::calculateUncertaintySetMatrices(boost::shared_ptr<UncertainSingleStageOptimizationModel> const pIn)
{
    
    size_t l (pIn->getNumUncertaintySetConstraints() );
    vector<vector<vector< pair<bool, boost::shared_ptr<LHSExpression> > > > > EMvec;
    EMvec.resize(l);
    vector<vector< boost::shared_ptr<LHSExpression> > > EVvec;
    EVvec.resize(l);
    
    uint cl(0);
    for (UncertainOptimizationModel::uncertaintySetIterator cit = pIn->uncertaintySetBegin(); cit != pIn->uncertaintySetEnd(); cit++)
    {
        if ( !(*cit)->isClassicConstraint() )
            throw MyException("uncertainty set should only contain classic constraints");
        
        if ( !(*cit)->isWellDefined() )
            throw MyException("badly defined constraint: cannot robustify");
        
        if ( (*cit)->definesUncertaintySet() )
        {
            boost::shared_ptr<ClassicConstraintIF> pClassic ( boost::static_pointer_cast<ClassicConstraintIF>(*cit) );
            
            boost::shared_ptr<LHSExpression> lin_part ( pClassic->getLinearPart() );
            boost::shared_ptr<NormTerm> norm_term;
            
            if ( pClassic->hasNormTerm() )
                norm_term = ( pClassic->getNormTerm() );
            
            // first dimension
            uint m(1);
            
            if ( pClassic->hasNormTerm() )
                m += norm_term->getNumTerms();
            
            // second dimension
            size_t k( pIn->getNumUncertainties() );
            
            EMvec[cl].resize(m);
            EVvec[cl].resize(m);
            
            for (uint i=0; i<m; i++)
                EMvec[cl][i].resize(k);
            
            // use lin_part to build the vector container (p_s(x))
            for (UncertainOptimizationModel::uncertaintiesIterator uit = pIn->uncertaintiesBegin(); uit != pIn->uncertaintiesEnd(); uit++)
            {
                uint ck(pIn->getAlphabeticalLocation(uit->second));
                
                // - c(x)^T terms
                {
                    pair<bool,boost::shared_ptr<LHSExpression> > tmp ( lin_part->factorOut(uit->second) );
                    (*tmp.second) *= -1.;
                    EMvec[cl][m-1][ck] = tmp;
                }
                // A(x) terms
                if ( pClassic->hasNormTerm() )
                {
                    uint cm(0);
                    for (NormTerm::const_iterator ntit = norm_term->begin(); ntit != norm_term->end(); ntit++, cm++)
                    {
                        pair<bool,boost::shared_ptr<LHSExpression> > tmp ( (*ntit)->factorOut(uit->second) );
                        EMvec[cl][cm][ck] = tmp;
                    }
                }
            }
            // - d(x) + h terms
            {
                boost::shared_ptr<LHSExpression> det_lin_part ( lin_part->getDeterministicLinearPart() );
                (*det_lin_part) *= -1.;
                EVvec[cl][m-1] = det_lin_part;
            }
            // b(x) terms
            if ( pClassic->hasNormTerm() )
            {
                uint cm(0);
                for (NormTerm::const_iterator ntit = norm_term->begin(); ntit != norm_term->end(); ntit++, cm++)
                {
                    boost::shared_ptr<LHSExpression> tmp ( (*ntit)->getDeterministicLinearPart() );
                    EVvec[cl][cm] = tmp;
                }
            }
            
            m_isEqCstr.insert( make_pair( make_pair( 1, cl ), pClassic->isEqConstraint() ) );
            
            cl++;
        }
    }
    
    m_EMvec.insert(make_pair(1,EMvec));
    m_EVvec.insert(make_pair(1,EVvec));
    
    m_uncertaintySetMatricesCalculated=true;
}


void RobustifyEngine::createDualVars(boost::shared_ptr<Bilinear_MISOCP> pOut, boost::shared_ptr<ConstraintIF> pCstr, vector<vector<boost::shared_ptr<DecisionVariableIF> > >& dualVars, bool feasible)
{
    dualVars.clear();
    
    if (pCstr->isDeterministic()&&feasible)
        throw MyException("constraint provided is deterministic");
    
    if (!m_uncertaintySetMatricesCalculated)
        throw MyException("you must first calculate the uncertainty set matrices");
    
    if (m_EMvec.size() != m_EVvec.size())
        throw MyException("sizes error in robustify engine");
    
    // get the time-stage associated with robustification of the constraint that we want to robustify
    
    uint t(1);
    
    // find this time stage in m_EMvec and m_EVvec
    
    map<uint, vector<vector<vector< pair<bool, boost::shared_ptr<LHSExpression> > > > > >::const_iterator em_it( m_EMvec.find(t) );
    map<uint, vector<vector< boost::shared_ptr<LHSExpression> > > >::const_iterator ev_it( m_EVvec.find(t) );
    
    if (em_it == m_EMvec.end())
        throw MyException("robustify stage not found in EMvec");
    
    if (ev_it == m_EVvec.end())
        throw MyException("robustify stage not found in EVvec");
    
    if (em_it->second.size()!=ev_it->second.size())
        throw MyException("wrong size for EMvec of EVvec");
    
    size_t l( em_it->second.size() );
    
    dualVars.resize(l);
    
    for ( uint cl = 0; cl<l; cl++)
    {
        if (em_it->second[cl].size() != ev_it->second[cl].size())
            throw MyException("sizes error in robustify engine");
        
        bool isEq( m_isEqCstr[ make_pair(t,cl) ] );
        
        dualVars[cl].resize(em_it->second[cl].size());
        
        boost::shared_ptr<DecisionVariableIF> conehead;
        vector<boost::shared_ptr<DecisionVariableIF> > nonheaddvs;
        for (uint cm = 0; cm<em_it->second[cl].size(); cm++) // iterate, in each constraint, through the first dimension of the constraint matrix
        {
            boost::shared_ptr<DecisionVariableIF> dv;
            
            string nme(m_dualNme+"_"+ boost::lexical_cast<string>(++m_dualVarsCounter));
            if (m_dualNme_suff!="")
                nme += "_"+m_dualNme_suff;
            
            
            if (cm+1==em_it->second[cl].size())
            {
                
                if ( isEq )
                {
                    
                    if ( em_it->second[cl].size() != 1 )
                        throw MyException("something wrong: equality constraint should have dimension 1");
                    
                    dv = boost::shared_ptr<DecisionVariableIF>(  new VariableDouble(nme) );
                }
                else{
                    dv = boost::shared_ptr<DecisionVariableIF>(  new VariableDouble(nme, 0.) );
                }
                
                conehead = dv;
            }
            else
            {
                dv = boost::shared_ptr<DecisionVariableIF>(  new VariableDouble(nme ) );
                nonheaddvs.push_back(dv);
            }
            
            dualVars[cl][cm] = dv;
        }
        if (!isEq)
            pOut->add_soc_constraint(conehead,nonheaddvs);
    }
}

void RobustifyEngine::robustifyConstraint(boost::shared_ptr<ConstraintIF> pConstraint, boost::shared_ptr<UncertainSingleStageOptimizationModel> const pIn, boost::shared_ptr<Bilinear_MISOCP> pOut, bool feasible)
{
    if (pConstraint->definesUncertaintySet())
        return;
    
    if (pConstraint->isDeterministic() && feasible)
    {
        pOut->add_constraint(pConstraint);
        return;
    }
    
    if (!pConstraint->isClassicConstraint() )
        throw MyException("cannot robustify non classic constraint");
    
    boost::shared_ptr<ClassicConstraintIF> pClassic ( boost::static_pointer_cast<ClassicConstraintIF>(pConstraint) );
    
    
    if (pClassic->hasNormTerm())
        throw MyException("cannot robustify constraint involving norm term");
    
    
    if (pClassic->isEqConstraint())
        throw MyException("cannot robustify uncertain equality constraint");
    
    
    vector<vector<boost::shared_ptr<DecisionVariableIF> > > dualVars;
    createDualVars(pOut,pClassic,dualVars,feasible);
    
    // get the time-stage associated with robustification of the constraint that we want to robustify
    
    uint t(1);
    
    // find this time stage in m_EMvec and m_EVvec
    
    map<uint, vector<vector<vector< pair<bool, boost::shared_ptr<LHSExpression> > > > > >::const_iterator em_it( m_EMvec.find(t) );
    map<uint, vector<vector< boost::shared_ptr<LHSExpression> > > >::const_iterator ev_it( m_EVvec.find(t) );
    
    if (em_it == m_EMvec.end())
        throw MyException("robustify stage not found in EMvec");
    
    if (ev_it == m_EVvec.end())
        throw MyException("robustify stage not found in EVvec");
    
    if (em_it->second.size()!=ev_it->second.size())
        throw MyException("wrong size for EMvec of EVvec");
    
    // first constraint
    {
        boost::shared_ptr<ClassicConstraintIF> cstr( new IneqConstraint() );
        cstr->add_lhs( pClassic->getLinearPart()->getDeterministicLinearPart()  );
        
        for (uint cl=0; cl<em_it->second.size(); cl++)  // iterate through the constraints defining the uncertainty set
        {
            for (uint cm=0; cm<ev_it->second[cl].size(); cm++) // iterate, in each constraint, through the first dimension of the constraint matrix
            {
                boost::shared_ptr<LHSExpression> tmp( ev_it->second[cl][cm]->Clone() );
                (*tmp) *= dualVars[cl][cm];
                cstr->add_lhs( tmp  );
            }
        }
        cstr->set_rhs( make_pair(0.,true) );
        pOut->add_constraint(cstr);
    }
    // second constraint (set of constraints)
    {
        // first build c vec
        vector<pair<bool,boost::shared_ptr<LHSExpression> > > cvec;
        {
            boost::shared_ptr<LHSExpression> lin_part ( pClassic->getLinearPart() );
            
            for (UncertainOptimizationModel::uncertaintiesIterator uit = pIn->uncertaintiesBegin(); uit != pIn->uncertaintiesEnd(); uit++)
            {
                pair<bool,boost::shared_ptr<LHSExpression> > tmp ( lin_part->factorOut(uit->second) );
                cvec.push_back(tmp);
            }
        }
        // now the constraints
        for (uint ck = 0; ck < pIn->getNumUncertainties(); ck++) // uncertainty count
        {
            boost::shared_ptr<ClassicConstraintIF> cstr( new EqConstraint() );
            
            bool found(false);
            
            if (cvec[ck].first)
            {
                cstr->add_lhs( cvec[ck].second );
                found = true;
            }
            for (uint cl=0; cl<ev_it->second.size(); cl++) // uncertainty set constraint count
            {
                for (uint cm=0; cm<ev_it->second[cl].size(); cm++) // row of uncertainty set constraint matrix count
                {
                    pair<bool,boost::shared_ptr<LHSExpression> > tmptmp( em_it->second[cl][cm][ck] );
                    pair<bool,boost::shared_ptr<LHSExpression> > tmp( make_pair(tmptmp.first, tmptmp.second->Clone() ));
                    if (tmp.first)
                    {
                        (*tmp.second) *= dualVars[cl][cm];
                        cstr->add_lhs( tmp.second  );
                        found = true;
                    }
                }
            }
            cstr->set_rhs( make_pair(0.,true) );
            if (found)
                pOut->add_constraint(cstr);
        }
    }
    
}
