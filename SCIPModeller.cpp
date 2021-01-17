//
//  SCIPModeller.cpp
//  ROCPP
//
//  Created by 靳晴 on 1/16/21.
//

#include "SCIPModeller.hpp"
#include "Exceptions.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "ObjectiveFunction.hpp"
#include "OptimizationModel.hpp"
#include "OptModelConverters.hpp"
#include "GurobiModeller.hpp"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%% SCIP MODELLER %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SCIPModeller::SCIPModeller(const SolverParams &pSParams, bool useLazyNACs) : SolverModellerIF(pSParams) {}

SCIPModeller::SCIPModeller() : SolverModellerIF(SolverParams()) {}

void SCIPModeller::Reset()
{
    m_pSCIPVC.Reset();
    m_problemSolved = false;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% Creater Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool SCIPModeller::createSCIPmodel(boost::shared_ptr<CPLEXMISOCP> pModel, SCIP* scip)
{
    // ================================================================================
    // =========== CREATE VARIABLES ===================================================
    // ================================================================================
    
    addSCIPdecisionVars(pModel, scip);
    
    // ================================================================================
    // =========== CREATE CONSTRAINTS =================================================
    // ================================================================================
    
    
    uint cnt(0);
    for (OptimizationModelIF::constraintIterator c_it=pModel->constraintBegin(); c_it != pModel->constraintEnd(); c_it++)
    {
        cnt++;
        if ( cnt % 10000 == 0)
            cout << cnt << " constraints modelled from " << pModel->getNumConstraints() << endl;
        
        addConstraint( scip, *c_it, cnt);
        
    }
    return true;
}

bool SCIPModeller::addSCIPdecisionVars(boost::shared_ptr<CPLEXMISOCP> pModel, SCIP* scip)
{
    // SCIP can only solve MILP problem with linear objective function
    boost::shared_ptr<LHSExpression> obj(pModel->getObj()->getObj(1));
    
    if(!obj->isLinear())
        throw MyException("SCIP can only solve MILP problem with linear objective function");
    
    map<string, double> varCoeff;
    // iterate through the terms and find the coefficient of the decision variables
    LHSExpression::const_iterator t_it = obj->begin();
    for(; t_it != obj->end(); t_it++)
    {
        if((*t_it)->getNumVars() == 0)
            throw MyException("Term in the objective function must involve decision variable");
        
        string name((*t_it)->getDVContainer()->begin()->first);
        varCoeff.insert(make_pair(name, (*t_it)->getCoeff()));
    }
    
    double coeff;
    map<string, double>::const_iterator c_it;
    for (OptimizationModelIF::varsIterator v_it=pModel->varsBegin(); v_it != pModel->varsEnd(); v_it++)
    {
        // find the coefficient of the decision variables
        c_it = varCoeff.find(v_it->first);
        if(c_it != varCoeff.end())
            coeff = c_it->second;
        else
            coeff = 0.0;
        
        double lb( v_it->second->getLB());
        double ub( v_it->second->getUB());
        
        SCIP_VAR* var = nullptr;
        if (v_it->second->getType() == contDV)
        {
            SCIP_CALL(SCIPcreateVarBasic(scip, &var, v_it->second->getName().c_str(), lb, ub, coeff, SCIP_VARTYPE_CONTINUOUS));
            SCIP_CALL(SCIPaddVar(scip, var));
            
            m_pSCIPVC.m_contVarMap.insert( make_pair( (v_it->second->getName()), var ) );
            m_pSCIPVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), var ) );
        }
        else if (v_it->second->getType() == boolDV)
        {
            SCIP_CALL(SCIPcreateVarBasic(scip, & var, v_it->second->getName().c_str(), lb, ub, coeff, SCIP_VARTYPE_BINARY));
            SCIP_CALL(SCIPaddVar(scip, var));
            
            m_pSCIPVC.m_boolVarMap.insert( make_pair( (v_it->second->getName()), var ) );
            m_pSCIPVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), var ) );
        }
        else if (v_it->second->getType() == intDV)
        {
            SCIP_CALL(SCIPcreateVarBasic(scip, & var, v_it->second->getName().c_str(), lb, ub, coeff, SCIP_VARTYPE_INTEGER));
            SCIP_CALL(SCIPaddVar(scip, var));
            
            m_pSCIPVC.m_intVarMap.insert( make_pair( (v_it->second->getName()), var ) );
            m_pSCIPVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), var ) );
        }
        else throw MyException("unexpected type");
    }
    
    return true;
}

bool SCIPModeller::addConstraint(SCIP* scip, boost::shared_ptr<ConstraintIF> pCstrIn, uint num)
{
    if ( pCstrIn->isClassicConstraint() )
    {
        boost::shared_ptr<ClassicConstraintIF> pClassic ( boost::static_pointer_cast<ClassicConstraintIF> (pCstrIn) );
        
        if (pClassic->isLinear())
        {
            SCIP_CONS* cons = nullptr;
            
            // decide the lhs and the rhs
            double lhs, rhs((pClassic->get_rhs()).first);
            if (pClassic->isEqConstraint())
                lhs = rhs;
            else
                lhs = -SCIPinfinity(scip);
            
            // set name of the constraint base on the number
            string name("cstr" + to_string(num));
            SCIP_CALL( SCIPcreateConsBasicLinear(scip, &cons, name.c_str(),
                                                  0, nullptr, nullptr, lhs, rhs) );
            
            for (ClassicConstraintIF::const_iterator tit = pClassic->begin(); tit != pClassic->end(); tit++)
            {
                if (!(*tit)->isProductTerm() )
                    throw MyException("cplexmisocp cannot have non-prod terms");
                
                boost::shared_ptr<ProductTerm> pPT = boost::static_pointer_cast<ProductTerm>(*tit);
                if (pPT->getNumUncertainties() != 0)
                    throw MyException("cplexmisocp should not involve uncertainties");

                if (pPT->getNumVars()==0)
                    throw MyException("term in left hand-side expression must have decision variable");
                else
                {
                    map<string, SCIP_VAR*>::const_iterator vit (m_pSCIPVC.m_allVarsMap.find( pPT->varsBegin()->second->getName()));
                    if (vit == m_pSCIPVC.m_allVarsMap.end())
                        throw MyException("SCIPVar not found");
                    
                    SCIP_Var* var1 = vit->second;
                    
                    if ( pPT->isLinear() )
                        SCIP_CALL( SCIPaddCoefLinear(scip, cons, var1, pPT->getCoeff()) );
                    else throw ("unacceptable term type");
                }
            }
            num += 1;
        }
        else throw ("SCIP can only support linear constraint");
    }
    else if ( pCstrIn->isSOSConstraint() )
    {
        throw MyException("not supported in Gurobi modeller at this time");
    }
    else if ( pCstrIn->isIfThenConstraint() )
    {
        throw MyException("If then constraints not supported by Gurobi");
    }
    else throw MyException("unknown constraint type");
    
    return true;
}
