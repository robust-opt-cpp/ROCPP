//
//  ObjectiveFunction.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "IncludeFiles.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "ObjectiveFunction.hpp"
#include <fstream>
#include <math.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% OBJECTIVE FUNCTION INTERFACE %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ObjectiveFunctionIF::checkCompatibility(ROCPPExpr_Ptr pExpression) const
{
    if (pExpression->hasNonlinearities())
        throw MyException("Cannot add nonlinear term to objective");
    if (pExpression->hasNormTerm() )
        throw MyException("Canot add norm term to objective");
    if (pExpression->hasProdsUncertainties())
        throw MyException("Can not add product of uncertainty into objective");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% SIMPLE OBJECTIVE FUNCTION %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SimpleObjective::SimpleObjective() : ObjectiveFunctionIF(simpleObj), m_pObjFun(new LHSExpression())
{
}

SimpleObjective::SimpleObjective(ROCPPExpr_Ptr objFun) : ObjectiveFunctionIF(simpleObj)
{
    checkCompatibility(objFun);
    m_pObjFun = objFun;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void SimpleObjective::add_to_obj(ROCPPVarIF_Ptr pVar, double cost)
{
    m_pObjFun->add(cost, pVar);
}

void SimpleObjective::add_vars_involved_in_prod(dvContainer &dvs) const
{
    vector<ROCPPExpr_Ptr > objvec(getObj());
    for (obj_fun_iterator it = objvec.begin(); it!= objvec.end(); it++)
        (*it)->add_vars_involved_in_prod(dvs);
}

void SimpleObjective::add_int_vars(dvContainer &dvs) const
{
    m_pObjFun->getDVContainer()->add_int_vars(dvs);
}

ROCPPObjectiveIF_Ptr SimpleObjective::replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr > &term, ROCPPVarIF_Ptr var) const
{
    ROCPPObjectiveIF_Ptr newObj( new SimpleObjective(m_pObjFun->replaceTermWithVar(term, var)));
    
    return newObj;
}

ROCPPObjectiveIF_Ptr SimpleObjective::mapObjVars(const map<string,ROCPPVarIF_Ptr > &mapFromOldToNewVars) const
{
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective( m_pObjFun->mapExprVars(mapFromOldToNewVars)));
    return newObj;
}

ROCPPObjectiveIF_Ptr SimpleObjective::mapObjUnc(const map<string,ROCPPUnc_Ptr > &mapFromOldToNewUnc) const
{
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective( m_pObjFun->mapExprUnc(mapFromOldToNewUnc)));
    return newObj;
}

ROCPPObjectiveIF_Ptr SimpleObjective::mapVars(const map<string, ROCPPExpr_Ptr > &mapFromVarToExpression) const
{
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective( m_pObjFun->mapVars(mapFromVarToExpression)));
    return newObj;
}

ROCPPObjectiveIF_Ptr SimpleObjective::mapUncs(const map<string, ROCPPExpr_Ptr > &mapFromUncToExpression) const
{
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective( m_pObjFun->mapUncs(mapFromUncToExpression)));
    return newObj;
}

void SimpleObjective::convertToEpigraph(ROCPPVarIF_Ptr epigraphVar, vector<ROCPPConstraint_Ptr > &epigraphConstraints) const
{
    epigraphConstraints.clear();
    
    ROCPPConstraint_Ptr epicst(new IneqConstraint());
    epicst->add_lhs(m_pObjFun);
    epicst->add_lhs(-1., epigraphVar);
    
    epigraphConstraints.push_back(epicst);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

uint SimpleObjective::getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr > &term) const
{
    uint out(m_pObjFun->getNumTimesTermAppears(term));
    
    return out;
}

vector<ROCPPExpr_Ptr > SimpleObjective::getObj() const
{
    vector<ROCPPExpr_Ptr > con;
    con.push_back(m_pObjFun);
    
    return con;
}

ROCPPExpr_Ptr SimpleObjective::getObj(uint i) const
{
    if(i != 1)
        throw MyException("The number of objective function must be 1(0)");
    
    return m_pObjFun;
}


bool SimpleObjective::hasNonlinearities() const
{
    return m_pObjFun->hasNonlinearities();
}

bool SimpleObjective::hasProdsUncertainties() const
{
    return m_pObjFun->hasProdsUncertainties();
}

bool SimpleObjective::hasProdsContVars() const
{
    return m_pObjFun->hasProdsContVars();
}

bool SimpleObjective::isDeterministic() const
{
    return m_pObjFun->isDeterministic();
}

uint SimpleObjective::getNumAdaptiveVars() const
{
    return m_pObjFun->getNumAdaptiveVars();
}

uint SimpleObjective::getTimeStage() const
{
    return m_pObjFun->getTimeStage();
}

bool SimpleObjective::varIsInvolved(string varName, uint i) const
{
    if(i != 1)
        throw MyException("Simple objective only has one objective function");
    
    return( ( m_pObjFun->getDVContainer()->find(varName) != m_pObjFun->getDVContainer()->end() ) );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPObjectiveIF_Ptr SimpleObjective::Clone() const
{
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective(getObj(1)->Clone()));
    
    return newObj;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void SimpleObjective::WriteToStream(ofstream &ofs)
{
    m_pObjFun->WriteToStream(ofs);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% MAX OBJECTIVE FUNCTION %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

MaxObjective::MaxObjective() : ObjectiveFunctionIF(maxObj), m_dvContainer(new dvContainer()), m_uncContainer(new uncContainer())
{
    for(obj_iterator obj = m_pObjFuns.begin(); obj != m_pObjFuns.end(); obj++){
        *m_dvContainer += *(*obj)->getDVContainer();
        *m_uncContainer += *(*obj)->getUncContainer();
    }
}

MaxObjective::MaxObjective(vector<ROCPPExpr_Ptr > objFuns) : ObjectiveFunctionIF(maxObj), m_dvContainer(new dvContainer()), m_uncContainer(new uncContainer())
{
    for (obj_iterator obj = objFuns.begin(); obj != objFuns.end(); obj++){
        
        checkCompatibility(*obj);
        m_pObjFuns.push_back(*obj);
    }
    
    for(obj_iterator obj = m_pObjFuns.begin(); obj != m_pObjFuns.end(); obj++){
        *m_dvContainer += *(*obj)->getDVContainer();
        *m_uncContainer += *(*obj)->getUncContainer();
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ObjectiveFunctionIF::varsIterator MaxObjective::varsBegin() const
{
    return m_dvContainer->begin();
}
ObjectiveFunctionIF::varsIterator MaxObjective::varsEnd() const
{
    return m_dvContainer->end();
}

ObjectiveFunctionIF::uncsIterator MaxObjective::uncBegin() const
{
    return m_uncContainer->begin();
}

ObjectiveFunctionIF::uncsIterator MaxObjective::uncEnd() const
{
    return m_uncContainer->end();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void MaxObjective::add_to_obj(ROCPPVarIF_Ptr pVar, double cost)
{
    for(obj_iterator obj = objectiveBegin(); obj!= objectiveEnd(); obj++){
        (*obj)->add(cost, pVar);
    }
}

void MaxObjective::add_vars_involved_in_prod(dvContainer &dvs) const
{
    for (obj_iterator obj = objectiveBegin(); obj != objectiveEnd(); obj++){
        (*obj)->add_vars_involved_in_prod(dvs);
    }
}

void MaxObjective::add_int_vars(dvContainer &dvs) const
{
    m_dvContainer->add_int_vars(dvs);
}

ROCPPObjectiveIF_Ptr MaxObjective::mapObjVars(const map<string,ROCPPVarIF_Ptr > &mapFromOldToNewVars) const
{
    vector<ROCPPExpr_Ptr > newObjFuns;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        newObjFuns.push_back((*objFun)->mapExprVars(mapFromOldToNewVars));
    }
    
    ROCPPObjectiveIF_Ptr newObj(new MaxObjective(newObjFuns));
    return newObj;
}

ROCPPObjectiveIF_Ptr MaxObjective::mapObjUnc(const map<string,ROCPPUnc_Ptr > &mapFromOldToNewUnc) const
{
    vector<ROCPPExpr_Ptr > newObjFuns;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        newObjFuns.push_back((*objFun)->mapExprUnc(mapFromOldToNewUnc));
    }
    
    ROCPPObjectiveIF_Ptr newObj(new MaxObjective( newObjFuns));
    return newObj;
}

ROCPPObjectiveIF_Ptr MaxObjective::mapVars(const map<string, ROCPPExpr_Ptr > &mapFromVarToExpression) const
{
    vector<ROCPPExpr_Ptr > newObjFuns;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        newObjFuns.push_back((*objFun)->mapVars(mapFromVarToExpression));
    }
    
    ROCPPObjectiveIF_Ptr newObj(new MaxObjective( newObjFuns));
    return newObj;
}

ROCPPObjectiveIF_Ptr MaxObjective::mapUncs(const map<string, ROCPPExpr_Ptr > &mapFromUncToExpression) const
{
    vector<ROCPPExpr_Ptr > newObjFuns;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        newObjFuns.push_back((*objFun)->mapUncs(mapFromUncToExpression));
    }
    
    ROCPPObjectiveIF_Ptr newObj(new MaxObjective( newObjFuns));
    return newObj;
}

ROCPPObjectiveIF_Ptr MaxObjective::replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr > &term, ROCPPVarIF_Ptr var) const
{
    vector<ROCPPExpr_Ptr > newObjs;
    
    vector<ROCPPExpr_Ptr >::const_iterator objIterator = m_pObjFuns.begin();
    
    for(; objIterator != m_pObjFuns.end(); objIterator++)
        newObjs.push_back((*objIterator)->replaceTermWithVar(term, var));
    
    ROCPPObjectiveIF_Ptr newObj( new MaxObjective(newObjs));
    
    return newObj;
}

void MaxObjective::convertToEpigraph(ROCPPVarIF_Ptr epigraphVar, vector<ROCPPConstraint_Ptr > &epigraphConstraints) const
{
    epigraphConstraints.clear();
    
    for (vector<ROCPPExpr_Ptr >::const_iterator oit = m_pObjFuns.begin(); oit != m_pObjFuns.end(); oit++)
    {
        ROCPPConstraint_Ptr epicst(new IneqConstraint());
        epicst->add_lhs(*oit);
        epicst->add_lhs(-1., epigraphVar);
        
        epigraphConstraints.push_back(epicst);
    }
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

uint MaxObjective::getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr > &term) const
{
    uint out(0);
    obj_iterator o_it(m_pObjFuns.begin());
    
    for(; o_it != m_pObjFuns.end(); o_it++)
        out += (*o_it)->getNumTimesTermAppears(term);
    
    return out;
}

vector<ROCPPExpr_Ptr > MaxObjective::getObj() const
{
    return m_pObjFuns;
}

ROCPPExpr_Ptr MaxObjective::getObj(uint i) const
{
    if (i == 0)
        throw MyException("index must be positive");
    
    if (i > m_pObjFuns.size())
        throw MyException("index cannot exceed number of objective terms");
    
    return m_pObjFuns[i - 1];
}

bool MaxObjective::hasNonlinearities() const
{
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        if((*objFun)->hasNonlinearities())
            return true;
    }
    return false;
}

bool MaxObjective::hasProdsUncertainties() const
{
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        if((*objFun)->hasProdsUncertainties() )
            return true;
    }
    return false;
}

bool MaxObjective::hasProdsContVars() const
{
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        if((*objFun)->hasProdsContVars() )
            return true;
    }
    return false;
}

bool MaxObjective::isDeterministic() const
{
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        if(!(*objFun)->isDeterministic() )
            return false;
    }
    return true;
}

uint MaxObjective::getNumAdaptiveVars() const
{
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        if((*objFun)->getNumAdaptiveVars() )
            return (*objFun)->getNumAdaptiveVars();
    }
    return 0;
}

uint MaxObjective::getTimeStage() const
{
    uint timeStage = 0;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        timeStage = max(timeStage, (*objFun)->getTimeStage());
    }
    return timeStage;
}

bool MaxObjective::varIsInvolved(string varName, uint i) const
{
    if( i > m_pObjFuns.size() )
        throw MyException("The objective function doesn't exit");
    
    return( ( m_pObjFuns[i - 1]->getDVContainer()->find(varName) != m_pObjFuns[i - 1]->getDVContainer()->end() ) );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPObjectiveIF_Ptr MaxObjective::Clone() const
{
    vector<ROCPPExpr_Ptr > newObjs;

    obj_iterator o_it(m_pObjFuns.begin());
    
    for(; o_it != m_pObjFuns.end(); o_it++)
        newObjs.push_back((*o_it)->Clone());
    
    ROCPPObjectiveIF_Ptr newObj(new MaxObjective(newObjs));
    
    return newObj;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void MaxObjective::WriteToStream(ofstream &ofs)
{
    uint i = 0;
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        ofs << endl;
        ofs << "Obj" << i << ": ";
        (*objFun)->WriteToStream(ofs);
        i++;
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% CREATE VAR FUNCTION %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPObjectiveIF_Ptr createObjective(ROCPPExpr_Ptr objFun)
{
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective(objFun));
    return newObj;
}

ROCPPObjectiveIF_Ptr creatMaxObjective(vector<ROCPPExpr_Ptr > objFuns)
{
    ROCPPObjectiveIF_Ptr newObj(new MaxObjective(objFuns));
    return newObj;
}
