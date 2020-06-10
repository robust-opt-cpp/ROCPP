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
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

uint ObjectiveFunctionIF::getNumTimesTermAppears(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term) const
{
    uint out(0);
    
    vector<boost::shared_ptr<LHSExpression> >::const_iterator objIterator = getObj().begin();
    
    for(; objIterator != getObj().end(); objIterator++)
        out += (*objIterator)->getNumTimesTermAppears(term);
    
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ObjectiveFunctionIF::checkCompatibility(boost::shared_ptr<LHSExpression> pExpression) const
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

SimpleObjective::SimpleObjective(boost::shared_ptr<LHSExpression> objFun) : ObjectiveFunctionIF(simpleObj)
{
    checkCompatibility(objFun);
    m_pObjFun = objFun;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void SimpleObjective::add_to_obj(boost::shared_ptr<DecisionVariableIF> pVar, double cost)
{
    m_pObjFun->add(cost, pVar);
}

void SimpleObjective::add_vars_involved_in_prod(dvContainer &dvs) const
{
    vector<boost::shared_ptr<LHSExpression> > objvec(getObj());
    for (obj_fun_iterator it = objvec.begin(); it!= objvec.end(); it++)
        (*it)->add_vars_involved_in_prod(dvs);
}

void SimpleObjective::add_int_vars(dvContainer &dvs) const
{
    m_pObjFun->getDVContainer()->add_int_vars(dvs);
}

boost::shared_ptr<ObjectiveFunctionIF> SimpleObjective::replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const
{
    boost::shared_ptr<ObjectiveFunctionIF> newObj( new SimpleObjective(m_pObjFun->replaceTermWithVar(term, var)));
    
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> SimpleObjective::mapObjVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const
{
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new SimpleObjective( m_pObjFun->mapExprVars(mapFromOldToNewVars)));
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> SimpleObjective::mapObjUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const
{
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new SimpleObjective( m_pObjFun->mapExprUnc(mapFromOldToNewUnc)));
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> SimpleObjective::mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const
{
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new SimpleObjective( m_pObjFun->mapVars(mapFromVarToExpression)));
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> SimpleObjective::mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const
{
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new SimpleObjective( m_pObjFun->mapUncs(mapFromUncToExpression)));
    return newObj;
}

void SimpleObjective::convertToEpigraph(boost::shared_ptr<DecisionVariableIF> epigraphVar, vector<boost::shared_ptr<ConstraintIF> > &epigraphConstraints) const
{
    epigraphConstraints.clear();
    
    boost::shared_ptr<ConstraintIF> epicst(new IneqConstraint());
    epicst->add_lhs(m_pObjFun);
    epicst->add_lhs(-1., epigraphVar);
    
    epigraphConstraints.push_back(epicst);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

vector<boost::shared_ptr<LHSExpression> > SimpleObjective::getObj() const
{
    vector<boost::shared_ptr<LHSExpression> > con;
    con.push_back(m_pObjFun);
    
    return con;
}

boost::shared_ptr<LHSExpression> SimpleObjective::getObj(uint i) const
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

boost::shared_ptr<ObjectiveFunctionIF> SimpleObjective::Clone() const
{
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new SimpleObjective(getObj(1)->Clone()));
    
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

MaxObjective::MaxObjective(vector<boost::shared_ptr<LHSExpression> > objFuns) : ObjectiveFunctionIF(maxObj), m_dvContainer(new dvContainer()), m_uncContainer(new uncContainer())
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

void MaxObjective::add_to_obj(boost::shared_ptr<DecisionVariableIF> pVar, double cost)
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

boost::shared_ptr<ObjectiveFunctionIF> MaxObjective::mapObjVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const
{
    vector<boost::shared_ptr<LHSExpression> > newObjFuns;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        newObjFuns.push_back((*objFun)->mapExprVars(mapFromOldToNewVars));
    }
    
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new MaxObjective(newObjFuns));
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> MaxObjective::mapObjUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const
{
    vector<boost::shared_ptr<LHSExpression> > newObjFuns;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        newObjFuns.push_back((*objFun)->mapExprUnc(mapFromOldToNewUnc));
    }
    
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new MaxObjective( newObjFuns));
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> MaxObjective::mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const
{
    vector<boost::shared_ptr<LHSExpression> > newObjFuns;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        newObjFuns.push_back((*objFun)->mapVars(mapFromVarToExpression));
    }
    
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new MaxObjective( newObjFuns));
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> MaxObjective::mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const
{
    vector<boost::shared_ptr<LHSExpression> > newObjFuns;
    
    for (obj_iterator objFun = objectiveBegin(); objFun != objectiveEnd(); objFun++){
        newObjFuns.push_back((*objFun)->mapUncs(mapFromUncToExpression));
    }
    
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new MaxObjective( newObjFuns));
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> MaxObjective::replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const
{
    vector<boost::shared_ptr<LHSExpression> > newObjs;
    
    vector<boost::shared_ptr<LHSExpression> >::const_iterator objIterator = m_pObjFuns.begin();
    
    for(; objIterator != m_pObjFuns.end(); objIterator++)
        newObjs.push_back((*objIterator)->replaceTermWithVar(term, var));
    
    boost::shared_ptr<ObjectiveFunctionIF> newObj( new MaxObjective(newObjs));
    
    return newObj;
}

void MaxObjective::convertToEpigraph(boost::shared_ptr<DecisionVariableIF> epigraphVar, vector<boost::shared_ptr<ConstraintIF> > &epigraphConstraints) const
{
    epigraphConstraints.clear();
    
    for (vector<boost::shared_ptr<LHSExpression> >::const_iterator oit = m_pObjFuns.begin(); oit != m_pObjFuns.end(); oit++)
    {
        boost::shared_ptr<ConstraintIF> epicst(new IneqConstraint());
        epicst->add_lhs(*oit);
        epicst->add_lhs(-1., epigraphVar);
        
        epigraphConstraints.push_back(epicst);
    }
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

vector<boost::shared_ptr<LHSExpression> > MaxObjective::getObj() const
{
    return m_pObjFuns;
}

boost::shared_ptr<LHSExpression> MaxObjective::getObj(uint i) const
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

boost::shared_ptr<ObjectiveFunctionIF> MaxObjective::Clone() const
{
    vector<boost::shared_ptr<LHSExpression> > newObjs;

    vector<boost::shared_ptr<LHSExpression> >::const_iterator o_it(getObj().begin());
    
    for(; o_it != getObj().end(); o_it++)
        newObjs.push_back((*o_it)->Clone());
    
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new MaxObjective(newObjs));
    
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

boost::shared_ptr<ObjectiveFunctionIF> createObjective(boost::shared_ptr<LHSExpression> objFun)
{
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new SimpleObjective(objFun));
    return newObj;
}

boost::shared_ptr<ObjectiveFunctionIF> creatMaxObjective(vector<boost::shared_ptr<LHSExpression> > objFuns)
{
    boost::shared_ptr<ObjectiveFunctionIF> newObj(new MaxObjective(objFuns));
    return newObj;
}
