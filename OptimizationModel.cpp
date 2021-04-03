//
//  OptimizationModel.cpp
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
#include "OptimizationModel.hpp"
#include "helpersOpt.hpp"

#include <fstream>
#include <math.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% OPTIMIZATIONMODEL INTERFACE %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OptimizationModelIF::OptimizationModelIF(uint numTimeStages) : m_pDVContainer ( new dvContainer() ), m_numTimeStages(numTimeStages),m_pObj(new SimpleObjective() ), m_pUncContainer(new uncContainer() ), m_dduContainer( new uncContainer() ), m_nondduContainer( new uncContainer() )
{
    ROCPPExpr_Ptr obj(new LHSExpression() );
    obj->add(0.);
    ROCPPObjectiveIF_Ptr initial(new SimpleObjective(obj) );
    m_pObj = initial;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OptimizationModelIF::varsIterator OptimizationModelIF::varsBegin() const {return m_pDVContainer->begin();}

OptimizationModelIF::varsIterator OptimizationModelIF::varsEnd() const {return m_pDVContainer->end();}

OptimizationModelIF::uncertaintiesIterator OptimizationModelIF::uncertaintiesBegin() const
{
    throw MyException("Deterministic Model does not have uncertainty");
}

OptimizationModelIF::uncertaintiesIterator OptimizationModelIF::uncertaintiesEnd() const
{
    throw MyException("Deterministic Model does not have uncertainty");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void OptimizationModelIF::checkCompatibility(ROCPPConstraint_Ptr pConstraint) const
{
    if ( pConstraint->hasProdsUncertainties() )
        throw MyException("products of uncertainties not allowed");
    
    if ( pConstraint->hasProdsContVars() )
        throw MyException("products of real valued variables not allowed");
}

void OptimizationModelIF::checkCompatibility(ROCPPVarIF_Ptr pVariable) const
{
    if ( pVariable->getTimeStage() > m_numTimeStages )
        throw MyException("The decision variable occurs at a time stage greater than the number of stages in the problem");
}

void OptimizationModelIF::checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const
{
    if ( pObjFun->hasProdsUncertainties() )
        throw MyException("products of uncertainties not allowed");
    
    if ( pObjFun->hasProdsContVars() )
        throw MyException("products of real valued variables not allowed");
    
    if( pObjFun->getTimeStage() > m_numTimeStages)
        throw MyException("time stage of the objective function exceeds the optimization model");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void OptimizationModelIF::add_constraint(ROCPPConstraint_Ptr pConstraint)
{
    if ( !pConstraint->isWellDefined() )
        throw MyException( "attempted to add a badly defined constraint to optimization model");
    
    if ( pConstraint->isClassicConstraint() )
    {
        ROCPPClassicConstraint_Ptr classicCstr = dynamic_pointer_cast<ClassicConstraintIF>(pConstraint);
        
        if (classicCstr->isEqConstraint()){
            if ( (classicCstr->getNumAdaptiveContVars()>0)  && (! classicCstr->definesUncertaintySet()) )
                throw MyException("cannot have adaptive real-valued variables in equality constraints that do not define the uncertainty-set");
            
            if ( (classicCstr->getNumUncertainties() >0) && (!classicCstr->definesUncertaintySet()) )
                throw MyException("cannot have uncertainties in equality constraints that do not define the uncertainty set");
        }
    }
    
    checkCompatibility(pConstraint);
    
    map<string, ROCPPVarIF_Ptr > varMap = createVarMap(pConstraint);
    ROCPPConstraint_Ptr tempConstraint = pConstraint->mapVars(varMap);
    
    map<string, ROCPPUnc_Ptr > uncMap = createUncMap(pConstraint);
    
    ROCPPConstraint_Ptr newConstraint;
    
    if(uncMap.size() >= 1){
        newConstraint = tempConstraint->mapUnc(uncMap);
        tempConstraint.reset();
    }
    else
        newConstraint = tempConstraint;
    
    push_constraint(newConstraint);
    if(newConstraint->definesUncertaintySet() )
        m_uncertaintySet.push_back(newConstraint);
}

void OptimizationModelIF::add_soc_constraint(ROCPPVarIF_Ptr coneHead, const vector<ROCPPVarIF_Ptr > &otherVars)
{
    ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
    
    if (!otherVars.empty())
    {
        vector<ROCPPExpr_Ptr > vec;
        for (vector<ROCPPVarIF_Ptr >::const_iterator it = otherVars.begin(); it != otherVars.end(); it++)
        {
            ROCPPExpr_Ptr exp(new LHSExpression() );
            exp->add( ROCPPCstrTerm_Ptr( new ProductTerm(1.,*it)));
            vec.push_back(exp);
        }
        
        ROCPPCstrTerm_Ptr pNT( new NormTerm(vec) );
        
        cstr->add_lhs(pNT);
    }
    else
    {
        // if there are no other variables and the lower bound of the current variable is 0, don't do anything
        double lb = coneHead->getLB();
        if ( lb < 1.e-10)
            return;
    }
    
    cstr->add_lhs(-1.,coneHead);
    cstr->set_rhs(make_pair(0.,true));
    add_constraint(cstr);
}

void OptimizationModelIF::add_constraints(vector<ROCPPConstraint_Ptr >::const_iterator first, vector<ROCPPConstraint_Ptr >::const_iterator last)
{
    for (vector<ROCPPConstraint_Ptr >::const_iterator it = first; it != last; it++)
        add_constraint(*it);
}

void OptimizationModelIF::add_epigraph()
{
    ROCPPVarIF_Ptr pEpi(new VariableDouble("epigraph") );
    for (auto& lhs : getObj()->getObj() ){
        ROCPPConstraint_Ptr pCstr(new IneqConstraint() );
        pCstr->add_lhs(lhs);
        pCstr->add_lhs(-1.,pEpi);
        pCstr->set_rhs(make_pair(0.,true));
        add_constraint(pCstr);
    }
    ROCPPExpr_Ptr newObjFun(new LHSExpression() );
    newObjFun->add(1.0, pEpi);
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective(newObjFun) );
    set_objective(newObj);
}

void OptimizationModelIF::add_ddu(ROCPPUnc_Ptr pUncertainty, uint firstTimeStageObservable, uint lastTimeStageObservable, const map<uint, double> &obsCosts)
{
    throw MyException("No ddu in the non ddu type model");
}

void OptimizationModelIF::set_ddu(ROCPPOptModelIF_Ptr pIn)
{
}

void OptimizationModelIF::set_ddu(const map< pair<string,uint>, measPair > &dduToMeasMap, const map< string, pair<uint,uint> > &dduStagesObs)
{
}

void OptimizationModelIF::set_objType(uncOptModelObjType pType)
{
    throw MyException("objective type only relevant for uncertain model");
}

void OptimizationModelIF::pair_uncertainties(ROCPPUnc_Ptr u1, ROCPPUnc_Ptr u2)
{
    throw MyException("No ddu in the non ddu type model");
}

void OptimizationModelIF::add_constraint_uncset(ROCPPConstraint_Ptr pUncCstr)
{
    pUncCstr->setParams(true, false);
    
    add_constraint(pUncCstr);
}

void OptimizationModelIF::set_objective(ROCPPObjectiveIF_Ptr pObj)
{
    
    checkCompatibility(pObj);
    
    map<string, ROCPPVarIF_Ptr > varMap = createVarMap(pObj);
    ROCPPObjectiveIF_Ptr tempObjFun = pObj->mapObjVars(varMap);
    map<string, ROCPPUnc_Ptr > uncMap = createUncMap(pObj);
    
    ROCPPObjectiveIF_Ptr newObjFun;
    if(uncMap.size() >= 1){
        newObjFun = tempObjFun->mapObjUnc(uncMap);
        tempObjFun.reset();
    }
    else
        newObjFun = tempObjFun;
    
    m_pObj = newObjFun;
}

void OptimizationModelIF::set_objective(ROCPPExpr_Ptr objFun)
{
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective(objFun));
    set_objective(newObj);
}

void OptimizationModelIF::set_objective(vector<ROCPPExpr_Ptr > objFuns)
{
    ROCPPObjectiveIF_Ptr newObj(new MaxObjective(objFuns));
    set_objective(newObj);
}


ROCPPOptModelIF_Ptr OptimizationModelIF::replaceTermWithVar(const multimap<string, ROCPPVarIF_Ptr > &term, ROCPPVarIF_Ptr var) const
{
    ROCPPOptModelIF_Ptr pOut;
    if (isUncertainOptimizationModel())
        pOut = InstanciateModel(getType(),getNumTimeStages(),getObjType());
    else
        pOut = InstanciateModel(getType(),getNumTimeStages(),robust);
    
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
    {
        bool termIsNonlinear( term.size()>1 );
        
        // if the term is nonlinear but the constraint isn't, just add the constraint as is
        if ( ( termIsNonlinear ) && ( !(*c_it)->hasNonlinearities()) )
            pOut->add_constraint( *c_it);
        else
            pOut->add_constraint( (*c_it)->replaceTermWithVar(term,var) );
    }
    
    pOut->set_objective(getObj() );
    
    return pOut;
}

map<string, ROCPPVarIF_Ptr > OptimizationModelIF::createVarMap(ROCPPConstraint_Ptr pConstraint)
{
    map<string, ROCPPVarIF_Ptr > varMap;
    
    for(ConstraintIF::varsIterator vit = pConstraint->varsBegin(); vit != pConstraint->varsEnd(); vit++)
    {
        if (varIsDefined(vit->first)){
            varMap.insert(make_pair(vit->first, this->getVar(vit->first)));
        }
        else{
            ROCPPVarIF_Ptr newvar( vit->second->Clone());
            add_var(newvar);
            varMap.insert(make_pair(vit->first, newvar));
        }
    }
    
    return varMap;
}

map<string, ROCPPVarIF_Ptr > OptimizationModelIF::createVarMap(ROCPPObjectiveIF_Ptr objFun)
{
    map<string, ROCPPVarIF_Ptr > varMap;
    
    for(ConstraintTermIF::dvIterator vit = objFun->varsBegin(); vit != objFun->varsEnd(); vit++)
    {
        if (varIsDefined(vit->first)){
            varMap.insert(make_pair(vit->first, getVar(vit->first)));
        }
        else{
            ROCPPVarIF_Ptr newvar( vit->second->Clone());
            add_var(newvar);
            varMap.insert(make_pair(vit->first, newvar));
        }
    }
    
    return varMap;
}

map<string, ROCPPUnc_Ptr > OptimizationModelIF::createUncMap(ROCPPConstraint_Ptr pConstraint)
{
    map<string, ROCPPUnc_Ptr > uncMap;
    
    return uncMap;
}

map<string, ROCPPUnc_Ptr > OptimizationModelIF::createUncMap(ROCPPObjectiveIF_Ptr objFun)
{
    map<string, ROCPPUnc_Ptr > uncMap;
    
    return uncMap;
}

void OptimizationModelIF::getExpectation()
{
    cout << "Your objective is already derministic now" << endl;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool OptimizationModelIF::isObservable(string uncName) const
{
    throw MyException("Deterministic model does not have uncertainty");
}

bool OptimizationModelIF::isAdaptive(string varName) const
{
    ROCPPVarIF_Ptr dv = getVar(varName);
    return (dv->isAdaptive());
}

bool OptimizationModelIF::isInStandardForm() const
{
    for (constraintIterator cit = constraintBegin(); cit != constraintEnd(); cit++)
    {
        if ( (*cit)->isClassicConstraint() )
        {
            ROCPPClassicConstraint_Ptr pClassic ( static_pointer_cast<ClassicConstraintIF>(*cit) );
            if ( pClassic->hasNormTerm() )
                return false;
        }
    }
    
    return true;
}

bool OptimizationModelIF::varIsDefined(string varName) const
{
    return( ( m_pDVContainer->find(varName) != m_pDVContainer->end() ) );
}

bool OptimizationModelIF::VarInObj(string varName, uint i) const
{
    return (m_pObj->varIsInvolved(varName, i) );
}

ROCPPVarIF_Ptr OptimizationModelIF::getMeasVar(string dduncName, uint timeStage) const
{
    throw MyException("None ddu model does not have this function");
}

ROCPPVarIF_Ptr OptimizationModelIF::getVar(string varName) const
{
    return( m_pDVContainer->findthrow(varName)->second );
}

ROCPPUnc_Ptr OptimizationModelIF::getUnc(string uncName) const
{
    throw MyException("Deterministic model does not have uncertainty");
}

uint OptimizationModelIF::getNumTimesTermAppears(const multimap<string, ROCPPVarIF_Ptr > &term) const
{
    uint out(0);
    
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
        out += (*c_it)->getNumTimesTermAppears(term);
    
    out += m_pObj->getNumTimesTermAppears(term);
        
    return out;
}

void OptimizationModelIF::getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, ROCPPVarIF_Ptr > > &termMap) const
{
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
        (*c_it)->getAllProductsOf2Variables(freqMap,termMap);
}

map< pair<string,uint>, measPair > OptimizationModelIF::getDDUToMeasMap() const
{
    throw MyException("No DDU map in non-ddu type model");
}

map<string, pair<uint,uint> > OptimizationModelIF::getdduStagesObs() const
{
    //throw MyException("No DDU map in non-ddu type model");
    return m_dduStagesObs;
}

uint OptimizationModelIF::getFirstStageObservable(string uncName) const
{
    throw MyException("No uncertainty in deterministic model");
}

uint OptimizationModelIF::getLastStageObservable(string uncName) const
{
    throw MyException("No uncertainty in deterministic model");
}

uncOptModelObjType OptimizationModelIF::getObjType() const
{
    throw MyException("problem is not uncertain");
    return robust;
}

ROCPPuncContainer_Ptr OptimizationModelIF::getUncContainer() const
{
    throw MyException("problem is not uncertain");
}

uint OptimizationModelIF::getNumContVars() const {return m_pDVContainer->getNumContVars();}
uint OptimizationModelIF::getNumIntVars() const {return m_pDVContainer->getNumIntVars();}
uint OptimizationModelIF::getNumBoolVars() const {return m_pDVContainer->getNumBoolVars();}
uint OptimizationModelIF::getNumAdaptiveContVars() const {return m_pDVContainer->getNumAdaptiveContVars();}
uint OptimizationModelIF::getNumAdaptiveVars() const {return m_pDVContainer->getNumAdaptiveVars();}
size_t OptimizationModelIF::getNumVars() const {return m_pDVContainer->getNumVars();}

bool OptimizationModelIF::hasNonlinearities() const
{
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
        if ( (*c_it)->hasNonlinearities() )
            return true;
    
    return false;
}

bool OptimizationModelIF::hasRectangularUncertaintySet() const
{
    throw MyException("This model doesn't have uncertainty set");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPOptModelIF_Ptr OptimizationModelIF::Clone() const
{
    ROCPPOptModelIF_Ptr pOut;
    if (isUncertainOptimizationModel())
        pOut = InstanciateModel(getType(),getNumTimeStages(),getObjType());
    else
        pOut = InstanciateModel(getType(),getNumTimeStages(),robust);
    
    for (OptimizationModelIF::constraintIterator cit = constraintBegin(); cit != constraintEnd(); cit++)
        pOut->add_constraint( (*cit)->Clone() );
    
    pOut->set_objective(getObj());
     
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void OptimizationModelIF::WriteToFile(string folderName, string fileName) const
{
    string filePath = folderName;
    if (folderName!="")
        filePath += "/";
    
    ofstream ofs((filePath+fileName+".rob").c_str() );
    ofs.clear();
    //ofs << scientific;
    //ofs.precision(30);
    ofs.setf( ios::showpos );
    //ofs.showpos(true);
    // write objective
    ofs << "Objective:" << endl;
    ofs << "min ";
    
    if (isUncertainOptimizationModel())
    {
        if (getObjType() == robust)
            ofs << "max";
        else if (getObjType() == stochastic)
            ofs << "E";
        else
            throw MyException("What's your objective function type?");
    }
    
    getObj()->WriteToStream(ofs);
    
    ofs << endl;
    ofs << endl;
    
    // write constraints
    
    ofs << "Constraints:" << endl;
    uint ccnt(0);
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
    {
        if ( !(*c_it)->definesUncertaintySet() )
        {
            if ( (*c_it)->isClassicConstraint() )
                (*c_it)->WriteToStream(ofs,ccnt++);
        }
    }
    
    ccnt=0;
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
    {
        if ( !(*c_it)->definesUncertaintySet() )
        {
            if ( !(*c_it)->isClassicConstraint() )
                (*c_it)->WriteToStream(ofs,ccnt++);
        }
    }
    
    ofs << endl;
    
    if(isUncertainOptimizationModel())
    {
        ofs << "Uncertainty Set:" << endl;
        
        ccnt=0;
        for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
        {
            if ( (*c_it)->definesUncertaintySet() )
            {
                (*c_it)->WriteToStream(ofs,ccnt++);
            }
        }
        
        ofs << endl;
    }
    
    ofs << "Decision Variables:" << endl;
    
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++)
    {
        string adapt((v_it->second->isAdaptive())?("Adaptive"):("Static"));
        string dvtype( (v_it->second->isBooleanVar())?("Boolean"):((v_it->second->isIntegerVar()?("Integer"):("Real-valued"))) );
        ofs << v_it->second->getName() << ": " << dvtype << ", " << adapt << ", " << v_it->second->getTimeStage() << endl;
    }
    
    
    ofs << endl;
    ofs << "Bounds:" << endl;
    
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++)
    {
        
        ofs << v_it->second->writeLB() << " <= " << v_it->second->getName() << " <= " << v_it->second->writeUB() << endl;
    }
    
    if (isUncertainOptimizationModel())
    {
        ROCPPUncOptModel_Ptr pModelUnc = static_pointer_cast<UncertainOptimizationModel>(this->Clone());
        
        ofs << endl;
        ofs << "Uncertainties:" << endl;
        
        for (UncertainOptimizationModel::uncertaintiesIterator u_it = pModelUnc->uncertaintiesBegin(); u_it != pModelUnc->uncertaintiesEnd(); u_it++)
        {
            string isobs((u_it->second->isObservable())?("Observable"):("Not Observable"));
            ofs << u_it->second->getName() << ": " << isobs << ", " << u_it->second->getTimeStage() << endl;
        }
    }
    
    ofs.close();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% Protected Functions %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void OptimizationModelIF::add_var(ROCPPVarIF_Ptr pVariable)
{
    checkCompatibility(pVariable);
    *m_pDVContainer += pVariable; // this will also check whether a different variable with the same name exists
}

void OptimizationModelIF::add_vars(ROCPPconstdvContainer_Ptr pDVcontainer)
{
    for (dvContainer::const_iterator it = pDVcontainer->begin(); it != pDVcontainer->end(); it++)
        add_var( it->second );
    
}

void OptimizationModelIF::add_ddu_obj(ROCPPVarIF_Ptr pVar, double cost)
{
    m_pObj->add_to_obj(pVar, cost);
}

void OptimizationModelIF::push_constraint(ROCPPConstraint_Ptr pConstraint)
{
    m_constraints.push_back(pConstraint);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%% Deterministic Optimization Model %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void DeterministicOptimizationModel::checkCompatibility(ROCPPConstraint_Ptr pConstraint) const
{
    if (!pConstraint->isDeterministic())
        throw MyException("cannot add non deterministic constraint to a simple optimization model");
    
    if (pConstraint->definesUncertaintySet())
        throw MyException("cannot add uncertainty set constraint");
    
    if (pConstraint->getNumAdaptiveVars()!=0)
        throw MyException("cannot add adaptive variables to a DeterministicOptimizationModel");
}

void DeterministicOptimizationModel::checkCompatibility(ROCPPVarIF_Ptr pVariable) const
{
    if (pVariable->isAdaptive())
        throw MyException("cannot add adaptive variable to DeterministicOptimizationModel");
}

void DeterministicOptimizationModel::checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const
{
    OptimizationModelIF::checkCompatibility(pObjFun);
    
    if (!pObjFun->isDeterministic())
        throw MyException("cannot add non deterministic objective to a simple optimization model");
    
    // check that it does not contain bilinear terms
    if (pObjFun->getNumAdaptiveVars()!=0)
        throw MyException("cannot add adaptive variables to a DeterministicOptimizationModel");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% Uncertain Optimization Model %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

UncertainOptimizationModel::UncertainOptimizationModel(uint numTimeStages, uncOptModelObjType objType) :
OptimizationModelIF(numTimeStages), m_objType(objType){}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OptimizationModelIF::uncertaintiesIterator UncertainOptimizationModel::uncertaintiesBegin() const
{
    return m_pUncContainer->begin();
}


OptimizationModelIF::uncertaintiesIterator UncertainOptimizationModel::uncertaintiesEnd() const
{
    return m_pUncContainer->end();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void UncertainOptimizationModel::checkCompatibility(ROCPPConstraint_Ptr pConstraint) const
{
    if (!pConstraint->definesUncertaintySet())
    {
        //if ( (!pConstraint->isDeterministic()) && (pConstraint->getNumAdaptiveVars()!=0) )
        if (pConstraint->hasNormTerm())
            throw MyException("cannot add such constraints to uncertain model");
    }
    else
    {
        if (pConstraint->isDeterministic())
            cout << "Warning: You are adding an uncertainty set constraint without any uncertainty";
    }
    
    if( !pConstraint->definesUncertaintySet()&&pConstraint->hasProdsUncertainties())
        throw MyException("cannot add prods of uncertainty to uncertain model");
    
}

void UncertainOptimizationModel::checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const
{
    OptimizationModelIF::checkCompatibility(pObjFun);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

map<string, ROCPPUnc_Ptr > UncertainOptimizationModel:: createUncMap(ROCPPConstraint_Ptr pConstraint)
{
    map<string, ROCPPUnc_Ptr > uncMap;
    
    for(ConstraintIF::uncertaintiesIterator uit = pConstraint->uncertaintiesBegin(); uit != pConstraint->uncertaintiesEnd(); uit++)
    {
        if (uncIsDefined(uit->first)){
            uncMap.insert(make_pair(uit->first, this->getUnc(uit->first)));
        }
        else{
            ROCPPUnc_Ptr newunc( uit->second->Clone());
            add_uncertainty(newunc);
            uncMap.insert(make_pair(uit->first, newunc));
        }
    }
    
    return uncMap;
}

map<string, ROCPPUnc_Ptr > UncertainOptimizationModel:: createUncMap(ROCPPObjectiveIF_Ptr objFun)
{
    map<string, ROCPPUnc_Ptr > uncMap;
    
    for(ConstraintTermIF::uncIterator uit = objFun->uncBegin(); uit != objFun->uncEnd(); uit++)
    {
        if (uncIsDefined(uit->first)){
            uncMap.insert(make_pair(uit->first, getUnc(uit->first)));
        }
        else{
            ROCPPUnc_Ptr newunc( uit->second->Clone());
            add_uncertainty(newunc);
            uncMap.insert(make_pair(uit->first, newunc));
        }
    }
    
    return uncMap;
}

void UncertainOptimizationModel::getExpectation()
{
    if(getObjType() != stochastic)
        throw MyException("Can only get expectation for stochastic problem.");
    
    map<string, pair<double, double> > margSupp;
    map<string,uint> numPartitionsMap;
    
    ROCPPOptModelIF_Ptr pModel(this->Clone());
    
    findWholeMarginalSupport(pModel, numPartitionsMap, margSupp);
    
    ObjectiveFunctionIF::uncsIterator uit(getObj()->uncBegin());
    
    map<string, pair<double, double> >::const_iterator mit;
    
    double meanValue;
    
    map<string, ROCPPExpr_Ptr > mapFromUncToExpression;
    
    for(; uit!=getObj()->uncEnd(); uit++)
    {
        mit = margSupp.find(uit->first);
        
        if(mit == margSupp.end())
            throw MyException("No mean value found for the uncertainty "+uit->first);
        
        meanValue = (mit->second.second + mit->second.first)/2.0;
        
        ROCPPExpr_Ptr mean(new LHSExpression());
        mean->add(meanValue);
        
        mapFromUncToExpression.insert(make_pair(uit->first, mean));
    }
    
    OptimizationModelIF::set_objective(getObj()->mapUncs(mapFromUncToExpression));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

size_t UncertainOptimizationModel::getNumUncertainties() const
{
    return m_pUncContainer->getNumUncertainties();
}

uint UncertainOptimizationModel::getAlphabeticalLocation(ROCPPUnc_Ptr pUncertainty) const
{
    return m_pUncContainer->getAlphabeticalLocation(pUncertainty->getName());
}

uint UncertainOptimizationModel::getObservableAlphabeticalLocation(ROCPPUnc_Ptr pUncertainty) const
{
    return m_pUncContainer->getObservableAlphabeticalLocation(pUncertainty->getName());
}

uint UncertainOptimizationModel::getFirstStageObservable(string uncName) const
{
    return 1;
}

uint UncertainOptimizationModel::getLastStageObservable(string uncName) const
{
    return getNumTimeStages();
}

size_t UncertainOptimizationModel::getNumObsUncertainties() const
{
    return m_pUncContainer->getNumObsUncertainties();
}

void UncertainOptimizationModel::getVarsAffectingUncSet(dvContainer& dvs)
{
    for (uncertaintySetIterator usit = uncertaintySetBegin(); usit != uncertaintySetEnd(); usit++)
    {
        dvs += *((*usit)->getDVContainer());
    }
}

bool UncertainOptimizationModel::uncIsDefined(string uncName) const
{
    return( ( m_pUncContainer->find(uncName) != m_pUncContainer->end() ) );
}

bool UncertainOptimizationModel::isObservable(string uncName) const
{
    return m_pUncContainer->isObservable(uncName);
}

bool UncertainOptimizationModel::hasRectangularUncertaintySet() const
{
    bool out(true);
    for (uncertaintySetIterator us_it = uncertaintySetBegin(); us_it != uncertaintySetEnd(); us_it++)
    {
        size_t nu((*us_it)->getNumUncertainties());
        if (nu > 1)
            out = false;
    }
    return out;
}

ROCPPUnc_Ptr UncertainOptimizationModel::getUnc(string uncName) const
{
    return ( m_pUncContainer->findthrow(uncName)->second );
}

ROCPPuncContainer_Ptr UncertainOptimizationModel::getObsUncContainer() const
{
    ROCPPuncContainer_Ptr tmp(getUncContainer());
    ROCPPuncContainer_Ptr out( new uncContainer() );
    
    for (uncContainer::const_iterator it = tmp->begin(); it!= tmp->end(); it++)
    {
        if (it->second->isObservable())
            *out += it->second;
    }
    
    return out;
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% Protected Functions %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void UncertainOptimizationModel::add_uncertainty(ROCPPUnc_Ptr pUncertainty)
{
    *m_pUncContainer += pUncertainty;
    
    if(pUncertainty->isObservable() && m_dduStagesObs.find(pUncertainty->getName()) == m_dduStagesObs.end())
    {
        m_dduStagesObs.insert(make_pair(pUncertainty->getName(), make_pair(pUncertainty->getTimeStage(), getNumTimeStages())));
    }
}

void UncertainOptimizationModel::add_uncertainties(ROCPPconstuncContainer_Ptr pUncContainer)
{
    for (uncContainer::const_iterator it = pUncContainer->begin(); it != pUncContainer->end(); it++)
        add_uncertainty(it->second);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% Simple Uncertain Optimization Model %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void SimpleUncertainOptimizationModel::checkCompatibility(ROCPPConstraint_Ptr pConstraint) const
{
    UncertainOptimizationModel::checkCompatibility(pConstraint);
    
    if (pConstraint->hasNonlinearities())
        throw MyException("SimpleUncertainOptimizationModel should not have bilinear terms");

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%% Simple Uncertain Single Stage Optimization Model %%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void SimpleUncertainSingleStageOptimizationModel::checkCompatibility(ROCPPVarIF_Ptr pVariable) const
{
    OptimizationModelIF::checkCompatibility(pVariable);
    
    if (pVariable->isAdaptive())
        throw MyException("cannot add adaptive variable to single stage model");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%% Uncertain Single Stage Optimization Model %%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void UncertainSingleStageOptimizationModel::checkCompatibility(ROCPPVarIF_Ptr pVariable) const
{
    OptimizationModelIF::checkCompatibility(pVariable);
    
    if (pVariable->isAdaptive())
        throw MyException("cannot add adaptive variable to single stage model");
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% MISOC Optimization Model %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void MISOCP::checkCompatibility(ROCPPConstraint_Ptr pConstraint) const
{
    DeterministicOptimizationModel::checkCompatibility(pConstraint);
    
    // check that it does not contain bilinear terms
    if (pConstraint->hasNonlinearities())
        throw MyException("cannot add nonlinear terms to an MISOCP problem");
}

void MISOCP::checkCompatibility(ROCPPVarIF_Ptr pVariable) const
{
    DeterministicOptimizationModel::checkCompatibility(pVariable);
}

void MISOCP::checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const
{
    DeterministicOptimizationModel::checkCompatibility(pObjFun);
    
    // check that it does not contain bilinear terms
    if (pObjFun->hasNonlinearities())
        throw MyException("cannot add nonlinear terms to an MISOCP problem");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% Bilinear MISOC Optimization Model %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Bilinear_MISOCP::checkCompatibility(ROCPPConstraint_Ptr pConstraint) const
{
    DeterministicOptimizationModel::checkCompatibility(pConstraint);
}

void Bilinear_MISOCP::checkCompatibility(ROCPPVarIF_Ptr pVariable) const
{
    DeterministicOptimizationModel::checkCompatibility(pVariable);
}

void Bilinear_MISOCP::checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const
{
    DeterministicOptimizationModel::checkCompatibility(pObjFun);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% CPLEX MISOC Optimization Model %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

CPLEXMISOCP::CPLEXMISOCP(ROCPPMISOCP_Ptr pIn, string baseVarNme) : m_baseVarNme(baseVarNme)
{
    
    uint ccnt(0);
    
    for (OptimizationModelIF::constraintIterator cit = pIn->constraintBegin(); cit != pIn->constraintEnd(); cit++)
    {
        
        if ( (*cit)->isClassicConstraint() ) // may be used even if no cone-head : results in the addition of a slack variable
        {
            
            ROCPPClassicConstraint_Ptr pClassic ( static_pointer_cast<ClassicConstraintIF>(*cit) );
            
            if ( pClassic->hasNormTerm() )
            {
                
                // first, build conehead
                ROCPPProdTerm_Ptr coneHead;
                ROCPPExpr_Ptr conehead_expr( pClassic->getLinearPart() );
                (*conehead_expr) *= -1.;
                
                if ( (conehead_expr->getNumTerms()==1) && (!(*conehead_expr->begin())->hasNonlinearities() ) )
                {
                    ROCPPCstrTerm_Ptr tmp( *conehead_expr->begin() );
                    if (!tmp->isProductTerm())
                        throw MyException("linear part should not involve norm term");
                    
                    coneHead = static_pointer_cast<ProductTerm>(tmp);
                }
                else// use a conehead to replace the expression.
                {
                    ROCPPVarIF_Ptr coneHeadVar = ROCPPVarIF_Ptr( new VariableDouble( m_baseVarNme + "_" + to_string(++ccnt) + "_0", 0.) );
                    coneHead = ROCPPProdTerm_Ptr( new ProductTerm(1., coneHeadVar) );
                    ROCPPClassicConstraint_Ptr cstr( new EqConstraint() );
                    cstr->add_lhs( -1., coneHeadVar );
                    cstr->add_lhs(conehead_expr);
                    cstr->set_rhs(make_pair(0.,true));
                    add_constraint(cstr);
                }
                
                // then build variables for norm term elements
                
                vector<ROCPPProdTerm_Ptr > nonHeadVarsVec;
                ROCPPNormTerm_Ptr pNT( pClassic->getNormTerm() );
                
                uint tnum(0);
                for (NormTerm::const_iterator it = pNT->begin(); it != pNT->end(); it++)
                {
                    ROCPPProdTerm_Ptr nonHead;
                    ROCPPExpr_Ptr nonhead_expr( *it );
                    
                    if ( (nonhead_expr->getNumTerms()==1) && (!(*nonhead_expr->begin())->hasNonlinearities() ) )
                    {
                        ROCPPCstrTerm_Ptr tmp( *nonhead_expr->begin() );
                        if (!tmp->isProductTerm())
                            throw MyException("norm term expression should not involve norm term");
                        
                        nonHead = static_pointer_cast<ProductTerm>(tmp);
                        nonHeadVarsVec.push_back(nonHead);
                    }
                    else// if (nonhead_expr->getNumTerms()>1)
                    {
                        ROCPPVarIF_Ptr nonHeadVar( new VariableDouble( m_baseVarNme + "_" + to_string(++ccnt) + "_" + to_string(++tnum) ) );
                        nonHeadVarsVec.push_back( ROCPPProdTerm_Ptr( new ProductTerm(1.,nonHeadVar) ) );
                        ROCPPExpr_Ptr var_expr( nonhead_expr );
                        (*var_expr) *= (-1.);
                        ROCPPClassicConstraint_Ptr cstr( new EqConstraint() );
                        cstr->add_lhs( 1., nonHeadVar );
                        cstr->add_lhs(var_expr);
                        cstr->set_rhs(make_pair(0.,true));
                        add_constraint(cstr);
                    }
                }
                
                // add the cone constraint
                add_cplexsoc_constraint( coneHead, nonHeadVarsVec );
            }
            else
            {
                add_constraint(*cit);
            }
        }
        else
            add_constraint(*cit);
    }
    set_objective(pIn->getObj() );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void CPLEXMISOCP::checkCompatibility(ROCPPConstraint_Ptr pConstraint) const
{
    DeterministicOptimizationModel::checkCompatibility(pConstraint);
}

void CPLEXMISOCP::checkCompatibility(ROCPPVarIF_Ptr pVariable) const
{
    DeterministicOptimizationModel::checkCompatibility(pVariable);
}

void CPLEXMISOCP::checkCompatibility(ROCPPObjectiveIF_Ptr pObjFun) const
{
    DeterministicOptimizationModel::checkCompatibility(pObjFun);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void CPLEXMISOCP::add_cplexsoc_constraint(ROCPPVarIF_Ptr coneHead, const vector<ROCPPVarIF_Ptr > &otherVars)
{
    ROCPPProdTerm_Ptr pPTconeHead( new ProductTerm(1., coneHead) );
    vector< ROCPPProdTerm_Ptr > pPTother;
    
    for (vector<ROCPPVarIF_Ptr >::const_iterator it = otherVars.begin(); it != otherVars.end(); it++)
    {
        pPTother.push_back( ROCPPProdTerm_Ptr( new ProductTerm( 1., *it) ) );
    }
    
    add_cplexsoc_constraint( pPTconeHead, pPTother );
}

void CPLEXMISOCP::add_cplexsoc_constraint(ROCPPProdTerm_Ptr coneHead, const vector<ROCPPProdTerm_Ptr > &otherVars)
{
    // this is correct, but it will imply that we cannot have soc constraints in MISOCP problems since they involve nonlinearities
    {
        if (coneHead->hasNonlinearities())
            throw MyException("cone head cannot have nonlinearities");
        
        if (coneHead->getNumUncertainties()!=0)
            throw MyException("cone head cannot have uncertainties");
        
        if (coneHead->getNumVars()>0)
        {
            ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
            ROCPPExpr_Ptr expr( new LHSExpression() );
            (*expr) += ROCPPCstrTerm_Ptr(coneHead);
            (*expr) *= -1.;
            cstr->add_lhs( expr );
            cstr->set_rhs( make_pair(0.,true) );
            add_constraint(cstr);
        }
    }
    {
        // cone head^2 >= sum non cone head ^2
        ROCPPClassicConstraint_Ptr cstr( new IneqConstraint() );
        
        {
            ROCPPExpr_Ptr expr( new LHSExpression() );
            (*expr) += ROCPPCstrTerm_Ptr(coneHead);
            (*expr) *= ROCPPCstrTerm_Ptr(coneHead);
            (*expr) *= -1.;
            cstr->add_lhs( expr );
        }
        
        for (vector<ROCPPProdTerm_Ptr >::const_iterator vit = otherVars.begin(); vit != otherVars.end(); vit++)
        {
            if ((*vit)->hasNonlinearities())
                throw MyException("this term cannot have nonlinearities");
            
            if ((*vit)->getNumUncertainties()!=0)
                throw MyException("this term cannot have uncertainties");
            
            ROCPPExpr_Ptr expr( new LHSExpression() );
            (*expr) += ROCPPCstrTerm_Ptr(*vit);
            (*expr) *= ROCPPCstrTerm_Ptr(*vit);
            cstr->add_lhs( expr );
        }
        
        cstr->set_rhs( make_pair(0.,true) );
        add_constraint(cstr);
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% DDU Optimization Model %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DDUOptimizationModel::DDUOptimizationModel(uint numTimeStages, uncOptModelObjType objType) :
UncertainOptimizationModel(numTimeStages,objType)
{
//    if(objType != robust)
//    {
//        throw MyException("The type of DDU problem must be robust");
//    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DDUOptimizationModel::dduIterator DDUOptimizationModel::dduBegin() const
{
    return m_dduContainer->begin();
}

DDUOptimizationModel::dduIterator DDUOptimizationModel::dduEnd() const
{
    return m_dduContainer->end();
}

DDUOptimizationModel::nondduIterator DDUOptimizationModel::nondduBegin() const
{
    return m_nondduContainer->begin();
}


DDUOptimizationModel::nondduIterator DDUOptimizationModel::nondduEnd() const
{
    return m_nondduContainer->end();
}

DDUOptimizationModel::dduToMeasMapIterator DDUOptimizationModel::dduToMeasMapBegin() const
{
    return m_dduToMeasMap.begin();
}

DDUOptimizationModel::dduToMeasMapIterator DDUOptimizationModel::dduToMeasMapEnd() const
{
    return m_dduToMeasMap.end();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void DDUOptimizationModel::add_ddu(ROCPPUnc_Ptr pUncertainty, uint firstTimeStageObservable, uint lastTimeStageObservable, const map<uint, double> &obsCosts)
{
    
    ROCPPUnc_Ptr newUnc = pUncertainty->Clone();
    
    if (newUnc->getTimeStage()!=1)
        throw MyException("Decision dependent uncertainty time stage should be 1");
    
    if (firstTimeStageObservable > lastTimeStageObservable)
        throw MyException("your uncertainty can never be observed");
    
    if (lastTimeStageObservable > m_numTimeStages)
        throw MyException("lastTimeStageObservable should be <= m_numTimeStages");
    
    *m_dduContainer += newUnc;
    *m_pUncContainer += newUnc;
    m_dduStagesObs.insert( make_pair( newUnc->getName(), make_pair(firstTimeStageObservable, lastTimeStageObservable) ) );
    
    
    ROCPPVarIF_Ptr prev_var;
    // create measurement variables and add corresponding constraints
    ROCPPExpr_Ptr obj_fun;
    
    double tmpCost;
    
    for (uint t = 1; t < firstTimeStageObservable; t++)
    {
        string varName("m"+ newUnc->getName() + "_" + to_string(t) );
        
        ROCPPVarIF_Ptr var;
        if (t==1){
            var=ROCPPVarIF_Ptr(new VariableBool(varName));
        }
        else{
            var=ROCPPVarIF_Ptr(new AdaptVarBool(varName,t,0.,1.));
        }
        add_var(var);
        
        measPair mp( newUnc, var );
        
        m_measVars.insert( make_pair(var->getName(), mp));
        m_dduToMeasMap.insert( make_pair(make_pair( newUnc->getName(), t ), mp ));
        
        ROCPPClassicConstraint_Ptr pConstraint( new EqConstraint() );
        pConstraint->add_lhs(1. , var );
        pConstraint->set_rhs(make_pair(0.,true));
        add_constraint(pConstraint);
    }
    
    for (uint t=firstTimeStageObservable; t<=min(lastTimeStageObservable, getNumTimeStages()-1); t++)
    {
        string varName("m"+ newUnc->getName() + "_" + to_string(t) );
        
        map<uint, double>::const_iterator oc_it( obsCosts.find(t) );
        if (oc_it==obsCosts.end())
            throw MyException("observation cost not found");
        
        tmpCost = oc_it->second;
        
        ROCPPVarIF_Ptr var;
        if ( t < min(lastTimeStageObservable, getNumTimeStages()-1) ){
            
            map<uint, double>::const_iterator ocn_it( obsCosts.find(t+1) );
            if (ocn_it==obsCosts.end())
                throw MyException("observation cost not found");
            
            if (t==1){
                var=ROCPPVarIF_Ptr(new VariableBool(varName));
                obsCost.insert(make_pair(varName, make_pair(var, tmpCost - ocn_it->second) ) );
            }
            else{
                var=ROCPPVarIF_Ptr(new AdaptVarBool(varName,t,0.,1.));
                obsCost.insert(make_pair(varName, make_pair(var, tmpCost - ocn_it->second) ) );
            }
        }
        else{
            if (t==1)
                var=ROCPPVarIF_Ptr(new VariableBool(varName));
            else
                var=ROCPPVarIF_Ptr(new AdaptVarBool(varName,t,0.,1.));
            
            obsCost.insert(make_pair(varName, make_pair(var,tmpCost) ) );
        }
        add_var(var);
        
        // information observed cannot be forgotten
        if ( (t>firstTimeStageObservable) && (t<=lastTimeStageObservable) )
        {
            ROCPPClassicConstraint_Ptr pConstraint( new IneqConstraint() );
            pConstraint->add_lhs(-1. , var );
            pConstraint->add_lhs(1.,prev_var);
            pConstraint->set_rhs(make_pair(0.,true));
            add_constraint(pConstraint);
        }
        
        measPair mp( newUnc, var );
        
        if ( t<=lastTimeStageObservable)
            m_measVars.insert( make_pair(var->getName(), mp));
        
        m_dduToMeasMap.insert( make_pair(make_pair( newUnc->getName(), t ), mp ));
        prev_var=var;
    }
    
    ROCPPVarIF_Ptr lastVar = getMeasVar(newUnc->getName(), min(lastTimeStageObservable, getNumTimeStages()-1));
    
    for (uint t = min(lastTimeStageObservable, getNumTimeStages()-1) + 1; t <= getNumTimeStages(); t++)
    {
        string varName("m"+ newUnc->getName() + "_" + to_string(t) );
        
        ROCPPVarIF_Ptr var;
        
        if(lastVar->isAdaptive())
            var=ROCPPVarIF_Ptr(new AdaptVarBool(varName,t,0.,1.));
        else
            var=ROCPPVarIF_Ptr(new VariableBool(varName,0.,1.));
        add_var(var);
        
        measPair mp( newUnc, var );
        
        m_measVars.insert( make_pair(var->getName(), mp));
        m_dduToMeasMap.insert( make_pair(make_pair( newUnc->getName(), t ), mp ));
        
        ROCPPClassicConstraint_Ptr pConstraint( new EqConstraint() );
        pConstraint->add_lhs(1. , var );
        pConstraint->add_lhs(-1. , lastVar );
        pConstraint->set_rhs(make_pair(0.,true));
        add_constraint(pConstraint);
    }
}

void DDUOptimizationModel::set_objective(ROCPPObjectiveIF_Ptr pObj)
{
    OptimizationModelIF::set_objective(pObj);
    
    if(obsCost.size() != 0)
    {
        cout << "Note: You are automatically adding the observation costs to the objective function." << endl;
        
        map<string, pair<ROCPPVarIF_Ptr, double> >::const_iterator obs = obsCost.begin();
        for(; obs != obsCost.end(); obs++)
        {
            if(obs->second.second >= 1e-4 || obs->second.second <= -1e-4){
                add_ddu_obj(obs->second.first, obs->second.second);
                cout << obs->first << ": " << obs->second.second << endl;
            }
        }
    }
}


void DDUOptimizationModel::pair_uncertainties(ROCPPUnc_Ptr u1, ROCPPUnc_Ptr u2)
{
    string u1name(u1->getName());
    string u2name(u2->getName());
    
    for (uint t=1; t<m_numTimeStages; t++)
    {
        dduToMeasMapIterator m_it1( find(u1name, t) );
        dduToMeasMapIterator m_it2( find(u2name, t) );
        
        if ( (m_it1 == dduToMeasMapEnd()) && (m_it2 != dduToMeasMapEnd()) )
            throw MyException("To be able to pair two uncertainties, they must be observalient in the same periods");
        
        if ( (m_it2 == dduToMeasMapEnd()) && (m_it1 != dduToMeasMapEnd()) )
            throw MyException("To be able to pair two uncertainties, they must be observalient in the same periods");
        
        if ( (m_it1 != dduToMeasMapEnd()) && (m_it2 != dduToMeasMapEnd()) )
        {
            ROCPPClassicConstraint_Ptr pCstr(new EqConstraint());
            pCstr->add_lhs( 1., (*m_it1).second.m_measVar );
            pCstr->add_lhs( -1., (*m_it2).second.m_measVar );
            pCstr->set_rhs( make_pair( 0.,true));
            add_constraint(pCstr);
        }
        
    }
}

void DDUOptimizationModel::set_ddu(ROCPPOptModelIF_Ptr pIn)
{
    set_ddu(pIn->getDDUToMeasMap(), pIn->getdduStagesObs());
}

void DDUOptimizationModel::set_ddu(const map< pair<string,uint>, measPair > &dduToMeasMap, const map< string, pair<uint,uint> > &dduStagesObs)
{
    m_dduStagesObs.clear();
    
    for(const auto& tmp : dduToMeasMap)
    {
        ROCPPUnc_Ptr unc;
        unc = getUnc(tmp.second.m_ddu->getName() );
        
        ROCPPVarIF_Ptr var;
        var = getVar(tmp.second.m_measVar->getName() );
        
        measPair newPair(unc, var);
        m_measVars.insert(make_pair(tmp.second.m_measVar->getName(), newPair));
        m_dduToMeasMap.insert(make_pair(tmp.first, newPair) );
    }
    
    ROCPPuncContainer_Ptr nonddu = ROCPPuncContainer_Ptr(new uncContainer() );
    
    for(map<string, ROCPPUnc_Ptr >::const_iterator uit = uncertaintiesBegin(); uit != uncertaintiesEnd(); uit++)
    {
        if( dduStagesObs.find(uit->first) != dduStagesObs.end() ){
            *m_dduContainer += uit->second;
            m_dduStagesObs.insert( *(dduStagesObs.find(uit->first) ) );
        }
        else{
            *nonddu += uit->second;
        }
    }
    m_nondduContainer = nonddu;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DDUOptimizationModel::dduToMeasMapIterator DDUOptimizationModel::find(string uncName, uint timeStage) const
{
    dduToMeasMapIterator it = m_dduToMeasMap.find( make_pair(uncName,timeStage) );
    if (it==dduToMeasMapEnd())
        throw MyException("pair (" + uncName + "," + to_string(timeStage) +") not found in m_dduToMeasMap");
    
    return it;
}

DDUOptimizationModel::measVarsIterator DDUOptimizationModel::find(string varName) const
{
    return (m_measVars.find(varName));
}

bool DDUOptimizationModel::isMeasVar(string varName) const
{
    if (varIsDefined(varName))
    {
        map<string,measPair>::const_iterator m2_it(find(varName));
        if (m2_it==m_measVars.end())
            return false;
    }
    else
        throw MyException("this variable does not exist");
    
    return true;
}

bool DDUOptimizationModel::isDDU(string uncName) const
{
    uncContainer::const_iterator ddu_it( m_dduContainer->find( uncName ) );
    
    if (ddu_it==m_dduContainer->end())
        return false;
    
    return true;
}

uint DDUOptimizationModel::getFirstStageObservable(string uncName) const
{
    map<string, pair<uint,uint> >::const_iterator obs_it( m_dduStagesObs.find(uncName) );
    if (obs_it == m_dduStagesObs.end() )
        throw MyException("could not find ddu in m_dduStagesObs");
    
    return (obs_it->second.first);
}

uint DDUOptimizationModel::getLastStageObservable(string uncName) const
{
    map<string, pair<uint,uint> >::const_iterator obs_it( m_dduStagesObs.find(uncName) );
    if (obs_it == m_dduStagesObs.end() )
        throw MyException("could not find ddu in m_dduStagesObs");
    
    return (obs_it->second.second);
}

size_t DDUOptimizationModel::getNumDDUncertainties() const {return m_dduContainer->getNumUncertainties();}

ROCPPVarIF_Ptr DDUOptimizationModel::getMeasVar(string dduncName, uint timeStage) const
{
    dduToMeasMapIterator m_it( find( dduncName,timeStage) );
    
    if (m_it==dduToMeasMapEnd())
        throw MyException("pair (dduncertainty,timeStage) not found");
    
    return (m_it->second.m_measVar);
}

ROCPPUnc_Ptr DDUOptimizationModel::getAssociatedUncertainty(string measVar) const
{
    if (!isMeasVar(measVar))
        throw MyException("this is not a measurement variable");
    
    return ((m_measVars.find(measVar))->second.m_ddu);
}

void DDUOptimizationModel::add_uncertainty( ROCPPUnc_Ptr pUnc)
{
    if (pUnc->getTimeStage() > getNumTimeStages())
        throw MyException("time stage of uncertainty should be no more than number of stages in the model");
    
    if( m_dduContainer->find(pUnc->getName()) == m_dduContainer->end())
        *m_nondduContainer += pUnc;
    
    UncertainOptimizationModel::add_uncertainty(pUnc);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void DDUOptimizationModel::WriteToFile(string folderName, string fileName) const
{
    string filePath = folderName;
    if (folderName!="")
        filePath += "/";
    
    ofstream ofs((filePath+fileName+".rob").c_str() );
    ofs.clear();
    //ofs << scientific;
    //ofs.precision(30);
    ofs.setf( ios::showpos );
    //ofs.showpos(true);
    // write objective
    ofs << "Objective" << endl;
    ofs << "min ";
    
    if (getObjType() == robust)
        ofs << "max ";
    else if (getObjType() == stochastic)
        ofs << "E ";
    else
        throw MyException("Unknown objective function type");
    
    getObj()->WriteToStream(ofs);
    
    ofs << endl;
    ofs << endl;
    
    // write constraints
    
    ofs << "Constraints" << endl;
    uint ccnt(0);
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
    {
        if ( !(*c_it)->definesUncertaintySet() )
        {
            if ( (*c_it)->isClassicConstraint() )
                (*c_it)->WriteToStream(ofs,ccnt++);
        }
    }
    
    ccnt=0;
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
    {
        if ( !(*c_it)->definesUncertaintySet() )
        {
            if ( !(*c_it)->isClassicConstraint() )
                (*c_it)->WriteToStream(ofs,ccnt++);
        }
    }
    
    ofs << endl;
    ofs << "Uncertainty Set:" << endl;
    
    ccnt=0;
    for (constraintIterator c_it = constraintBegin(); c_it != constraintEnd(); c_it++)
    {
        if ( (*c_it)->definesUncertaintySet() )
        {
            (*c_it)->WriteToStream(ofs,ccnt++);
        }
    }
    
    ofs << endl;
    ofs << "Decision Variables:" << endl;
    
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++)
    {
        bool meas;
        meas = (m_measVars.find( (*v_it).second->getName() ) != m_measVars.end() );
        
        if(!meas){
            string adapt((v_it->second->isAdaptive())?("Adaptive"):("Static"));
            string dvtype( (v_it->second->isBooleanVar())?("Boolean"):((v_it->second->isIntegerVar()?("Integer"):("Real-valued"))) );
            ofs << v_it->second->getName() << ": " << dvtype << ", " << adapt << ", " << v_it->second->getTimeStage() << ", "<< "Non-Measurement" << endl;
        }
    }
    for(auto& tmp : m_dduToMeasMap)
    {
        string adapt((tmp.second.m_measVar->isAdaptive())?("Adaptive"):("Static"));
        ofs << tmp.second.m_measVar->getName() << ": " << "Boolean" << ", " << adapt << ", " << tmp.second.m_measVar->getTimeStage() << ", "<< "Measurement" << ", " << tmp.first.first << endl;
    }
    
    ofs << endl;
    ofs << "Bounds:" << endl;
    
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++)
    {
        
        ofs << v_it->second->writeLB() << " <= " << v_it->second->getName() << " <= " << v_it->second->writeUB() << endl;
    }
    
    if (isUncertainOptimizationModel())
    {
        ROCPPUncOptModel_Ptr pModelUnc = static_pointer_cast<UncertainOptimizationModel>(this->Clone());
        
        ofs << endl;
        ofs << "Uncertainties:" << endl;
        
        for (UncertainOptimizationModel::uncertaintiesIterator u_it = m_nondduContainer->begin(); u_it != m_nondduContainer->end(); u_it++)
        {
            string isobs((u_it->second->isObservable())?("Observable"):("Not Observable"));
            ofs << u_it->second->getName() << ": " << isobs << ", " << u_it->second->getTimeStage()<< ", "<< "Non-DDU" << endl;
        }
        for (UncertainOptimizationModel::uncertaintiesIterator u_it = m_dduContainer->begin(); u_it != m_dduContainer->end(); u_it++)
        {
            pair<uint, uint> timeObs = m_dduStagesObs.find(u_it->first)->second;
            string isobs((u_it->second->isObservable())?("Observable"):("Not Observable"));
            ofs << u_it->second->getName() << ": " << isobs << ", " << u_it->second->getTimeStage() << ", "<< "DDU" << ", "<< timeObs.first << ", " << timeObs.second << endl;
        }
    }
    
    ofs.close();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPOptModelIF_Ptr DDUOptimizationModel::Clone() const
{
    ROCPPOptModelIF_Ptr pOut;
    if (isUncertainOptimizationModel())
        pOut = InstanciateModel(getType(),getNumTimeStages(),getObjType());
    else
        pOut = InstanciateModel(getType(),getNumTimeStages(),robust);
    
    for (OptimizationModelIF::constraintIterator cit = constraintBegin(); cit != constraintEnd(); cit++)
        pOut->add_constraint( (*cit)->Clone() );
    
    pOut->set_objective(getObj());
    pOut->set_ddu(getDDUToMeasMap(), getdduStagesObs());
    
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Tool function %%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPOptModelIF_Ptr InstanciateModel( problemType type, uint numTimeStages, uncOptModelObjType objType )
{
    //enum problemType{uncertainType,dduType,simpleuType,uncertainssType,misocpType,bmisocpType};
    if (type==uncertainType)
        return ROCPPOptModelIF_Ptr( new  UncertainOptimizationModel(numTimeStages,objType) );
    else if (type == dduType)
        return ROCPPOptModelIF_Ptr( new  DDUOptimizationModel(numTimeStages,objType) );
    else if (type == simpleuType)
        return ROCPPOptModelIF_Ptr( new  SimpleUncertainOptimizationModel(numTimeStages,objType) );
    else if (type == uncertainssType)
        return ROCPPOptModelIF_Ptr( new  UncertainSingleStageOptimizationModel(objType) );
    else if (type == suncertainssType)
        return ROCPPOptModelIF_Ptr( new  SimpleUncertainSingleStageOptimizationModel(objType) );
    if (type==deterministicType)
        return ROCPPOptModelIF_Ptr( new  DeterministicOptimizationModel() );
    else if (type == misocpType)
        return ROCPPOptModelIF_Ptr( new  MISOCP() );
    else if (type == bmisocpType)
        return ROCPPOptModelIF_Ptr( new  Bilinear_MISOCP() );
    else
        throw MyException("unknown problem type");
    
}

bool approximatelyEqual(double a, double b, double epsilon)
{
    return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}

