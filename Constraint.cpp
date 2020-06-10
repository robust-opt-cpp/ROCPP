//
//  Constraint.cpp
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
#include <fstream>

bool DoublesAreEssentiallyEqual(double A, double B, double epsilon);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% CONSTRAINT INTERFACE %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ConstraintIF::ConstraintIF(bool definesUncertaintySet, bool isNAC) :
m_definesUncertaintySet(definesUncertaintySet),
m_isNAC(isNAC){}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ConstraintIF::uncertaintiesIterator ConstraintIF::uncertaintiesBegin() const
{
    throw MyException("not applicable to this type of constraint");
}

ConstraintIF::uncertaintiesIterator ConstraintIF::uncertaintiesEnd() const
{
    throw MyException("not applicable to this type of constraint");
}

ConstraintIF::const_iterator ConstraintIF::begin() const
{
    throw MyException("not applicable to this type of constraint");
}

ConstraintIF::const_iterator ConstraintIF::end() const
{
    throw MyException("not applicable to this type of constraint");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ConstraintIF::add_lhs(double c)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(double c, boost::shared_ptr<DecisionVariableIF> pVariable)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(double c, boost::shared_ptr<UncertaintyIF> pUncertainty,  boost::shared_ptr<DecisionVariableIF> pVariable)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(double c, boost::shared_ptr<UncertaintyIF> pUncertainty)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(double c, boost::shared_ptr<DecisionVariableIF> pVariable1, boost::shared_ptr<DecisionVariableIF> pVariable2)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(double c, boost::shared_ptr<UncertaintyIF> pUncertainty, boost::shared_ptr<DecisionVariableIF> pVariable1, boost::shared_ptr<DecisionVariableIF> pVariable2)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(boost::shared_ptr<const ConstraintTermIF> term)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(double c, boost::shared_ptr<const ConstraintTermIF> term)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(boost::shared_ptr<const LHSExpression> pExpression)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(double c, boost::shared_ptr<const LHSExpression> pExpression)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::add_lhs(double c, boost::shared_ptr<const LHSExpression> pExpression,  boost::shared_ptr<DecisionVariableIF> pVariable)
{
    throw MyException("not applicable to this type of constraint");
}

void ConstraintIF::set_rhs(pair<double,bool> rhs)
{
    throw MyException("not applicable to this type of constraint");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% CLASSIC CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ClassicConstraintIF::ClassicConstraintIF(bool definesUncertaintySet, bool isNAC) :
ConstraintIF(definesUncertaintySet, isNAC),
m_pLHS(boost::shared_ptr<LHSExpression>( new LHSExpression() ) ),
m_rhsParams()
{}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ClassicConstraintIF::const_iterator ClassicConstraintIF::begin() const
{
    return m_pLHS->begin();
}

ClassicConstraintIF::const_iterator ClassicConstraintIF::end() const
{
    return m_pLHS->end();
}

ConstraintIF::varsIterator ClassicConstraintIF::varsBegin() const {return m_pLHS->varsBegin();}

ConstraintIF::varsIterator ClassicConstraintIF::varsEnd() const {return m_pLHS->varsEnd();}

ConstraintIF::uncertaintiesIterator ClassicConstraintIF::uncertaintiesBegin() const {return m_pLHS->uncBegin();}

ConstraintIF::uncertaintiesIterator ClassicConstraintIF::uncertaintiesEnd() const {return m_pLHS->uncEnd();}

void ClassicConstraintIF::add_lhs(double c)
{
    boost::shared_ptr<ConstraintTermIF> toAdd ( new ProductTerm(c) );
    add_lhs( toAdd );
}

void ClassicConstraintIF::add_lhs(double c, boost::shared_ptr<DecisionVariableIF> pVariable)
{
    boost::shared_ptr<ConstraintTermIF> toAdd ( new ProductTerm(c,pVariable) );
    add_lhs( toAdd );
}

void ClassicConstraintIF::add_lhs(double c, boost::shared_ptr<UncertaintyIF> pUncertainty,  boost::shared_ptr<DecisionVariableIF> pVariable)
{
    m_pLHS->add(c,pUncertainty,pVariable);
}

void ClassicConstraintIF::add_lhs(double c, boost::shared_ptr<UncertaintyIF> pUncertainty)
{
    m_pLHS->add(c,pUncertainty);
}

void ClassicConstraintIF::add_lhs(double c, boost::shared_ptr<DecisionVariableIF> pVariable1, boost::shared_ptr<DecisionVariableIF> pVariable2)
{
    m_pLHS->add(c,pVariable1,pVariable2);
}

void ClassicConstraintIF::add_lhs(double c, boost::shared_ptr<UncertaintyIF> pUncertainty, boost::shared_ptr<DecisionVariableIF> pVariable1, boost::shared_ptr<DecisionVariableIF> pVariable2)
{
    m_pLHS->add(c,pUncertainty,pVariable1,pVariable2);
}

void ClassicConstraintIF::add_lhs(boost::shared_ptr<const ConstraintTermIF> term)
{
    m_pLHS->add( term->Clone() );
}

void ClassicConstraintIF::add_lhs(double c, boost::shared_ptr<const LHSExpression> pExpression)
{
    boost::shared_ptr<LHSExpression> scaled_expr( pExpression->Clone() );
    *scaled_expr *= c;
    add_lhs(scaled_expr);
}

void ClassicConstraintIF::add_lhs(double c, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<ConstraintTermIF> scaled_term( term->Clone() );
    *scaled_term *= c;
    add_lhs(scaled_term);
}
void ClassicConstraintIF::add_lhs(boost::shared_ptr<const LHSExpression> pExpression)
{
    for (LHSExpression::const_iterator it = pExpression->begin(); it != pExpression->end(); it++)
        add_lhs((*it)->Clone());
}

void ClassicConstraintIF::add_lhs(double c, boost::shared_ptr<const LHSExpression> pExpression,  boost::shared_ptr<DecisionVariableIF> pVariable)
{
    boost::shared_ptr<LHSExpression> expr( pExpression->Clone() );
    *expr *= c;
    *expr *= pVariable;
    
    add_lhs(expr);
}

void ClassicConstraintIF::set_rhs(pair<double,bool> rhs)
{
    //if ( (rhs.second) && ( (rhs.first<-10E-4) || (rhs.first>10E-4) ) )
        //throw MyException("you are effectively setting the right hand side to zero");
    m_rhsParams.m_rhs=rhs.first; m_rhsParams.m_rhsSet = true; m_rhsParams.m_rhsIsZero=false;
    return;
}

boost::shared_ptr<ConstraintIF> ClassicConstraintIF::mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const
{
    boost::shared_ptr<ClassicConstraintIF> out;
    if (this->isEqConstraint())
        out=boost::shared_ptr<ClassicConstraintIF>(new EqConstraint(this->definesUncertaintySet(),this->isNAC()));
    else
        out=boost::shared_ptr<ClassicConstraintIF>(new IneqConstraint(this->definesUncertaintySet(),this->isNAC()));
    
    
    out->set_rhs( get_rhs() );
    
    out->add_lhs( m_pLHS->mapVars(mapFromVarToExpression) );
    
    return out;
}

boost::shared_ptr<ConstraintIF> ClassicConstraintIF::mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const
{
    boost::shared_ptr<ClassicConstraintIF> out;
    if (this->isEqConstraint())
        out=boost::shared_ptr<ClassicConstraintIF>(new EqConstraint(this->definesUncertaintySet(),this->isNAC()));
    else
        out=boost::shared_ptr<ClassicConstraintIF>(new IneqConstraint(this->definesUncertaintySet(),this->isNAC()));
    
    out->set_rhs( get_rhs() );
    
    out->add_lhs( m_pLHS->mapUncs(mapFromUncToExpression) );
    
    return out;
}

boost::shared_ptr<ConstraintIF> ClassicConstraintIF::replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const
{
    if (term.size()==2) // shortcut for the case when the term has 2 variables
    {
        map< pair<string,string>, uint> freqMap;
        map< pair<string,string>, multimap<string, boost::shared_ptr<DecisionVariableIF> > > termMap;
        getAllProductsOf2Variables(freqMap, termMap);
        
        bool tmp1(freqMap.find(make_pair(term.begin()->first,term.rbegin()->first)) == freqMap.end());
        bool tmp2(freqMap.find(make_pair(term.begin()->first,term.rbegin()->first)) == freqMap.end());
        
        if ( tmp1 && tmp2 )
            return this->Clone();
    }
    
    boost::shared_ptr<ClassicConstraintIF> out;
    if (this->isEqConstraint())
        out=boost::shared_ptr<ClassicConstraintIF>(new EqConstraint(this->definesUncertaintySet(),this->isNAC()));
    else
        out=boost::shared_ptr<ClassicConstraintIF>(new IneqConstraint(this->definesUncertaintySet(),this->isNAC()));
    
    out->set_rhs( get_rhs() );
    
    out->add_lhs( m_pLHS->replaceTermWithVar(term,var) );
    
    return out;
}

boost::shared_ptr<ConstraintIF> ClassicConstraintIF::replaceBilinearTerm(map<pair<string,string>, boost::shared_ptr<DecisionVariableIF> > &allTerm, uint &count) const
{
    boost::shared_ptr<LHSExpression> newLhs;
    
    if(isEqConstraint()){
        boost::shared_ptr<ConstraintIF> newCstr(new EqConstraint(definesUncertaintySet(), isNAC()));
        newLhs = m_pLHS->replaceBilinearTerm(allTerm, count);
        newCstr->add_lhs(newLhs);
        newCstr->set_rhs(get_rhs());
        return newCstr;
    }
    else{
        boost::shared_ptr<ConstraintIF> newCstr(new IneqConstraint(definesUncertaintySet(), isNAC()));
        newLhs = m_pLHS->replaceBilinearTerm(allTerm, count);
        newCstr->add_lhs(newLhs);
        newCstr->set_rhs(get_rhs());
        newCstr->isDeterministic();
        return newCstr;
    }
}

boost::shared_ptr<ConstraintIF> ClassicConstraintIF::mapVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const
{
    boost::shared_ptr<ClassicConstraintIF> out;
    if (this->isEqConstraint())
        out=boost::shared_ptr<ClassicConstraintIF>(new EqConstraint(this->definesUncertaintySet(),this->isNAC()));
    else
        out=boost::shared_ptr<ClassicConstraintIF>(new IneqConstraint(this->definesUncertaintySet(),this->isNAC()));
    
    // iterate through the lhs
    
    boost::shared_ptr<LHSExpression> expOut =  m_pLHS->mapExprVars(mapFromOldToNewVars);
    out->add_lhs( expOut );
    
    // set right hand side
    out->set_rhs( this->get_rhs() );
    return out;
}

boost::shared_ptr<ConstraintIF> ClassicConstraintIF::mapUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const
{
    boost::shared_ptr<ClassicConstraintIF> out;
    if (this->isEqConstraint())
        out=boost::shared_ptr<ClassicConstraintIF>(new EqConstraint(this->definesUncertaintySet(),this->isNAC()));
    else
        out=boost::shared_ptr<ClassicConstraintIF>(new IneqConstraint(this->definesUncertaintySet(),this->isNAC()));
    
    // iterate through the lhs
    
    boost::shared_ptr<LHSExpression> expOut =  m_pLHS->mapExprUnc(mapFromOldToNewUnc);
    out->add_lhs( expOut );
    
    // set right hand side
    out->set_rhs( this->get_rhs() );
    return out;
}

void ClassicConstraintIF::add_vars_involved_in_prod(dvContainer &dvs) const
{
    for (const_iterator it = begin(); it!=end(); it++)
        (*it)->add_vars_involved_in_prod(dvs);
}

void ClassicConstraintIF::add_int_vars(dvContainer &dvs) const
{
    m_pLHS->getDVContainer()->add_int_vars(dvs);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

uint ClassicConstraintIF::getNumTimesTermAppears(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term) const
{
    return (m_pLHS->getNumTimesTermAppears(term));
}

void ClassicConstraintIF::getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, boost::shared_ptr<DecisionVariableIF> > > &termMap) const
{
    m_pLHS->getAllProductsOf2Variables(freqMap,termMap);
}

boost::shared_ptr<const dvContainer> ClassicConstraintIF::getDVContainer() const
{
    return getLHS()->getDVContainer();
}

boost::shared_ptr<const uncContainer> ClassicConstraintIF::getUncContainer() const
{
    return getLHS()->getUncContainer();
}

uint ClassicConstraintIF::getNumContVars() const {return m_pLHS->getNumContVars();}
uint ClassicConstraintIF::getNumIntVars() const {return m_pLHS->getNumIntVars();}
uint ClassicConstraintIF::getNumBoolVars() const {return m_pLHS->getNumBoolVars();}
uint ClassicConstraintIF::getNumAdaptiveContVars() const {return m_pLHS->getNumAdaptiveContVars();}
uint ClassicConstraintIF::getNumAdaptiveVars() const {return m_pLHS->getNumAdaptiveVars();}

uint ClassicConstraintIF::getTimeStage() const {return m_pLHS->getTimeStage();}
size_t ClassicConstraintIF::getNumUncertainties() const {return m_pLHS->getNumUncertainties();}

bool ClassicConstraintIF::hasNonlinearities() const {return m_pLHS->hasNonlinearities();};
bool ClassicConstraintIF::hasProdsUncertainties() const {return m_pLHS->hasProdsUncertainties();}
bool ClassicConstraintIF::hasProdsContVars() const {return m_pLHS->hasProdsContVars();}

bool ClassicConstraintIF::isWellDefined() const {return ( (m_pLHS->isWellDefined()) && m_rhsParams.m_rhsSet );}

bool ClassicConstraintIF::hasNoDVs() const {return (m_pLHS->getNumVars() == 0);}

bool ClassicConstraintIF::AnyVarIsInvolved(dvContainer& dvs) const {return m_pLHS->AnyVarIsInvolved(dvs);}

bool ClassicConstraintIF::hasNormTerm() const {return (m_pLHS->hasNormTerm());}

bool ClassicConstraintIF::isLinear() const {return m_pLHS->isLinear();}

bool ClassicConstraintIF::isQuadratic() const {return m_pLHS->isQuadratic();}

pair<double,bool> ClassicConstraintIF::get_rhs() const
{
    if (m_rhsParams.m_rhsSet)
        return make_pair(m_rhsParams.m_rhs,m_rhsParams.m_rhsIsZero);
    else
    {
        throw MyException("rhs not set");
        return make_pair(0.,false);
    }
}

boost::shared_ptr<LHSExpression> ClassicConstraintIF::getLHS() const {return boost::shared_ptr<LHSExpression>( m_pLHS ) ;}

boost::shared_ptr<LHSExpression> ClassicConstraintIF::getLinearPart() const
{
    boost::shared_ptr<LHSExpression> out( new LHSExpression() );
    (*out) += m_pLHS->getLinearPart();
    if(! get_rhs().second)
        (*out) += -1.*(get_rhs().first);
    return out;
}

boost::shared_ptr<NormTerm> ClassicConstraintIF::getNormTerm() const {return m_pLHS->getNormTerm();}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<ConstraintIF> ClassicConstraintIF::Clone() const
{
    
    boost::shared_ptr<ClassicConstraintIF> out;
    if (this->isEqConstraint())
        out=boost::shared_ptr<ClassicConstraintIF>(new EqConstraint(this->definesUncertaintySet(),this->isNAC()));
    else
        out=boost::shared_ptr<ClassicConstraintIF>(new IneqConstraint(this->definesUncertaintySet(),this->isNAC()));
    
    // iterate through the lhs
    
    boost::shared_ptr<LHSExpression> expOut(m_pLHS->Clone());
    out->add_lhs( expOut );
    
    // set right hand side
    out->set_rhs( this->get_rhs() );
    return out;
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ClassicConstraintIF::WriteToStream(ofstream &ofs, uint ccnt) const
{
    if (!isWellDefined())
        throw MyException("constraint is not well defined");
    
    ofs << "c" << ccnt << ":";
    
    
    // lhs
    m_pLHS->WriteToStream(ofs);
    
    // rhs
    if ( isEqConstraint() )
        ofs << " == ";
    else
        ofs << " <= ";
    
    ofs << get_rhs().first;
    
    ofs << endl;
    
    ofs << endl;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% EQUAL CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool EqConstraint::isUseful(double epsilon) const
{
    
    if ( !(getLHS()->isConstant()) )
        return true;
    else
    {
        double rhs( get_rhs().first );
        
        double lhs_const( getLHS()->getSumConstantTerms() );
        
        if ( DoublesAreEssentiallyEqual(rhs, lhs_const, epsilon) )
            return false;
        else
            return true;
    }
    
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% SOS CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SOSConstraint::SOSConstraint(uint SOSType) : ConstraintIF(false, false), m_SOSType(SOSType), m_pDVContainer ( new dvContainer() )
{
    if ( (SOSType != 1) && (SOSType != 2) )
        throw MyException("unkown sos type");
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SOSConstraint::varsIterator SOSConstraint::varsBegin() const
{
    return m_pDVContainer->begin();
}

SOSConstraint::varsIterator SOSConstraint::varsEnd() const
{
    return m_pDVContainer->end();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void SOSConstraint::add(boost::shared_ptr<DecisionVariableIF> dv, double weight)
{
    if ( (dv->isAdaptive()) &&  (!dv->isBooleanVar()) && (!dv->isIntegerVar()) )
        throw MyException("cannot have adaptive real-valued variable in SOS set");
    
    pair<map<string, pair<boost::shared_ptr<DecisionVariableIF>, double> >::iterator,bool> it( m_sosMap.insert(make_pair( dv->getName(), make_pair(dv,weight) ) ) );
    
    if (!it.second)
        throw MyException("variable was already present in the set");
    
    *m_pDVContainer += dv;
}

void SOSConstraint::add_int_vars(dvContainer &dvs) const
{
    m_pDVContainer->add_int_vars(dvs);
}

boost::shared_ptr<ConstraintIF> SOSConstraint::mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const
{
    for (map<string, boost::shared_ptr<LHSExpression> >::const_iterator it = mapFromVarToExpression.begin(); it != mapFromVarToExpression.end(); it++)
    {
        dvContainer::const_iterator vit ( m_pDVContainer->find( it->first ) );
        if (vit != m_pDVContainer->end())
            throw MyException("cannot map variable to expression in SOSConstraint: consider modelling as classical constraint");
    }
    
    return this->Clone();
}

boost::shared_ptr<ConstraintIF> SOSConstraint::replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const
{
    if (term.size()>1)
        return this->Clone();
    
    if (term.size()==0)
        throw MyException("something wrong in SOSConstraint::getNumTimesTermAppears");
    
    
    if ( (m_pDVContainer->find(term.begin()->first)) == m_pDVContainer->end() )
        return this->Clone();
    
    map<string,boost::shared_ptr<DecisionVariableIF> > tmp;
    tmp.insert(make_pair(term.begin()->first, var) );
    
    return mapVars(tmp);
}

boost::shared_ptr<ConstraintIF> SOSConstraint::replaceBilinearTerm(map<pair<string,string>, boost::shared_ptr<DecisionVariableIF> > &allTerm, uint &count) const
{
    return this->Clone();
}

boost::shared_ptr<ConstraintIF> SOSConstraint::mapVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const
{
    boost::shared_ptr<SOSConstraint> out ( new SOSConstraint(this->getSOSType()) );
    
    for (SOSMapType::const_iterator it = m_sosMap.begin(); it != m_sosMap.end(); it++)
    {
        // try to find the current variable in the map
        map<string,boost::shared_ptr<DecisionVariableIF> >::const_iterator tsl_it ( mapFromOldToNewVars.find( it->second.first->getName() ) );
        
        if (tsl_it == mapFromOldToNewVars.end())
        {
            out->add( it->second.first, it->second.second );
        }
        else
        {
            out->add( tsl_it->second, it->second.second );
        }
    }
    
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<const dvContainer> SOSConstraint::getDVContainer() const
{
    return m_pDVContainer;
}

boost::shared_ptr<const uncContainer> SOSConstraint::getUncContainer() const
{
    return boost::shared_ptr<const uncContainer>(new uncContainer());
}

bool SOSConstraint::hasNoDVs() const
{
    return m_pDVContainer->getNumVars()==0;
}

bool SOSConstraint::isWellDefined() const
{
    return m_sosMap.size()>1;
}

uint SOSConstraint::getNumContVars() const
{
    return m_pDVContainer->getNumContVars();
}

uint SOSConstraint::getNumIntVars() const
{
    return m_pDVContainer->getNumIntVars();
}

uint SOSConstraint::getNumBoolVars() const
{
    return m_pDVContainer->getNumBoolVars();
}

uint SOSConstraint::getNumAdaptiveContVars() const
{
    return m_pDVContainer->getNumAdaptiveContVars();
}

uint SOSConstraint::getNumAdaptiveVars() const
{
    return m_pDVContainer->getNumAdaptiveVars();
}

size_t SOSConstraint::getNumVars() const
{
    return m_pDVContainer->getNumVars();
}

uint SOSConstraint::getTimeStage() const
{
    return m_pDVContainer->getTimeStage();
}

size_t SOSConstraint::getNumUncertainties() const
{
    return 0;
}

bool SOSConstraint::hasNonlinearities() const
{
    return false;
}

bool SOSConstraint::hasProdsUncertainties() const
{
    return false;
}

bool SOSConstraint::hasProdsContVars() const
{
    return false;
}

bool SOSConstraint::AnyVarIsInvolved(dvContainer& dvs) const
{
    return m_pDVContainer->AnyVarIsInvolved(dvs);
}

uint SOSConstraint::getNumTimesTermAppears(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term) const
{
    if (term.size()>1)
        return 0;
    
    if (term.size()==0)
        throw MyException("something wrong in SOSConstraint::getNumTimesTermAppears");
    
    
    if ( (m_pDVContainer->find(term.begin()->first)) == m_pDVContainer->end() )
        return 0;
    
    
    throw MyException("probably shouldn't be here?");
    return 1;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<ConstraintIF> SOSConstraint::Clone() const
{
    boost::shared_ptr<SOSConstraint> pOut ( new SOSConstraint( getSOSType() ) );
    
    for (SOSMapType::const_iterator it = m_sosMap.begin(); it != m_sosMap.end(); it++)
        pOut->add(it->second.first, it->second.second);
    
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void SOSConstraint::WriteToStream(ofstream &ofs, uint cnt) const
{
    if (!isWellDefined())
        throw MyException("constraint is not well defined");
    
    ofs << "s" << cnt << ": S" << m_SOSType << ":: ";
    
    for ( SOSMapType::const_iterator mit = m_sosMap.begin(); mit != m_sosMap.end(); mit++)
    {
        ofs << mit->second.first->getName() << " : " << mit->second.second << " ";
    }
    
    ofs << endl;
    
    ofs << endl;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% IF THEN CONSTRAINT %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IfThenConstraint::IfThenConstraint(boost::shared_ptr<ConstraintIF> lhs, boost::shared_ptr<ConstraintIF> rhs) : ConstraintIF(false, false), m_lhs(lhs), m_rhs(rhs), m_pDVContainer(new dvContainer()), m_pUncContainer(new uncContainer())
{
    if ( (!lhs->isClassicConstraint()) || (!rhs->isClassicConstraint()) )
        throw MyException("can only have classic constraints in if then constraint arguments");
    
    *m_pDVContainer+=*lhs->getDVContainer();
    *m_pDVContainer+=*rhs->getDVContainer();
    
    *m_pUncContainer+=*lhs->getUncContainer();
    *m_pUncContainer+=*rhs->getUncContainer();
    
    for(const_iterator tit = lhs->begin(); tit != lhs->end(); tit++)
        m_terms.push_back((*tit));
    
    for(const_iterator tit = rhs->begin(); tit != rhs->end(); tit++)
        m_terms.push_back((*tit));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

IfThenConstraint::varsIterator IfThenConstraint::varsBegin() const {return m_pDVContainer->begin();}

IfThenConstraint::varsIterator IfThenConstraint::varsEnd() const {return m_pDVContainer->end();}

IfThenConstraint::const_iterator IfThenConstraint::begin() const {return m_terms.begin();}
IfThenConstraint::const_iterator IfThenConstraint::end() const {return m_terms.end();}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<ConstraintIF> IfThenConstraint::mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const
{
    boost::shared_ptr<ConstraintIF> mapped_lhs ( m_lhs->mapVars(mapFromVarToExpression) );
    boost::shared_ptr<ConstraintIF> mapped_rhs ( m_rhs->mapVars(mapFromVarToExpression) );
    
    boost::shared_ptr<ConstraintIF> pOut ( new IfThenConstraint(mapped_lhs, mapped_rhs) );
    
    return pOut;
}

boost::shared_ptr<ConstraintIF> IfThenConstraint::mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const
{
    boost::shared_ptr<ConstraintIF> mapped_lhs ( m_lhs->mapUncs(mapFromUncToExpression) );
    boost::shared_ptr<ConstraintIF> mapped_rhs ( m_rhs->mapUncs(mapFromUncToExpression) );
    
    boost::shared_ptr<ConstraintIF> pOut ( new IfThenConstraint(mapped_lhs, mapped_rhs) );
    
    return pOut;
}

boost::shared_ptr<ConstraintIF> IfThenConstraint::replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const
{
    boost::shared_ptr<ConstraintIF> mapped_lhs ( m_lhs->replaceTermWithVar(term,var) );
    boost::shared_ptr<ConstraintIF> mapped_rhs ( m_rhs->replaceTermWithVar(term,var) );
    boost::shared_ptr<ConstraintIF> pOut ( new IfThenConstraint(mapped_lhs, mapped_rhs) );
    
    return pOut;
}

boost::shared_ptr<ConstraintIF> IfThenConstraint::replaceBilinearTerm(map<pair<string,string>, boost::shared_ptr<DecisionVariableIF> > &allTerm, uint &count) const
{
    boost::shared_ptr<ConstraintIF> newCstr(new IfThenConstraint(m_lhs->replaceBilinearTerm(allTerm, count),m_rhs->replaceBilinearTerm(allTerm, count)));
    
    return newCstr;
}

boost::shared_ptr<ConstraintIF> IfThenConstraint::mapVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const
{
    boost::shared_ptr<ConstraintIF> mapped_lhs ( m_lhs->mapVars(mapFromOldToNewVars) );
    boost::shared_ptr<ConstraintIF> mapped_rhs ( m_rhs->mapVars(mapFromOldToNewVars) );
    
    boost::shared_ptr<ConstraintIF> pOut ( new IfThenConstraint(mapped_lhs, mapped_rhs) );
    
    return pOut;
}

boost::shared_ptr<ConstraintIF> IfThenConstraint::mapUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const
{
    boost::shared_ptr<ConstraintIF> mapped_lhs ( m_lhs->mapUnc(mapFromOldToNewUnc) );
    boost::shared_ptr<ConstraintIF> mapped_rhs ( m_rhs->mapUnc(mapFromOldToNewUnc) );
    
    boost::shared_ptr<ConstraintIF> pOut ( new IfThenConstraint(mapped_lhs, mapped_rhs) );
    
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<const dvContainer> IfThenConstraint::getDVContainer() const {return
    m_pDVContainer;}

boost::shared_ptr<const uncContainer> IfThenConstraint::getUncContainer() const {return m_pUncContainer;}

uint IfThenConstraint::getNumContVars() const {return m_pDVContainer->getNumContVars();}
uint IfThenConstraint::getNumIntVars() const {return m_pDVContainer->getNumIntVars();}
uint IfThenConstraint::getNumBoolVars() const {return m_pDVContainer->getNumBoolVars();}
uint IfThenConstraint::getNumAdaptiveContVars() const {return m_pDVContainer->getNumAdaptiveContVars();}
uint IfThenConstraint::getNumAdaptiveVars() const {return m_pDVContainer->getNumAdaptiveVars();}
size_t IfThenConstraint::getNumVars() const {return m_pDVContainer->getNumVars();}

bool IfThenConstraint::hasNoDVs() const {return m_pDVContainer->getNumVars()==0;}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<ConstraintIF> IfThenConstraint::Clone() const
{
    boost::shared_ptr<ConstraintIF> pOut ( new IfThenConstraint(m_lhs->Clone(), m_rhs->Clone()) );
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void IfThenConstraint::WriteToStream(ofstream &ofs, uint ccnt) const
{
    
    if (!isWellDefined())
        throw MyException("constraint is not well defined");
    
    
    // lhs
    ofs << "( ";
    m_lhs->WriteToStream(ofs,1);
    ofs << " )    ->    ( ";
    
    
    // rhs
    m_rhs->WriteToStream(ofs,2);
    ofs << " )";
    
    ofs << endl;
    
    ofs << endl;
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% TOOL FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool DoublesAreEssentiallyEqual(double A, double B, double epsilon)
{
    double diff = A - B;
    return (diff < epsilon) && (-diff < epsilon);
}

boost::shared_ptr<ConstraintIF> createConstraint(boost::shared_ptr<LHSExpression> lhs, double rhs, bool isEqual, bool definesUncertaintySet, bool isNAC)
{
    bool isZero = (DoublesAreEssentiallyEqual(rhs, 0., 10e-4));
    if(isEqual)
    {
        boost::shared_ptr<ConstraintIF> newCstr(new EqConstraint(definesUncertaintySet, isNAC));
        newCstr->add_lhs(lhs);
        newCstr->set_rhs(make_pair(rhs, isZero));
        return newCstr;
    }
    else
    {
        boost::shared_ptr<ConstraintIF> newCstr(new IneqConstraint(definesUncertaintySet, isNAC));
        newCstr->add_lhs(lhs);
        newCstr->set_rhs(make_pair(rhs, isZero));
            //newCstr->set_rhs(make_pair(rhs, false));
        return newCstr;
    }
}
