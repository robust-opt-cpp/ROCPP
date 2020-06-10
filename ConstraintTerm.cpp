//
//  ConstraintTerm.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "IncludeFiles.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include <fstream>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% CONSTRAINT TERM INTERFACE %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ConstraintTermIF::ConstraintTermIF() : m_pDVContainer ( new dvContainer() ), m_pUncContainer( new uncContainer() ) {}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Operators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool ConstraintTermIF::operator==(const ProductTerm &other) const {throw MyException("operation not applicable");};
void ConstraintTermIF::operator*=(boost::shared_ptr<const ConstraintTermIF> term) {throw MyException("operation not applicable");};
void ConstraintTermIF::operator*=(boost::shared_ptr<DecisionVariableIF> var){throw MyException("operation not applicable");};
void ConstraintTermIF::operator*=(boost::shared_ptr<UncertaintyIF> unc){throw MyException("operation not applicable");};
void ConstraintTermIF::operator*=(double a){throw MyException("operation not applicable");}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pair<bool,boost::shared_ptr<ConstraintTermIF> > ConstraintTermIF::factorOut(boost::shared_ptr<UncertaintyIF> unc) const
{
    throw MyException("no factor");
}

void ConstraintTermIF::add(boost::shared_ptr<const ConstraintTermIF> other) {throw MyException("should not be here");}

void ConstraintTermIF::add_int_vars(dvContainer &dvs) const
{
    m_pDVContainer->add_int_vars(dvs);
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool ConstraintTermIF::isNonlinearProdTerm() const {throw MyException("this is not a product term");}
bool ConstraintTermIF::isConstant() const {throw MyException("only useable for product terms");}
bool ConstraintTermIF::isDeterministic() const {throw MyException("only useable for product terms");}
bool ConstraintTermIF::isLinear() const {throw MyException("only useable for product terms");}
bool ConstraintTermIF::isQuadratic() const {throw MyException("only useable for product terms");}

double ConstraintTermIF::getCoeff() const {throw MyException("no coeff");}

uint ConstraintTermIF::getNumContVars() const {return m_pDVContainer->getNumContVars();}
uint ConstraintTermIF::getNumIntVars() const {return m_pDVContainer->getNumIntVars();}
uint ConstraintTermIF::getNumBoolVars() const {return m_pDVContainer->getNumBoolVars();}
uint ConstraintTermIF::getNumAdaptiveContVars() const {return m_pDVContainer->getNumAdaptiveContVars();}
uint ConstraintTermIF::getNumAdaptiveVars() const {return m_pDVContainer->getNumAdaptiveVars();}
size_t ConstraintTermIF::getNumUncertainties() const {return m_pUncContainer->getNumUncertainties();}
size_t ConstraintTermIF::getNumVars() const {return m_pDVContainer->getNumVars();}

bool ConstraintTermIF::allIntVarsBounded () const
{
    bool out(true);
    
    if (getNumIntVars()==0)
        return out;
    
    for (dvMapType::const_iterator it = m_pDVContainer->begin(); it != m_pDVContainer->end(); it++)
    {
        if ( it->second->isIntegerVar() )
        {
            if ( (!it->second->getLB()) || (!it->second->getUB()) )
            {
                out = false;
                break;
            }
        }
    }
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% PRODUCT TERM %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ProductTerm::ProductTerm(double c) : m_coeff(c){}

ProductTerm::ProductTerm(double c, boost::shared_ptr<DecisionVariableIF> pVariable) : m_coeff(c)
{
    addVariable(pVariable);
}

ProductTerm::ProductTerm(double c, boost::shared_ptr<UncertaintyIF> pUncertainty,  boost::shared_ptr<DecisionVariableIF> pVariable) : m_coeff(c)
{
    addUncertainty(pUncertainty);
    addVariable(pVariable);
}

ProductTerm::ProductTerm(double c, boost::shared_ptr<UncertaintyIF> pUncertainty) : m_coeff(c)
{
    addUncertainty(pUncertainty);
}

ProductTerm::ProductTerm(double c, boost::shared_ptr<DecisionVariableIF> pVariable1, boost::shared_ptr<DecisionVariableIF> pVariable2) : m_coeff(c)
{
    addVariable(pVariable1);
    addVariable(pVariable2);
}

ProductTerm::ProductTerm(double c, boost::shared_ptr<UncertaintyIF> pUncertainty, boost::shared_ptr<DecisionVariableIF> pVariable1, boost::shared_ptr<DecisionVariableIF> pVariable2) : m_coeff(c)
{
    addUncertainty(pUncertainty);
    addVariable(pVariable1);
    addVariable(pVariable2);
}

ProductTerm::ProductTerm(double c, const vector<boost::shared_ptr<UncertaintyIF> >& uncVec, const vector<boost::shared_ptr<DecisionVariableIF> >& varVec) : m_coeff(c)
{
    for (vector<boost::shared_ptr<UncertaintyIF> >::const_iterator uit = uncVec.begin(); uit != uncVec.end(); uit++)
        addUncertainty(*uit);
    
    for (vector<boost::shared_ptr<DecisionVariableIF> >::const_iterator vit = varVec.begin(); vit != varVec.end(); vit++)
        addVariable(*vit);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Operators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool ProductTerm::operator==(const ProductTerm &other) const
{
    return ( ( m_DVMap == other.m_DVMap ) && (m_UncMap == other.m_UncMap) );
}

void ProductTerm::operator*=(boost::shared_ptr<const ConstraintTermIF> term)
{
    if (term->getType() != prodTerm)
        throw MyException("can only multiply with product type");
    
    boost::shared_ptr<const ProductTerm> pTerm = boost::static_pointer_cast<const ProductTerm>(term->Clone());
    
    for ( varsIterator v_it = pTerm->varsBegin(); v_it!= pTerm->varsEnd(); v_it++ )
    {
        this->addVariable( (v_it)->second );
    }
    
    for ( uncIterator u_it = pTerm->uncBegin(); u_it != pTerm->uncEnd(); u_it++)
    {
        this->addUncertainty( (u_it)->second );
    }
    
    (*this) *= pTerm->getCoeff();
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pair<bool,boost::shared_ptr<ConstraintTermIF> > ProductTerm::factorOut(boost::shared_ptr<UncertaintyIF> unc) const
{
    uncIterator uit( m_UncMap.find(unc->getName()) );
    
    if (uit==m_UncMap.end())
        return make_pair(false,boost::shared_ptr<ConstraintTermIF>(new ProductTerm(0.)));
    
    if ( (uit!=m_UncMap.end()) && (uit->second!=unc) )
        throw MyException("uncertainty with same name found");
    
    boost::shared_ptr<ProductTerm> out (new ProductTerm(m_coeff));
    
    for (varsIterator vit = varsBegin(); vit != varsEnd(); vit++)
        (*out) *= vit->second;
    
    for (uncIterator uit = uncBegin(); uit != uncEnd(); uit++)
    {
        if (uit->second->getName() != unc->getName())
        {
            (*out) *= uit->second;
        }
        else
        {
            pair<uncMapType::const_iterator,uncMapType::const_iterator> er( m_UncMap.equal_range (uit->second->getName()) );
            size_t k( distance(er.first, er.second ) );
            
            if (k==0)
                throw MyException("shouldn't have happened");
            
            for (uint i=1; i<k; i++)
                (*out) *= uit->second;
        }
    }
    
    return make_pair(true,boost::shared_ptr<ConstraintTermIF>(out));
}

boost::shared_ptr<ConstraintTermIF> ProductTerm::mapTermVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const
{
    vector<boost::shared_ptr<DecisionVariableIF> > vars;
    vector<boost::shared_ptr<UncertaintyIF> > unc;
    
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++)
    {
        map<string,boost::shared_ptr<DecisionVariableIF> >::const_iterator m_it( mapFromOldToNewVars.find( v_it->first ) );
        
        if (m_it==mapFromOldToNewVars.end())
        {
            vars.push_back(v_it->second);
        }
        else
        {
            vars.push_back(m_it->second);
        }
    }
    
    for (uncIterator u_it = uncBegin(); u_it != uncEnd(); u_it++)
    {
        unc.push_back(u_it->second);
    }
    
    boost::shared_ptr<ConstraintTermIF> termOut ( new ProductTerm( getCoeff(), unc, vars ) );
    return termOut;
}

boost::shared_ptr<ConstraintTermIF> ProductTerm::mapTermUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const
{
    vector<boost::shared_ptr<DecisionVariableIF> > vars;
    vector<boost::shared_ptr<UncertaintyIF> > unc;
    
    for (uncIterator u_it = uncBegin(); u_it != uncEnd(); u_it++)
    {
        map<string,boost::shared_ptr<UncertaintyIF> >::const_iterator m_it( mapFromOldToNewUnc.find( u_it->first ) );
        
        if (m_it==mapFromOldToNewUnc.end())
        {
            unc.push_back(u_it->second);
        }
        else
        {
            unc.push_back(m_it->second);
        }
    }
    
    
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++)
    {
        vars.push_back(v_it->second);
    }
    
    boost::shared_ptr<ConstraintTermIF> termOut ( new ProductTerm( getCoeff(), unc, vars ) );
    return termOut;
}

boost::shared_ptr<LHSExpression> ProductTerm::mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const
{
    // first check if the product term contains any variables that need to be mapped
    bool noneFound(true);
    vector<map<string, boost::shared_ptr<LHSExpression> >::const_iterator> expr_it_vec;
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++)
    {
        map<string, boost::shared_ptr<LHSExpression> >::const_iterator expr_it (mapFromVarToExpression.find( v_it->second->getName() ) );
        expr_it_vec.push_back( expr_it );
        if (expr_it != mapFromVarToExpression.end() )
            noneFound = false; // at least one variable must be mapped
    }
    
    if ( (mapFromVarToExpression.empty()) || (noneFound) )
    {
        boost::shared_ptr<LHSExpression> same(new LHSExpression());
        same->add(boost::shared_ptr<const ConstraintTermIF>(this->Clone()));
        return same;
    }
    
    boost::shared_ptr<LHSExpression> out( new LHSExpression() );
    out->add( m_coeff );
    
    vector<map<string, boost::shared_ptr<LHSExpression> >::const_iterator>::const_iterator expr_vec_it(expr_it_vec.begin());
    
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++, expr_vec_it++)
    {
        map<string, boost::shared_ptr<LHSExpression> >::const_iterator expr_it( *expr_vec_it );
        
        boost::shared_ptr<LHSExpression> expr( new LHSExpression() );
        if (expr_it == mapFromVarToExpression.end() )
            expr->add( 1., (*v_it).second );
        else
            expr->add( expr_it->second );
        
        (*out) *= expr;
    }
    
    for (uncIterator u_it = uncBegin(); u_it!= uncEnd(); u_it++)
        *out *= u_it->second;
    
    return out;
}

boost::shared_ptr<LHSExpression> ProductTerm::mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const
{
    boost::shared_ptr<LHSExpression> out( new LHSExpression() );
    out->add( m_coeff );
    
    for (varsIterator v_it = varsBegin(); v_it != varsEnd(); v_it++)
        *out *= v_it->second;
    
    for (uncIterator u_it = uncBegin(); u_it!= uncEnd(); u_it++)
    {
        
        map<string, boost::shared_ptr<LHSExpression> >::const_iterator expr_it (mapFromUncToExpression.find( u_it->second->getName() ) );
        
        boost::shared_ptr<LHSExpression> expr( new LHSExpression() );
        if (expr_it == mapFromUncToExpression.end() )
            expr->add( 1., (*u_it).second );
        else
            expr->add( expr_it->second );
        
        (*out) *= expr;
    }
    
    return out;
}

boost::shared_ptr<ConstraintTermIF> ProductTerm::replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const
{
    
    boost::shared_ptr<ConstraintTermIF> pOut;
    
    if ( (term.size()>1) && this->m_DVMap.size()==1) // the term provided is nonlinear but the current term is linear then there is nothing to do
    {
        pOut = this->Clone();
    }
    else
    {
        uint nt (getNumTimesTermAppears(term));
        
        
        if (nt == 0)
        {
            pOut = this->Clone();
        }
        else
        {
            multimap<string, boost::shared_ptr<DecisionVariableIF> > newdvmap = m_DVMap;
            
            for (multimap<string, boost::shared_ptr<DecisionVariableIF> >::const_iterator tit = term.begin(); tit != term.end(); tit++)
            {
                for (uint i=1; i<=nt; i++)
                {
                    multimap<string, boost::shared_ptr<DecisionVariableIF> >::iterator it ( newdvmap.find( tit->first ) );
                    
                    if (it==newdvmap.end())
                        throw MyException("this shouldn't have happened!");
                    
                    newdvmap.erase(it);
                }
            }
            
            vector<boost::shared_ptr<DecisionVariableIF> > vvars;
            vector<boost::shared_ptr<UncertaintyIF> > vuncs;
            
            for (uint i=1; i<=nt; i++)
                vvars.push_back(var);
            
            for (multimap<string, boost::shared_ptr<DecisionVariableIF> >::const_iterator vit = newdvmap.begin(); vit!=newdvmap.end(); vit++)
                vvars.push_back(vit->second);
            
            for (uncIterator uit = uncBegin(); uit != uncEnd(); uit++)
                vuncs.push_back(uit->second);
            
            pOut = boost::shared_ptr<ConstraintTermIF>( new ProductTerm( m_coeff, vuncs, vvars ) );
        }
        
    }
    
    return pOut;
}

void ProductTerm::add(boost::shared_ptr<const ConstraintTermIF> other)
{
    if (!other->isProductTerm())
        throw MyException("cannot add non product term");
    
    if (!is_same(other))
        throw MyException("terms should be the same");
    
    m_coeff += other->getCoeff();
}

void ProductTerm::add_vars_involved_in_prod(dvContainer &dvs) const
{
    if ( hasNonlinearities() )
        dvs += *m_pDVContainer;
}

double ProductTerm::evaluate(const map<string,double>& valuesMap ) const
{
    if ( m_pUncContainer->getNumUncertainties() != 0 )
        throw MyException("need uncertainty values to evaluate this term");
    
    double out(m_coeff);
    
    for (varsIterator it = varsBegin(); it != varsEnd(); it++)
    {
        map<string,double>::const_iterator mit ( valuesMap.find( it->second->getName() ) );
        
        if (mit == valuesMap.end() )
            throw MyException("map provided not suitable");
        
        out *= mit->second;
    }
    return out;
}

constraintTermType ProductTerm::getType() const
{
    return prodTerm;
}

bool ProductTerm::hasNonlinearities() const
{
    return m_DVMap.size()>1;
}

bool ProductTerm::hasProdsUncertainties() const
{
    if ( m_UncMap.size() > 1)
        return true;
    
    return false;
}

bool ProductTerm::hasProdsContVars() const
{
    if ( m_DVMap.size() > 1)
        return true;
    
    return false;
}

bool ProductTerm::is_same(boost::shared_ptr<const ConstraintTermIF> other) const
{
    if (other->getType() != getType())
        return false;
    
    boost::shared_ptr<const ProductTerm> pOther = boost::static_pointer_cast<const ProductTerm>(other);
    
    return ( *this == *pOther );
}

uint ProductTerm::getNumTimesTermAppears(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term) const
{
    return getNTTermAppears(m_DVMap,term);
}

void ProductTerm::getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, boost::shared_ptr<DecisionVariableIF> > > &termMap) const
{
    GetAllProductsOf2Variables(m_DVMap,freqMap,termMap);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<ConstraintTermIF> ProductTerm::Clone() const
{
    vector<boost::shared_ptr<UncertaintyIF> > uncVec;
    vector<boost::shared_ptr<DecisionVariableIF> > varVec;
    
    for (ProductTerm::varsIterator vit = varsBegin(); vit != varsEnd(); vit++)
        varVec.push_back(vit->second);
    
    for (ProductTerm::uncIterator uit = uncBegin(); uit != uncEnd(); uit++)
        uncVec.push_back(uit->second);
    
    
    boost::shared_ptr<ConstraintTermIF> out ( new ProductTerm( this->getCoeff(), uncVec, varVec ) );
    
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ProductTerm::WriteToStream(ofstream &ofs) const
{
    if (!isWellDefined())
        throw MyException("Product term is not well defined");
    
    
    if ( (!isConstant()) || (m_coeff != 0.0))
        ofs << " " << m_coeff;
    
    
    for (dvContainer::const_iterator vit = m_pDVContainer->begin(); vit != m_pDVContainer->end(); vit++)
    {
        pair<varMapType::const_iterator,varMapType::const_iterator> er( m_DVMap.equal_range (vit->second->getName()) );
        size_t k( distance(er.first, er.second ) );
        
        ofs << " " << vit->second->getName();
        
        if ( k != 1 )
            ofs << "^" << k;
        
    }
    
    
    for (uncContainer::const_iterator uit = m_pUncContainer->begin(); uit != m_pUncContainer->end(); uit++)
    {
        pair<uncMapType::const_iterator,uncMapType::const_iterator> er( m_UncMap.equal_range (uit->second->getName()) );
        size_t k( distance(er.first, er.second ) );
        
        ofs << " " << uit->second->getName();
        
        if ( k != 1 )
            ofs << "^" << k;
        
    }
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Private Fuctions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ProductTerm::addVariable(boost::shared_ptr<DecisionVariableIF> pVariable)
{
    if ( (!pVariable->isBooleanVar()) || (!m_pDVContainer->varIsInvolved(pVariable)) )
        m_DVMap.insert( make_pair( pVariable->getName(), pVariable ) );
    *m_pDVContainer += pVariable;
}

void ProductTerm::addUncertainty(boost::shared_ptr<UncertaintyIF> pUncertainty)
{
    m_UncMap.insert( make_pair( pUncertainty->getName(), pUncertainty ) );
    *m_pUncContainer += pUncertainty;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%% NORM TERM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

NormTerm::NormTerm(const vector<boost::shared_ptr<LHSExpression> > &pExpressionVec) : m_pExpressionVec(pExpressionVec)
{
    if (m_pExpressionVec.empty())
        throw MyException("m_pExpressionVec cannot be empty");
    
    if (!isWellDefined())
        throw MyException("expression in norm term is not well defined");
    
    for (const_iterator it = m_pExpressionVec.begin(); it != m_pExpressionVec.end(); it++)
    {
        if ( (*it)->hasNormTerm() )
            throw MyException("expressions in norm term should not contain a norm term");
        
        (*m_pDVContainer) += *((*it)->getDVContainer());
        (*m_pUncContainer) += *((*it)->getUncContainer());
    }
    
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<ConstraintTermIF> NormTerm::mapTermVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const
{
    vector<boost::shared_ptr<LHSExpression> > vecOut;
    
    for (const_iterator it = m_pExpressionVec.begin(); it != m_pExpressionVec.end(); it++)
    {
        boost::shared_ptr<LHSExpression> pExpression ( (*it)->mapExprVars( mapFromOldToNewVars ) );
        vecOut.push_back( pExpression );
    }
    
    boost::shared_ptr<ConstraintTermIF> termOut ( new NormTerm( vecOut ) );
    return termOut;
}

boost::shared_ptr<ConstraintTermIF> NormTerm::mapTermUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const
{
    vector<boost::shared_ptr<LHSExpression> > vecOut;
    
    for (const_iterator it = m_pExpressionVec.begin(); it != m_pExpressionVec.end(); it++)
    {
        boost::shared_ptr<LHSExpression> pExpression ( (*it)->mapExprUnc( mapFromOldToNewUnc ) );
        vecOut.push_back( pExpression );
    }
    
    boost::shared_ptr<ConstraintTermIF> termOut ( new NormTerm( vecOut ) );
    return termOut;
}

boost::shared_ptr<LHSExpression> NormTerm::mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const
{
    vector<boost::shared_ptr<LHSExpression> > vecOut;
    
    for (const_iterator it = begin(); it != end(); it++)
    {
        boost::shared_ptr<LHSExpression> pExpression ( (*it)->mapVars( mapFromVarToExpression ) );
        vecOut.push_back( pExpression );
    }
    
    boost::shared_ptr<ConstraintTermIF> termOut ( new NormTerm( vecOut ) );
    
    boost::shared_ptr<LHSExpression> exprOut( new LHSExpression() );
    exprOut->add(termOut);
    
    return exprOut;
}

boost::shared_ptr<LHSExpression> NormTerm::mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const
{
    vector<boost::shared_ptr<LHSExpression> > vecOut;
    
    for (const_iterator it = begin(); it != end(); it++)
    {
        boost::shared_ptr<LHSExpression> pExpression ( (*it)->mapUncs( mapFromUncToExpression ) );
        vecOut.push_back( pExpression );
    }
    
    boost::shared_ptr<ConstraintTermIF> termOut ( new NormTerm( vecOut ) );
    
    boost::shared_ptr<LHSExpression> exprOut( new LHSExpression() );
    exprOut->add(termOut);
    
    return exprOut;
}

boost::shared_ptr<ConstraintTermIF> NormTerm::replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const
{
    vector<boost::shared_ptr<LHSExpression> > vecOut;
    
    for (const_iterator it = m_pExpressionVec.begin(); it != m_pExpressionVec.end(); it++)
    {
        boost::shared_ptr<LHSExpression> pExpression ( (*it)->replaceTermWithVar( term, var ) );
        vecOut.push_back( pExpression );
    }
    
    boost::shared_ptr<ConstraintTermIF> termOut ( new NormTerm( vecOut ) );
    return termOut;
}

void NormTerm::add_vars_involved_in_prod(dvContainer &dvs) const
{
    for ( const_iterator it = begin(); it!=end(); it++ )
        (*it)->add_vars_involved_in_prod(dvs);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

constraintTermType NormTerm::getType() const
{
    return normTerm;
}

bool NormTerm::hasNonlinearities() const
{
    for (const_iterator v_it = begin(); v_it!= end(); v_it++)
    {
        if ( (*v_it)->hasNonlinearities() )
            return true;
        
    }
    
    return false;
    
}

bool NormTerm::hasProdsUncertainties() const
{
    for (const_iterator v_it = begin(); v_it!= end(); v_it++)
    {
        if ( (*v_it)->hasProdsUncertainties() )
            return true;
        
    }
    
    return false;
}

bool NormTerm::hasProdsContVars() const
{
    for (const_iterator v_it = begin(); v_it!= end(); v_it++)
    {
        //for (LHSExpression::const_iterator it = (*v_it)->begin(); it != (*v_it)->end(); it++)
        if ( (*v_it)->hasProdsContVars() )
            return true;
        
    }
    
    return false;
    
}

bool NormTerm::isWellDefined() const
{
    
    for (const_iterator it = m_pExpressionVec.begin(); it != m_pExpressionVec.end(); it++)
    {
        if (!(*it)->isWellDefined())
            return false;
    }
    
    return true;
    
}

bool NormTerm::is_same(boost::shared_ptr<const ConstraintTermIF> other) const
{
    if (other->getType() != getType())
        return false;
    
    
    throw MyException("only one norm term allowed - you should not be here!");
}

uint NormTerm::getNumTimesTermAppears(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term) const
{
    uint out(0);
    
    for (const_iterator v_it = begin(); v_it!= end(); v_it++)
        out += (*v_it)->getNumTimesTermAppears(term);
    
    return out;
}

void NormTerm::getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, boost::shared_ptr<DecisionVariableIF> > > &termMap) const
{
    for (const_iterator v_it = begin(); v_it!= end(); v_it++)
        (*v_it)->getAllProductsOf2Variables(freqMap,termMap);
}

double NormTerm::evaluate(const map<string,double>& valuesMap ) const
{
    if ( m_pUncContainer->getNumUncertainties() != 0 )
        throw MyException("need uncertainty values to evaluate this term");
    
    double out(0.);
    
    for (const_iterator it = begin(); it!= end(); it++)
        out += (*it)->evaluate(valuesMap);
    
    return out;
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<ConstraintTermIF> NormTerm::Clone() const
{
    vector<boost::shared_ptr<LHSExpression> > vecOut;
    
    for (const_iterator it = begin(); it != end(); it++)
        vecOut.push_back( (*it)->Clone() );
    
    boost::shared_ptr<ConstraintTermIF> out ( new NormTerm(vecOut) );
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void NormTerm::WriteToStream(ofstream &ofs) const
{
    if (!isWellDefined())
        throw MyException("Norm term is not well defined");
    
    //ofs.showpos;
    ofs << " + || ( ";
    
    for ( vector<boost::shared_ptr<LHSExpression> >::const_iterator it = m_pExpressionVec.begin(); it != m_pExpressionVec.end(); it++)
    {
        if (it!=m_pExpressionVec.begin())
            ofs << ", ";
        
        (*it)->WriteToStream(ofs);
    }
    
    ofs << " ) ||_2";
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% LEFT-HAND SIDE EXPRESSION %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Iterators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

LHSExpression::const_iterator LHSExpression::find(boost::shared_ptr<const ConstraintTermIF> term) const
{
    for (const_iterator it = m_terms.begin(); it != m_terms.end(); it++)
    {
        if ( term.get() == it->get() )
            return it;
    }
    return (m_terms.end());
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Operators %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void LHSExpression::operator*=(boost::shared_ptr<const ConstraintTermIF> term)
{
    for (const_iterator it = begin(); it!=end(); it++)
        *(*it) *= term;
    
    *this->m_pDVContainer += *term->getDVContainer();
    *this->m_pUncContainer += *term->getUncContainer();
};

void LHSExpression::operator*=(boost::shared_ptr<const LHSExpression> other)
{
    boost::shared_ptr<LHSExpression> out (new LHSExpression() );
    
    for (const_iterator oit = other->begin(); oit!=other->end(); oit++)
    {
        boost::shared_ptr<ConstraintTermIF> oterm( (*oit)->Clone() );
        boost::shared_ptr<LHSExpression> sexpr( new LHSExpression() );
        (*sexpr) += boost::shared_ptr<LHSExpression>(this->Clone());
        (*sexpr) *= oterm;
        (*out) += sexpr;
    }
    
    *this = *out;
};

void LHSExpression::operator*=(boost::shared_ptr<UncertaintyIF> unc)
{
    for (const_iterator it = begin(); it != end(); it++)
        *(*it) *= unc;
    
    (*m_pUncContainer) += unc;
}

void LHSExpression::operator*=(boost::shared_ptr<DecisionVariableIF> var)
{
    for (const_iterator it = begin(); it != end(); it++)
        *(*it) *= var;
    
    (*m_pDVContainer) += var;
}

void LHSExpression::operator*=(double a)
{
    for (const_iterator it = begin(); it != end(); it++)
        *(*it) *= a;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void LHSExpression::add(double c)
{
    boost::shared_ptr<ConstraintTermIF> pT( new ProductTerm(c) );
    add(pT);
}

void LHSExpression::add(double c, boost::shared_ptr<DecisionVariableIF> pVariable)
{
    boost::shared_ptr<ConstraintTermIF> pT( new ProductTerm(c,pVariable) );
    add(pT);
}

void LHSExpression::add(double c, boost::shared_ptr<UncertaintyIF> pUncertainty,  boost::shared_ptr<DecisionVariableIF> pVariable)
{
    boost::shared_ptr<ConstraintTermIF> pT( new ProductTerm(c,pUncertainty,pVariable) );
    add(pT);
}

void LHSExpression::add(double c, boost::shared_ptr<UncertaintyIF> pUncertainty)
{
    boost::shared_ptr<ConstraintTermIF> pT( new ProductTerm(c,pUncertainty) );
    add(pT);
}


void LHSExpression::add(double c, boost::shared_ptr<DecisionVariableIF> pVariable1, boost::shared_ptr<DecisionVariableIF> pVariable2)
{
    boost::shared_ptr<ConstraintTermIF> pT( new ProductTerm(c,pVariable1,pVariable2) );
    add(pT);
}

void LHSExpression::add(double c, boost::shared_ptr<UncertaintyIF> pUncertainty, boost::shared_ptr<DecisionVariableIF> pVariable1, boost::shared_ptr<DecisionVariableIF> pVariable2)
{
    boost::shared_ptr<ConstraintTermIF> pT( new ProductTerm(c,pUncertainty,pVariable1,pVariable2) );
    add(pT);
}

void LHSExpression::add(boost::shared_ptr<const ConstraintTermIF> term)
{

    boost::shared_ptr<ConstraintTermIF> tClone( term->Clone() );
    
    if ( (term->isNormTerm()) && ( hasNormTerm() ) )
        throw MyException("a norm term is already present");
    
    if (term->isNormTerm())
        m_hasNormTerm=true;
    
    iterator it = find( tClone );
    if (it==end())
    {
        m_terms.push_back(tClone);
        
        *this->m_pDVContainer += *tClone->getDVContainer();
        *this->m_pUncContainer += *tClone->getUncContainer();
    }
    else
    {
        if (term->isProductTerm())
        {
            (*it)->add(tClone);
        }
        else
            throw MyException("you should not be here");
    }
}

void LHSExpression::add(boost::shared_ptr<const LHSExpression> expr)
{
    for (const_iterator it = expr->begin(); it != expr->end(); it++)
        add( *it);
}

void LHSExpression::add(double c, boost::shared_ptr<const ConstraintTermIF> term)
{
    boost::shared_ptr<ConstraintTermIF> scaled_term( term->Clone() );
    *scaled_term *= c;
    
    add(scaled_term);
}

void LHSExpression::add(double c, boost::shared_ptr<const LHSExpression> expr)
{
    
    boost::shared_ptr<LHSExpression> scaled_expr( expr->Clone() );
    *scaled_expr *= c;
    
    add(scaled_expr);
}

void LHSExpression::add(double c, boost::shared_ptr<const LHSExpression> pExpression,  boost::shared_ptr<DecisionVariableIF> pVariable)
{
    boost::shared_ptr<LHSExpression> scaled_expr( pExpression->Clone() );
    *scaled_expr *= c;
    *scaled_expr *= pVariable;
    
    add(scaled_expr);
}

void LHSExpression::add(double c, boost::shared_ptr<const LHSExpression> pExpression,  boost::shared_ptr<UncertaintyIF> pUnc)
{
    boost::shared_ptr<LHSExpression> scaled_expr( pExpression->Clone() );
    *scaled_expr *= c;
    *scaled_expr *= pUnc;
    
    add(scaled_expr);
}

boost::shared_ptr<LHSExpression> LHSExpression::mapExprVars(const map<string,boost::shared_ptr<DecisionVariableIF> > &mapFromOldToNewVars) const
{
    boost::shared_ptr<LHSExpression> exprOut( new LHSExpression() );
    
    for (const_iterator it = begin(); it != end(); it++)
        exprOut->add( (*it)->mapTermVars(mapFromOldToNewVars ) );
    
    return exprOut;
}

boost::shared_ptr<LHSExpression> LHSExpression::mapExprUnc(const map<string,boost::shared_ptr<UncertaintyIF> > &mapFromOldToNewUnc) const
{
    boost::shared_ptr<LHSExpression> exprOut( new LHSExpression() );
    
    for (const_iterator it = begin(); it != end(); it++)
        exprOut->add( (*it)->mapTermUnc(mapFromOldToNewUnc ) );
    
    return exprOut;
}

boost::shared_ptr<LHSExpression> LHSExpression::mapVars(const map<string, boost::shared_ptr<LHSExpression> > &mapFromVarToExpression) const
{
    boost::shared_ptr<LHSExpression> exprOut( new LHSExpression() );
    
    for (const_iterator it = begin(); it != end(); it++)
        exprOut->add( (*it)->mapVars(mapFromVarToExpression ) );
    
    return exprOut;
}

boost::shared_ptr<LHSExpression> LHSExpression::mapUncs(const map<string, boost::shared_ptr<LHSExpression> > &mapFromUncToExpression) const
{
    boost::shared_ptr<LHSExpression> exprOut( new LHSExpression() );
    
    for (const_iterator it = begin(); it != end(); it++)
        exprOut->add( (*it)->mapUncs(mapFromUncToExpression ) );
    
    return exprOut;
}

boost::shared_ptr<LHSExpression> LHSExpression::replaceTermWithVar(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, boost::shared_ptr<DecisionVariableIF> var) const
{
    boost::shared_ptr<LHSExpression> exprOut( new LHSExpression() );
    
    for (const_iterator it = begin(); it != end(); it++)
        exprOut->add( (*it)->replaceTermWithVar(term, var ) );
    
    return exprOut;
}

boost::shared_ptr<LHSExpression> LHSExpression::replaceBilinearTerm(map<pair<string,string>, boost::shared_ptr<DecisionVariableIF> > &allTerm, uint &count ) const
{
    boost::shared_ptr<LHSExpression> newLhs(new LHSExpression());
    
    for(ConstraintLHS_const_iterator tit = begin(); tit != end(); tit++)
    {
        if(!(*tit)->hasNonlinearities())
            newLhs->add((*tit));
        else{
            
            map< pair<string,string>, uint> freqMap;
            map< pair<string,string>, multimap<string, boost::shared_ptr<DecisionVariableIF> > > termMap;
            (*tit)->getAllProductsOf2Variables(freqMap, termMap);
            
            
            map< pair<string,string>, multimap<string, boost::shared_ptr<DecisionVariableIF> > >::const_iterator term = termMap.begin();
            for(; term != termMap.end(); term++)
            {
                pair<string, string> otherName(make_pair(term->first.second,term->first.first));
                
                multimap<string, boost::shared_ptr<DecisionVariableIF> > tmpTerm;
                boost::shared_ptr<DecisionVariableIF> dv1 ( m_pDVContainer->find(term->first.first)->second );
                boost::shared_ptr<DecisionVariableIF> dv2 ( m_pDVContainer->find(term->first.second)->second );
                
                tmpTerm.insert(make_pair(term->first.first, dv1) );
                tmpTerm.insert(make_pair(term->first.second, dv2) );
                
                if(allTerm.find(term->first) != allTerm.end())
                {
                    boost::shared_ptr<DecisionVariableIF> newdv = allTerm.find(term->first)->second;
                    newLhs->add((*tit)->replaceTermWithVar(tmpTerm, newdv));
                }
                else if(allTerm.find(otherName) != allTerm.end())
                {
                    boost::shared_ptr<DecisionVariableIF> newdv = allTerm.find(otherName)->second;
                    newLhs->add((*tit)->replaceTermWithVar(tmpTerm, newdv));
                }
                else
                {
                    boost::shared_ptr<DecisionVariableIF> bindv;
                    boost::shared_ptr<DecisionVariableIF> otherdv;
                    
                    pair<string, string> newDvMap;
                    if ( dv1->isBooleanVar() )
                    {
                        bindv = dv1;
                        otherdv = dv2;
                        
                        newDvMap = make_pair(dv1->getName(), dv2->getName());
                    }
                    else if (dv2->isBooleanVar())
                    {
                        bindv = dv2;
                        otherdv = dv1;
                        
                        newDvMap = make_pair(dv2->getName(), dv1->getName());
                    }
                    else
                        throw MyException("at least one of the two variables must be binary");
                    
                    boost::shared_ptr<DecisionVariableIF> newdv;
                    
                    string nme;
                    nme = "bl_"+bindv->getName()+"_"+otherdv->getName()+"_"+ boost::lexical_cast<string>(count++);
                    
                    if (otherdv->isBooleanVar())
                    {
                        
                        if ( (otherdv->isAdaptive()) || (bindv->isAdaptive()) )
                            newdv = boost::shared_ptr<DecisionVariableIF>( new AdaptVarBool(  nme, max( bindv->getTimeStage(), otherdv->getTimeStage() ) ) );
                        else
                            newdv = boost::shared_ptr<DecisionVariableIF>( new VariableBool(  nme ) );
                        
                    }
                    else if (otherdv->isIntegerVar())
                    {
                        
                        // calculate bounds
                        double lb, ub;
                        
                        lb = ((otherdv->getLB() > -INFINITY)?min( otherdv->getLB(), 0. ):(-INFINITY));
                        ub = ((otherdv->getUB() < INFINITY)?max( otherdv->getUB(), 0. ):(INFINITY));
                        
                        if ( (otherdv->isAdaptive()) || (bindv->isAdaptive()) )
                            newdv = boost::shared_ptr<DecisionVariableIF>( new AdaptVarInt(  nme, max( bindv->getTimeStage(), otherdv->getTimeStage() ), lb, ub ) );
                        else
                            newdv = boost::shared_ptr<DecisionVariableIF>( new VariableInt(  nme, lb, ub ) );
                        
                    }
                    else if (otherdv->isRealVar() )
                    {
                        double lb, ub;
                        
                        lb = ((otherdv->getLB() > -INFINITY)?min( otherdv->getLB(), 0. ):(-INFINITY));
                        ub = ((otherdv->getUB() < INFINITY)?max( otherdv->getUB(), 0. ):(INFINITY));
                        
                        if ( (otherdv->isAdaptive()) || (bindv->isAdaptive()) )
                            newdv = boost::shared_ptr<DecisionVariableIF>( new AdaptVarDouble(nme, max( bindv->getTimeStage(), otherdv->getTimeStage() ), lb, ub ) );
                        else
                            newdv = boost::shared_ptr<DecisionVariableIF>( new VariableDouble( nme, lb, ub ) );
                        
                    }
                    
                    else throw MyException("unknown variable type");
                    
                    newLhs->add((*tit)->replaceTermWithVar(tmpTerm, newdv));
                    
                    allTerm.insert(make_pair(newDvMap, newdv));
                }
            }
        }
    }
    return newLhs;
}

pair<bool,boost::shared_ptr<LHSExpression> > LHSExpression::factorOut(boost::shared_ptr<UncertaintyIF> unc) const
{
    boost::shared_ptr<LHSExpression> out(new LHSExpression());
    bool found(false);
    
    for (const_iterator it = begin(); it != end(); it++)
    {
        if ( (*it)->isProductTerm() )
        {
            boost::shared_ptr<ProductTerm> pPT = boost::static_pointer_cast<ProductTerm>(*it);
            pair<bool,boost::shared_ptr<ConstraintTermIF> > tmp( pPT->factorOut(unc) );
            if (tmp.first)
            {
                (*out) += tmp.second;
                found = true;
            }
        }
    }
    
    //if (!out->isWellDefined())
    //    throw MyException("expression should be well defined");
    
    if (!found)
        (*out) += 0.;
    
    return make_pair(found,out);
}

void LHSExpression::add_vars_involved_in_prod(dvContainer &dvs) const
{
    for (const_iterator it = begin(); it != end(); it++ )
        (*it)->add_vars_involved_in_prod(dvs);
};

void LHSExpression::add_int_vars(dvContainer &dvs) const
{
    m_pDVContainer->add_int_vars(dvs);
};

double LHSExpression::evaluate(const map<string,double>& valuesMap ) const
{
    double out(0.);
    
    for (const_iterator it = begin(); it != end(); it++)
        out += (*it)->evaluate( valuesMap );
    
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double LHSExpression::getSumConstantTerms() const
{
    double out(0.);
    
    for (ConstraintLHS_const_iterator it = begin(); it!=end(); it++)
    {
        if ( (*it)->isConstant() )
            out += (*it)->getCoeff();
    }
    
    return out;
    
}

bool LHSExpression::hasNonlinearities() const
{
    for (const_iterator it = begin(); it != end(); it++)
        if ( (*it)->hasNonlinearities() )
            return true;
    
    return false;
}

bool LHSExpression::hasProdsContVars() const
{
    for (const_iterator it = begin(); it != end(); it++)
        if ( (*it)->hasProdsContVars() )
            return true;
    
    return false;
}

bool LHSExpression::hasProdsUncertainties() const
{
    for (const_iterator it = begin(); it != end(); it++)
        if ( (*it)->hasProdsUncertainties() )
            return true;
    
    return false;
}

bool LHSExpression::isDeterministic() const
{
    return ( !m_pUncContainer->getNumUncertainties());
}

bool LHSExpression::isConstant() const
{
    for (const_iterator it = begin(); it != end(); it++)
        if ( !(*it)->isConstant() )
            return false;
    
    return true;
}


bool LHSExpression::isLinear() const
{
    for (const_iterator it = begin(); it != end(); it++)
        if ( !(*it)->isLinear() )
            return false;
    
    return true;
}

bool LHSExpression::isQuadratic() const
{
    bool someQuadratic(false);
    for (const_iterator it = begin(); it != end(); it++)
    {
        if ( !((*it)->isLinear()) && !((*it)->isQuadratic()) )
            return false;
        if ( (*it)->isQuadratic() )
            someQuadratic=true;
    }
    return someQuadratic;
}

boost::shared_ptr<LHSExpression> LHSExpression::getLinearPart() const
{
    boost::shared_ptr<LHSExpression> out( new LHSExpression() );
    for (const_iterator tit = begin(); tit != end(); tit++)
    {
        if ( (*tit)->isProductTerm() )
            (*out) += (*tit)->Clone();
    }
    return out;
}

boost::shared_ptr<NormTerm> LHSExpression::getNormTerm() const
{
    if (!hasNormTerm() )
        throw MyException("this expression does not have a norm term");
    
    boost::shared_ptr<NormTerm> out;
    for (const_iterator tit = begin(); tit != end(); tit++)
    {
        if ( (*tit)->isNormTerm() )
        {
            out = boost::static_pointer_cast<NormTerm>((*tit)->Clone());
            return out;
        }
    }
    
    throw MyException("norm term not found");
    
}

boost::shared_ptr<LHSExpression> LHSExpression::getDeterministicLinearPart() const
{
    boost::shared_ptr<LHSExpression> out( new LHSExpression() );
    
    for (const_iterator it = begin(); it != end(); it++)
    {
        if ( ((*it)->isProductTerm()) && ((*it)->getNumUncertainties()==0) )
            (*out) += (*it)->Clone();
    }
    
    return out;
}

uint LHSExpression::getNumTimesTermAppears(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term) const
{
    uint out(0);
    
    for (const_iterator it = begin(); it != end(); it++)
        out += (*it)->getNumTimesTermAppears(term);
    
    return out;
}

void LHSExpression::getAllProductsOf2Variables(map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, boost::shared_ptr<DecisionVariableIF> > > &termMap) const
{
    for (const_iterator it = begin(); it != end(); it++)
        (*it)->getAllProductsOf2Variables(freqMap,termMap);
    
}

boost::shared_ptr<const dvContainer> LHSExpression::getDVContainer() const
{
    return m_pDVContainer;
}

boost::shared_ptr<const uncContainer> LHSExpression::getUncContainer() const
{
    return m_pUncContainer;
}

boost::shared_ptr<DecisionVariableIF> LHSExpression::getVar(string varName) const
{
    return( m_pDVContainer->findthrow(varName)->second );
}

bool LHSExpression::varIsInvolved(boost::shared_ptr<DecisionVariableIF> dv) const
{
    return m_pDVContainer->varIsInvolved(dv);
}

bool LHSExpression::AnyVarIsInvolved(dvContainer& dvs) const
{
    return m_pDVContainer->AnyVarIsInvolved(dvs);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

boost::shared_ptr<LHSExpression> LHSExpression::Clone() const
{
    boost::shared_ptr<LHSExpression> out ( new LHSExpression() );
    for (LHSExpression::const_iterator it = begin(); it != end(); it++)
        (*out) += (*it)->Clone();
    
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void LHSExpression::WriteToStream(ofstream& ofs) const
{
    //ofs.showpos;
    
    for (const_iterator it = begin(); it!=end(); it++)
        (*it)->WriteToStream(ofs);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% Protected Members %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

LHSExpression::iterator LHSExpression::find(boost::shared_ptr<ConstraintTermIF> term)
{
    for (iterator it = m_terms.begin(); it != m_terms.end(); it++)
    {
        if ( (*it)->is_same(term) )
            return it;
    }
    return (m_terms.end());
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% TOOL FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool GetAllProductsOf2Variables(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &term, map< pair<string,string>, uint> &freqMap, map< pair<string,string>, multimap<string, boost::shared_ptr<DecisionVariableIF> > > &termMap)
{
    
    if (term.size()==2)
    {
        string v1nme(term.begin()->first);
        string v2nme(term.rbegin()->first);
        
        if (v1nme.compare(v2nme) <= 0)
        {
            
            // add term to the term map
            termMap.insert(make_pair(make_pair(v1nme, v2nme), term));
            map< pair<string,string>, uint>::iterator fmit = freqMap.find(make_pair(v1nme, v2nme));
            
            // add 1 to the frequency map
            if (fmit==freqMap.end())
                freqMap.insert(make_pair(make_pair(v1nme, v2nme), 1));
            else
                fmit->second += 1;
        }
        else
        {
            // add term to the term map
            termMap.insert(make_pair(make_pair(v2nme, v1nme), term));
            
            // add 1 to the frequency map
            map< pair<string,string>, uint>::iterator fmit = freqMap.find(make_pair(v2nme, v1nme));
            if (fmit==freqMap.end())
                freqMap.insert(make_pair(make_pair(v2nme, v1nme), 1));
            else
                fmit->second += 1;
        }
        
        return true;
    }
    else if (term.size()<2)
        return true;
    else
    {
        bool tmp(true);
        
        for (multimap<string, boost::shared_ptr<DecisionVariableIF> >::const_iterator tit=term.begin(); tit!=term.end(); tit++)
        {
            // duplicate the term
            multimap<string, boost::shared_ptr<DecisionVariableIF> > termClone(term);
            
            // from the clone, delete the current term
            multimap<string, boost::shared_ptr<DecisionVariableIF> >::iterator tcit = termClone.find(tit->first);
            termClone.erase(tcit);
            
            // pass the subset term to the function recursively
            if (!GetAllProductsOf2Variables(termClone, freqMap, termMap))
                tmp=false;
            
        }
        
        return tmp;
    }
}

bool comparison(const pair<string, boost::shared_ptr<DecisionVariableIF> >& p1, const pair<string, boost::shared_ptr<DecisionVariableIF> >& p2) {
    return p1.first < p2.first;
}

uint getNTTermAppears(const multimap<string, boost::shared_ptr<DecisionVariableIF> > &sup, const multimap<string, boost::shared_ptr<DecisionVariableIF> > &sub)
{
    
    //multimap<string, boost::shared_ptr<DecisionVariableIF> >::key_compare thiscomp ( sub.key_comp() );
    
    bool termIsSubset ( includes( sup.begin(), sup.end(), sub.begin(), sub.end(), comparison ) );
    
    if (!termIsSubset)
        return 0;
    
    multimap<string, boost::shared_ptr<DecisionVariableIF> > newdvmap = sup;
    
    for (multimap<string, boost::shared_ptr<DecisionVariableIF> >::const_iterator tit = sub.begin(); tit != sub.end(); tit++)
    {
        multimap<string, boost::shared_ptr<DecisionVariableIF> >::iterator it ( newdvmap.find( tit->first ) );
        newdvmap.erase(it);
    }
    
    return 1+getNTTermAppears(newdvmap,sub);
}
