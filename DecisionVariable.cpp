//
//  DecisionVariable.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "DecisionVariable.hpp"
#include "IncludeFiles.hpp"


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% DECISION VARIABLE INTERFACE %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DecisionVariableIF::DecisionVariableIF(string name, decVariableType type, double lb, double ub, bool isAdaptive, uint timeStage):
m_name(name),
m_type(type),
m_lb(lb),
m_ub(ub),
m_isAdaptive(isAdaptive),
m_timeStage(timeStage)
{
    if (name.length()==0)
        throw MyException("dv name cannot be empty");

    if ( (timeStage == 1) && (isAdaptive) )
        throw MyException("attempted to create an adaptive variable with time stage =1");
    
    if (m_lb > m_ub)
        throw MyException("the lower bound should be <= the upper bound");

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void DecisionVariableIF::setLB(double lb)
{
    if ( lb > this->getUB() )
        throw MyException("lb should be <= ub");

    m_lb = lb;
}

void DecisionVariableIF::setUB(double ub)
{
    if ( ub < this->getLB() )
        throw MyException("lb should be <= ub");

    m_ub = ub;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


string DecisionVariableIF::writeLB() const
{
    return to_string(m_lb);
}

string DecisionVariableIF::writeUB() const
{
    return to_string(m_ub);
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% STATIC VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% STATIC BOOL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

VariableBool::VariableBool(string name, double lb, double ub):
VariableIF(name, boolDV, max(0.,lb), min(1.0,ub))
{
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void VariableBool::setLB(double lb)
{
    DecisionVariableIF::setLB(max(0., lb));
}

void VariableBool::setUB(double ub)
{
    DecisionVariableIF::setUB(min(1.0, ub));
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Clone Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPVarIF_Ptr VariableBool::Clone()
{
    ROCPPVarIF_Ptr out(new VariableBool( this->getName(),this->getLB(),this->getUB()) );
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% STATIC INT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

VariableInt::VariableInt(string name, double lb, double ub):
VariableIF(name, intDV, lb, ub)
{
}

ROCPPVarIF_Ptr VariableInt::Clone()
{
    ROCPPVarIF_Ptr out( new VariableInt( this->getName(), this->getLB(), this->getUB()) );
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% STATIC DOUBLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

VariableDouble::VariableDouble(string name, double lb, double ub):
VariableIF(name, contDV, lb, ub)
{
}

ROCPPVarIF_Ptr VariableDouble::Clone()
{
    ROCPPVarIF_Ptr out( new VariableDouble( this->getName(), this->getLB(), this->getUB()) );
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% ADAPTIVE VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% ADAPTIVE BOOL %%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


AdaptVarBool::AdaptVarBool(string name, uint timeStage, double lb, double ub):
AdaptiveVariableIF(name, boolDV, timeStage, max(0.,lb), min(1.0,ub))
{
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void AdaptVarBool::setLB(double lb)
{
    DecisionVariableIF::setLB(max(0., lb));
}

void AdaptVarBool::setUB(double ub)
{
    DecisionVariableIF::setUB(min(1.0, ub));
}

ROCPPVarIF_Ptr AdaptVarBool::Clone()
{
    ROCPPVarIF_Ptr out( new AdaptVarBool( this->getName(), this->getTimeStage(), this->getLB(), this->getUB()) );
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% ADAPTIVE INT %%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

AdaptVarInt::AdaptVarInt(string name, uint timeStage, double lb, double ub):
AdaptiveVariableIF(name, intDV, timeStage, lb, ub)
{
}

ROCPPVarIF_Ptr AdaptVarInt::Clone()
{
    ROCPPVarIF_Ptr out( new AdaptVarInt(  this->getName(), this->getTimeStage(), this->getLB(), this->getUB()) );
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% ADAPTIVE DOUBLE %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

AdaptVarDouble::AdaptVarDouble(string name, uint timeStage, double lb, double ub):
AdaptiveVariableIF(name, contDV, timeStage, lb, ub)
{
}

ROCPPVarIF_Ptr AdaptVarDouble::Clone()
{
    ROCPPVarIF_Ptr out( new AdaptVarDouble(  this->getName(), this->getTimeStage(), this->getLB(), this->getUB()) );
    return out;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% DECISION VARIABLE CONTAINER %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void dvContainer::operator+=(const dvContainer &a)
{
    // iterate through the decision variables in a
    for (map<string, ROCPPVarIF_Ptr >::const_iterator dv_it = a.m_DVMap.begin(); dv_it != a.m_DVMap.end(); dv_it++)
        *this += dv_it->second;
}

void dvContainer::operator+=(const ROCPPVarIF_Ptr& dv)
{

    string varName(dv->getName());

    dvMapType::const_iterator mn_it(m_DVMap.find(varName));

    if ( (mn_it != m_DVMap.end()) && (mn_it->second != dv) )
        throw MyException("attempted to add two different variables with same name to the dv container");

    if ( mn_it == m_DVMap.end() )
    {
        m_DVMap[varName]=dv;
        if ( (dv->isAdaptive() ) && ( dv->getType()==contDV) )
            m_adaptiveContVarsCnt++;

        if (dv->getType()==contDV )
            m_numContVars++;

        if (dv->getType()==boolDV )
            m_numBoolVars++;

        if (dv->getType()==intDV )
            m_numIntVars++;

        if (dv->isAdaptive())
            m_adaptiveVarsCnt++;

        m_timeStage = max( m_timeStage, dv->getTimeStage() );
    }


}

dvContainer::const_iterator dvContainer::find(string varName) const
{
    const_iterator it ( m_DVMap.find( varName ) );
    return it;
}

dvContainer::const_iterator dvContainer::findthrow(string varName) const
{
    const_iterator it( find(varName) );
    if ( it==end() )
        throw MyException("variable not found");

    return it;
}

void dvContainer::add_int_vars(dvContainer &dvs) const
{
    for (const_iterator it = begin(); it!=end(); it++)
        if ( it->second->isIntegerVar() )
            dvs += it->second;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


bool dvContainer::AnyVarIsInvolved(dvContainer& dvs) const
{
    for (dvContainer::const_iterator vit = dvs.begin(); vit != dvs.end(); vit++)
    {
        if (varIsInvolved(vit->second))
            return true;
    }
    return false;
}

bool dvContainer::varIsInvolved(ROCPPVarIF_Ptr dv) const
{
    const_iterator it (find(dv->getName()));
    return (it!=end());
}



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% CREATE VAR FUNCTION %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


ROCPPVarIF_Ptr createVariable( string name, decVariableType type, bool isAdaptive, uint timeStage, double lb, double ub)
{
    if(!isAdaptive)
    {
        if(timeStage > 1)
            throw MyException("The time stage of static variable must be 1");
        switch (type) {
            case boolDV:
                if(ub < INFINITY)
                    return ROCPPVarIF_Ptr( new  VariableBool(name, lb, ub) );
                else
                    return ROCPPVarIF_Ptr( new  VariableBool(name) );
                break;
            case intDV:
                return ROCPPVarIF_Ptr( new  VariableInt(name, lb, ub) );
            case contDV:
                return ROCPPVarIF_Ptr( new  VariableDouble(name, lb, ub) );
            default:
            {
                throw MyException("Variable type can not be recognized");
                break;
            }
        }
    }
    else
    {
        switch (type) {
            case boolDV:
                if(ub < INFINITY)
                    return ROCPPVarIF_Ptr( new  AdaptVarBool(name, timeStage, lb, ub) );
                else
                    return ROCPPVarIF_Ptr( new  AdaptVarBool(name, timeStage) );
                break;
            case intDV:
                return ROCPPVarIF_Ptr( new  AdaptVarInt(name, timeStage, lb, ub) );
            case contDV:
                return ROCPPVarIF_Ptr( new  AdaptVarDouble(name, timeStage, lb, ub) );
            default:
            {
                throw MyException("Variable type can not be recognized");
                break;
            }
        }
    }
}

