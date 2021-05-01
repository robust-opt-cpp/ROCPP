//
//  PWDecisionRule.cpp
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
#include "ReformulationOrchestrator.hpp"
#include "VariableConverter.hpp"
#include "OptModelConverters.hpp"
#include "RobustifyEngine.hpp"
#include "helpersOpt.hpp"
#include "FileRelations.hpp"
#include "IndexSetCreator.hpp"
#include "DecisionRule.hpp"
#include "PWDecisionRule.hpp"
#include "UncertaintyConverter.hpp"
#include "Interface.hpp"



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
    map< string, vector< ROCPPConstraint_Ptr> >::const_iterator mit( m_partitionUSconstraints.find(partition) );
    
    if (mit == m_partitionUSconstraints.end() )
        throw MyException("partition not found");
    
    return (mit->second.begin());
}

PartitionConstructorIF::usconstraints_iterator PartitionConstructorIF::USCend(string partition) const
{
    map< string, vector< ROCPPConstraint_Ptr> >::const_iterator mit( m_partitionUSconstraints.find(partition) );
    
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
    
//    // phebe added 2021/04/03
//    // for each subset of the partition, for each uncertain parameter, create one uncertain parameter for that subset
//    for (map<string, map<string,uint> >::const_iterator pm_it=m_partitionsMap.begin(); pm_it!=m_partitionsMap.end(); pm_it++)
//    {
//        map<string, shared_ptr<UncertaintyIF> > tmp;
//        for (OptimizationModelIF::uncertaintiesIterator unc_it = pIn->uncertaintiesBegin(); unc_it != pIn->uncertaintiesEnd(); unc_it++)
//        {
//            shared_ptr<UncertaintyIF> unc( new UncertaintyIF( unc_it->second->getName() + "_" + pm_it->first ) );
//
//            tmp.insert(make_pair(unc_it->second->getName(), unc));
//        }
//
//        m_UncMap.insert(make_pair(pm_it->first, tmp));
//    }
    
    
    // end phebe added 2021/04/03
    
    
    for (map<string, map<string,uint> >::const_iterator pm_it=m_partitionsMap.begin(); pm_it!=m_partitionsMap.end(); pm_it++)
    {
        vector<ROCPPConstraint_Ptr> tmp;
        
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
                map<pair<string,uint>, ROCPPExpr_Ptr>::const_iterator loc2( m_uncToBreakpointMap.find( make_pair(u_it->second->getName(), bpNum) ) );
                if (loc2==m_uncToBreakpointMap.end())
                    throw MyException("pair not found in m_uncToBreakpointMap");
                
                ROCPPClassicConstraint_Ptr pOutConstraint(new IneqConstraint(true));
                
                pOutConstraint->add_lhs( 1. , u_it->second );
                //pOutConstraint->add_lhs( 1. , getUncOnPartition( pm_it->first, u_it->second->getName()) );
                ROCPPExpr_Ptr expr( loc2->second->Clone() );
                (*expr) *= -1.;
                pOutConstraint->add_lhs( expr );
                pOutConstraint->set_rhs(make_pair(0.,true));
                
                
                tmp.push_back( pOutConstraint );
            }
            if (bpNum>1)
            {
                map<pair<string,uint>, ROCPPExpr_Ptr>::const_iterator loc2( m_uncToBreakpointMap.find( make_pair(u_it->second->getName(), bpNum-1) ) );
                if (loc2==m_uncToBreakpointMap.end())
                    throw MyException("pair not found in m_uncToBreakpointMap");
                
                ROCPPClassicConstraint_Ptr pOutConstraint(new IneqConstraint(true));
                
                pOutConstraint->add_lhs( -1. , u_it->second );
                //pOutConstraint->add_lhs( -1. , getUncOnPartition( pm_it->first, u_it->second->getName()) );
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
    map< pair<string,uint>, ROCPPExpr_Ptr> ::const_iterator bp(m_uncToBreakpointMap.find(uncOnPartition));
    
    if (bp == m_uncToBreakpointMap.end())
        throw MyException("Partition and uncertainty pair not found");
        
    return bp->second;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% Protected Function %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//ROCPPUnc_Ptr PartitionConstructorIF::getUncOnPartition(string partition, string origUncName) const
//{
//    // map<string, map<string,ROCPPUnc_Ptr> > m_UncMap;
//
//    map<string, map<string,ROCPPUnc_Ptr> >::const_iterator subset_it = m_UncMap.find(partition);
//
//    if (subset_it==m_UncMap.end())
//        throw MyException("Partition subset not found");
//
//    map<string,ROCPPUnc_Ptr>::const_iterator unc_it = subset_it->second.find(origUncName);
//
//    if (unc_it==subset_it->second.end())
//        throw MyException("Uncertain parameter not found");
//
//    return unc_it->second;
//}

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


//map<string,ROCPPUnc_Ptr> PartitionConstructorIF::getMapFromOriginalUncToSubsetUnc(string subset) const
//{
//    map<string, map<string,ROCPPUnc_Ptr> >::const_iterator mit(m_UncMap.find(subset));
//    if (mit==m_UncMap.end())
//        throw MyException("Uncertain parameter not found in m_UncMap");
//
//    return (mit->second);
//}


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
//%%%%%%%%%%%%%%%%%%%%%% PIECEWISE APPROXIMATOR %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PiecewiseDecisionRule::PiecewiseDecisionRule(const map<string,uint> &numPartitionsMap, double bigM, bool useExplicitNACs, string folder) : m_numPartitionsMap(numPartitionsMap), m_bigM(bigM), m_useExplicitNACs(useExplicitNACs), m_folder(folder)
{
    //m_numBits=numBits;
    
    m_pCVA = ROCPPContinuousVarsDR_Ptr(new LinearDecisionRule());
    m_pDVA = ROCPPDiscreteVarsDR_Ptr(new ConstantDecisionRule());
    
    // ******************** MI to MB converter ******************************************************
    
    m_pMItoMB_Bilinear = ROCPPMItoMB_Ptr(new BinaryConverter());
    
    m_pBPA = ROCPPUncSetRealVarApprox_Ptr( new UncertaintySetRealVarApproximator(0));
    
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void PiecewiseDecisionRule::initialize(ROCPPOptModelIF_Ptr pIn)
{
    
    // partition constructor
    
    map<string, uint>::const_iterator pMap;
    
    map<string, uint> numMap;
    
    for (UncertainOptimizationModel::uncertaintiesIterator u_it=pIn->uncertaintiesBegin(); u_it!=pIn->uncertaintiesEnd(); u_it++)
    {
        pMap = m_numPartitionsMap.find(u_it->second->getName());
        
        if ( u_it->second->isObservable() && pMap != m_numPartitionsMap.end())
            numMap.insert(pair<string,uint>(pMap->first,pMap->second));
        else
            numMap.insert(pair<string,uint>(u_it->first,1));
    }
    
    m_numPartitionsMap = numMap;
    m_pPartConstructor = ROCPPParConstructor_Ptr(new StaticPartitionConstructor(numMap));
    
    // decision rule approximators
    
    // do DDU appoximation using piecewise constant decision rule
    m_pCVA = ROCPPContinuousVarsDR_Ptr(new LinearDecisionRule());
    m_pDVA = ROCPPDiscreteVarsDR_Ptr(new ConstantDecisionRule());
    
    // -- partition related --
    
    // find the maximum value inside partitionsMap -- this will determine the number of elements needed in PartitionConverter
    
    map<string,uint>::const_iterator mit(max_element(m_numPartitionsMap.begin(), m_numPartitionsMap.end(),
                                                     [] (const std::pair<string,uint>& a, const std::pair<string,uint>& b)->bool{ return a.second < b.second; }));
    
    m_pPartConverter = ROCPPParConverter_Ptr(new PartitionConverter( (to_string(mit->second)).size() ) );
    m_numPartitionsStr = m_pPartConverter->convertPartitionToString(m_numPartitionsMap, pIn);
    
    
    // ******************** MI to MB converter ******************************************************
    
    m_pMItoMB_Bilinear = ROCPPMItoMB_Ptr(new BinaryConverter());
    
    // ******************** bilinear term reformulation ******************************************************
    //m_pBTR = ROCPPBilinearReform_Ptr(new BTR_bigM());
    
    
    // ******************* uncertainty set real var approximator ***********************************
    //m_pUSRVA = ROCPPUncSetRealVarApprox_Ptr( new UncertaintySetRealVarApproximator(numBits));
    m_pBPA = ROCPPUncSetRealVarApprox_Ptr( new UncertaintySetRealVarApproximator(0));
}

bool PiecewiseDecisionRule::isApplicable(ROCPPOptModelIF_Ptr pIn) const
{
    if (!pIn->isUncertainOptimizationModel())
    {
        cout << "Cannot apply PiecewiseDecisionRule to deterministic model" << endl;
        return false;
    }
    
    if ((pIn->getObjType() == stochastic) && ( (!pIn->hasRectangularUncertaintySet()) || (pIn->hasDecisionDependentUncertaintySet()) ) )
    {
        cout << "PiecewiseDecisionRule approximator is only applicable to stochastic problems with rectangular and decision-independent uncertainty set or to robust problems" << endl;
        return false;
    }
    
    if (pIn->hasRealVarsInUncertaintySet())
    {
        cout << "PiecewiseDecisionRule approximator does not apply to problems with uncertainty set affected by real valued decisions" << endl;
        return false;
    }
    
    vector<ROCPPExpr_Ptr> objs(pIn->getObj()->getObj());
    
    vector<ROCPPExpr_Ptr>::const_iterator obj(objs.begin());
    
    for(; obj != objs.end(); obj++)
    {
        if((*obj)->hasProdsUncertainties())
        {
            cout << "Cannot deal with products of uncertainties at this time" << endl;
            return false;
        }
    }
    
    if ( (pIn->hasNonlinearities()) ) // here also will need to check if we have real valued decisions affecting the uncertainty set
    {
        cout << "Cannot handle nonlinear problem at present" << endl;
        return false;
    }
    
    return true;
}


ROCPPOptModelIF_Ptr PiecewiseDecisionRule::approximate(ROCPPOptModelIF_Ptr pIn)
{
    
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "================== APPROXIMATING USING PREPARTITIONING ==================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;

    auto start = chrono::high_resolution_clock::now();
    
    initialize(pIn);
    
    // first write the problem in epigraph form
    ROCPPOptModelIF_Ptr pModel(pIn->Clone());
    pModel->add_epigraph();
    
    // Create output model
    ROCPPOptModelIF_Ptr pOut( new ROCPPUncSSOptModel() );
    
    // do linear decision rule
    ROCPPOptModelIF_Ptr pLDRModel( m_pCVA->convertVar(pModel, true) );
    
    // do constant decision rule
    ROCPPOptModelIF_Ptr pMiddle( m_pDVA->convertVar(pLDRModel, true) );
    
    // calculate marginal support of approximated model
    map<string,pair<double,double> > margSupp;
    
    // prepare the partition constructor
    // only deal with constraints
    string solver="gurobi";
    map<string,pair<double,double> > OAmargSupp;
    m_pPartConstructor->getReady(pMiddle,m_pPartConverter,m_pMItoMB_Bilinear,margSupp, OAmargSupp,solver);
    
    // add to pMiddle any additional constraints provided by m_pPartConstructor (in adaptive constructor)
    for (PartitionConstructorIF::addconstraints_iterator cit = m_pPartConstructor->ACbegin(); cit != m_pPartConstructor->ACend(); cit++)
        pMiddle->add_constraint(*cit);
    
    // create translation map for the breakpoint variables (if any)
    // w_i
    ROCPPconstdvContainer_Ptr bpdvs( m_pPartConstructor->getBPDVContainer() );// );ROCPPdvContainer_Ptr( new dvContainer() )
    map<string,ROCPPExpr_Ptr>  BPDVTranslationMap;
    vector<ROCPPConstraint_Ptr> BPtoAdd;
    m_pBPA->createTranslationMap(*bpdvs,BPDVTranslationMap,BPtoAdd);
    
    for (vector<ROCPPConstraint_Ptr>::const_iterator cit = BPtoAdd.begin(); cit != BPtoAdd.end(); cit++)
        pOut->add_constraint(*cit);
    
    // create one to one converter using BPDVTranslationMap
    ROCPPO2EVarConverter_Ptr pO2OBPDVS( ROCPPO2EVarConverter_Ptr( new PredefO2EVariableConverter(BPDVTranslationMap) ) );
    pO2OBPDVS->createInverseMap(*bpdvs);
    
    vector<ROCPPConstraint_Ptr> vecNACs;
    createVariableMap(pModel,pMiddle,vecNACs);
    
    map<string, ROCPPVarIF_Ptr> inverseVarMapAll;
    
    // only for stochastic
    
    double allArea(1.0);
    map<string, pair<double, double> > allMap;
    
    if (pIn->getObjType() == stochastic) {
        findWholeMarginalSupport(pIn, m_pPartConstructor->getNumPartitionsMap(), allMap);
        
        map<string, pair<double, double> >::const_iterator m_it(margSupp.begin());
        for(; m_it != margSupp.end(); m_it++)
            allMap.insert((*m_it));
        
        allArea = calculateArea(allMap);
    }
    
    
    // add the constraints assossiated with each subset of the partition
    uint partitionsCnt(1);
    
    ROCPPExpr_Ptr newObjExpr( new ROCPPExpr() );
    
    for (PartitionConstructorIF::const_iterator pit = m_pPartConstructor->begin(); pit != m_pPartConstructor->end(); pit++, partitionsCnt++)
    {
        
        // get the variable converter
        map<string,ROCPPVarIF_Ptr> mapFromOldToNewVar(m_VariableMap[ (*pit).first ]);
        
        // get the uncertainty converter
        //map<string,ROCPPUnc_Ptr> mapFromOldToNewUnc(m_pPartConstructor->getMapFromOriginalUncToSubsetUnc((*pit).first));
        
        // add to it the uncertain constraints of pMiddle after mapping the variables and the uncertain parameters
        for (OptimizationModelIF::constraintIterator cit = pMiddle->constraintBegin(); cit != pMiddle->constraintEnd(); cit++)
        {
            //if ( !(*cit)->isDeterministic() )
            pOut->add_constraint( (*cit)->mapVars(mapFromOldToNewVar), pit->first);
        }
        
//        // add to the problem the constraints specific to this partition (mapping the breakpoint variables)
//        for (PartitionConstructorIF::usconstraints_iterator cit = m_pPartConstructor->USCbegin( (*pit).first ); cit != m_pPartConstructor->USCend((*pit).first ); cit++)
//            pOut->add_constraint( (*cit)->mapVars(mapFromOldToNewVar), pit->first );
        
        
        // map the variables to variables over this partition
        // Y_(ij) -> Y_(ij)^s
        
        
        //inverseVarMapAll.insert(pO2OVC->beginInv(), pO2OVC->endInv() );
        
        // approximate real-valued variables affecting the uncertainty set
        //ROCPPOptModelIF_Ptr pTmp3( m_pUSRVA->convertVar(pTmp2) );
        
        // convert problem to uncertain single-stage problem (no bilinearities)
        //ROCPPUncSSOptModel_Ptr pTmp4( convertToUSSOM(pTmp3) );
        
        if (pIn->getObjType() == stochastic)
        {
            ROCPPObjectiveIF_Ptr oldObj(pMiddle->getObj());
            pair<double, map<string, ROCPPExpr_Ptr> > meanAndProb(calculateMeanAndProb(pMiddle, pit->first, allMap, allArea));
            newObjExpr->add( getStochasticObj(meanAndProb, oldObj, pit->first) );
            
        }
        
    }
    
    if (pIn->getObjType() == robust)
        pOut->set_objective( pMiddle->getObj() );
    else
    {
        ROCPPObjectiveIF_Ptr newObj (new ROCPPSimpleObjective(newObjExpr));
        pOut->set_objective(newObj);
        
//        if(newObj.size() == 0)
//            throw MyException("The objective size should not be 0");
//        
//        if(newObj.size() == 1)
//        {
//            ROCPPObjectiveIF_Ptr objToSet( new SimpleObjective(newObj[0] ) );
//            pOut->set_objective(objToSet);
//        }
//        else
//        {
//            ROCPPObjectiveIF_Ptr objToSet( new MaxObjective(newObj) );
//            pOut->set_objective(objToSet);
//        }

    }
    
//    pOutTmp->set_ddu(pMiddle);
//
//    // Eliminate integer terms appearing in bilinearities
//    ROCPPOptModelIF_Ptr pOutTmp2( m_pMItoMB_Bilinear->convertVar( ROCPPOptModelIF_Ptr(pOutTmp) ) );
//
//    // then, eliminate bilinearities between binary and other terms
//    ROCPPOptModelIF_Ptr pOutTmp3( m_pBTR->linearize(pOutTmp2) );
//
//    // convert resulting problem to MISOCP
//    ROCPPMISOCP_Ptr pOut( convertToMISOCP(pOutTmp3) );
    
    if(pIn->isMultiStageOptModelDDID() )
    {
        ROCPPOptModelDDID_Ptr pIn_DDU( static_pointer_cast<MultiStageOptModelDDID>(pIn));
        
        ROCPPLinearDR_Ptr pLDR(static_pointer_cast<LinearDecisionRule>(m_pCVA));
        
        // ---------------- DECISION-DEPENDENT NON-ANTICIPATIVITY CONSTRAINTS -----------------------------------------------
        
        // |Y_{t,ij}^p| <= M x_{t-1,j}^p \forall i,j,p,t
        for (PartitionConstructorIF::const_iterator mp_it=m_pPartConstructor->begin(); mp_it!=m_pPartConstructor->end(); mp_it++)
        {
            for (OneToExprVariableConverterIF::const_iterator tmldr_it=pLDR->begin(); tmldr_it!=pLDR->end(); tmldr_it++)
            {
                ROCPPVarIF_Ptr odv( pIn_DDU->getVar( tmldr_it->first ) );//Y_{t,i}
                
                for (MultiStageOptModelDDID::dduIterator ddu_it = pIn_DDU->dduBegin(); ddu_it != pIn_DDU->dduEnd(); ddu_it++)
                {
                    ROCPPVarIF_Ptr mv( pIn_DDU->getMeasVar(ddu_it->first,odv->getTimeStage()-1) );//original x_{t-1, j}
                    ROCPPVarIF_Ptr mvp( getVarOnPartition(mp_it->first, mv->getName()) );//x_{t-1, j}^p
                    
                    ROCPPVarIF_Ptr ldrCoeff ( pLDR->getCoeffDV( odv->getName(),ddu_it->second->getName()) );//original Y_{t, ij}
                    ROCPPVarIF_Ptr ldrCoeffp( getVarOnPartition(mp_it->first, ldrCoeff->getName()) );//Y_{t, ij}^p
                    
                    // add non-anticipativity constraints
                    ROCPPConstraint_Ptr pConstraint1( new IneqConstraint(false,true) );
                    pConstraint1->add_lhs(1.,ldrCoeffp);
                    pConstraint1->add_lhs(-1.*m_bigM,mvp);
                    pConstraint1->set_rhs(make_pair(0.,true));
                    pOut->add_constraint(pConstraint1);
                    
                    ROCPPConstraint_Ptr pConstraint2( new IneqConstraint(false,true) );
                    pConstraint2->add_lhs(-1.,ldrCoeffp);
                    pConstraint2->add_lhs(-1.*m_bigM,mvp);
                    pConstraint2->set_rhs(make_pair(0.,true));
                    pOut->add_constraint(pConstraint2);
                }
            }
        }
        
        // check that first stage variables are not adaptive
        for (UncertainOptimizationModel::varsIterator v_it=pIn_DDU->varsBegin(); v_it!=pIn_DDU->varsEnd(); v_it++)
        {
            if ( (v_it->second->getTimeStage() == 1) && (v_it->second->isAdaptive()) )
                throw MyException("the adaptive variables should have a time stage >1");
        }
        
        // | x_{t,l}^q - x_{t,l}^p | <= x_{t-1,j}^p \forall q \in P_j(p), \forall p: p_j=1, \forall l \neq j, \forall j,p,t
        // | Y_{t,il}^q - Y_{t,il}^p | <= Mx_{t-1,j}^p \forall i \forall q \in P_j(p), \forall p: p_j=1, \forall l \neq j, \forall j,p,t
        
        for (MultiStageOptModelDDID::dduIterator ddu_it=pIn_DDU->dduBegin(); ddu_it != pIn_DDU->dduEnd(); ddu_it++)
        {
            uint loc(pIn_DDU->getObservableAlphabeticalLocation(ddu_it->second));

            //pIn_DDU->getFirstStageObservable(ddu_it->first)
            for (uint t=1; t<pIn_DDU->getNumTimeStages(); t++)
            {
                ROCPPVarIF_Ptr mv( pIn_DDU->getMeasVar(ddu_it->first,t) );// original x_{t, j}
                for (PartitionConstructorIF::const_iterator mp_it=m_pPartConstructor->begin(); mp_it!=m_pPartConstructor->end(); mp_it++)
                {
                    ROCPPVarIF_Ptr mvp( getVarOnPartition(mp_it->first, mv->getName()) );//x_{t, j}^p
                    
                    string deb(m_pPartConverter->convertPartitionToString(1));
                    
                    int tmp( (mp_it->first).compare(loc*m_pPartConverter->getNumEls(),m_pPartConverter->getNumEls(),m_pPartConverter->convertPartitionToString(1)) );
                    
                    tmp += 0;
                    
                    if(tmp == 0) //find the first(basic) partition
                    {
                        PartitionConstructorIF::const_iterator mpi_it(m_pPartConstructor->begin());
                        for (;mpi_it!=m_pPartConstructor->end();mpi_it++)
                        {
                            string str1 = (mp_it->first);
                            str1.erase(loc*m_pPartConverter->getNumEls(),m_pPartConverter->getNumEls());
                            string str2 = (mpi_it->first);
                            str2.erase(loc*m_pPartConverter->getNumEls(),m_pPartConverter->getNumEls());
                            
                            if ( (str1==str2) && ( (mp_it->first)!= (mpi_it->first) ) )// p(-1) = p(-j)
                            {
                                // iterate through all decision variables on which an LDR was performed
                                for (OneToExprVariableConverterIF::const_iterator tmldr_it=pLDR->begin(); tmldr_it!=pLDR->end(); tmldr_it++)
                                {
                                    // find the original variable
                                    ROCPPVarIF_Ptr odv( pIn_DDU->getVar( tmldr_it->first ) );
                                    
                                    if ((odv->getTimeStage() == (mv->getTimeStage()+1) ))
                                    {
                                        // iterate through the variables that are used in the expression for the decision rule
                                        for (LHSExpression::dvIterator coeff_it=tmldr_it->second->varsBegin(); coeff_it!=tmldr_it->second->varsEnd(); coeff_it++)
                                        {
                                            // get coefficient of vars on each of subsets of partition
                                            ROCPPVarIF_Ptr ldrCoeff ( coeff_it->second );
                                            ROCPPVarIF_Ptr ldrCoeffp1( getVarOnPartition(mp_it->first, ldrCoeff->getName()) );
                                            ROCPPVarIF_Ptr ldrCoeffp2( getVarOnPartition(mpi_it->first, ldrCoeff->getName()) );
                                            
                                            // for each coefficient, add NACs
                                            ROCPPConstraint_Ptr pConstraint1( new IneqConstraint(false,true) );
                                            pConstraint1->add_lhs(1.,ldrCoeffp1);
                                            pConstraint1->add_lhs(-1.,ldrCoeffp2);
                                            pConstraint1->add_lhs(-1.*m_bigM,mvp);
                                            pConstraint1->set_rhs(make_pair(0.,true));
                                            pOut->add_constraint(pConstraint1);
                                            
                                            ROCPPConstraint_Ptr pConstraint2( new IneqConstraint(false,true) );
                                            pConstraint2->add_lhs(1.,ldrCoeffp2);
                                            pConstraint2->add_lhs(-1.,ldrCoeffp1);
                                            pConstraint2->add_lhs(-1.*m_bigM,mvp);
                                            pConstraint2->set_rhs(make_pair(0.,true));
                                            pOut->add_constraint(pConstraint2);
                                        }
                                    }
                                }
                                // iterate through all the decision variables on which a PWC DR was performed
                                for (OneToOneVariableConverterIF::const_iterator tmcdr_it=m_pDVA->begin(); tmcdr_it!=m_pDVA->end(); tmcdr_it++)
                                {
                                    // find the original variable
                                    ROCPPVarIF_Ptr odv( pIn_DDU->getVar( tmcdr_it->first ) );
                                    
                                    if ((odv->getTimeStage() == (mv->getTimeStage()+1) ))
                                    {
                                        // get coefficient of vars on each of subsets of partition
                                        ROCPPVarIF_Ptr cdrCoeff ( m_pDVA->find( odv->getName() )->second );
                                        ROCPPVarIF_Ptr cdrCoeffp1( getVarOnPartition(mp_it->first, cdrCoeff->getName()) );
                                        ROCPPVarIF_Ptr cdrCoeffp2( getVarOnPartition(mpi_it->first, cdrCoeff->getName()) );
                                        
                                        ROCPPConstraint_Ptr pConstraint1( new IneqConstraint(false,true) );
                                        pConstraint1->add_lhs(1.,cdrCoeffp1);
                                        pConstraint1->add_lhs(-1.,cdrCoeffp2);
                                        if (odv->getType()==intDV)
                                            pConstraint1->add_lhs(-1.*m_bigM,mvp);
                                        else
                                            pConstraint1->add_lhs(-1.,mvp);
                                        pConstraint1->set_rhs(make_pair(0.,true));
                                        pOut->add_constraint(pConstraint1);
                                        
                                        ROCPPConstraint_Ptr pConstraint2( new IneqConstraint(false,true) );
                                        pConstraint2->add_lhs(1.,cdrCoeffp2);
                                        pConstraint2->add_lhs(-1.,cdrCoeffp1);
                                        if (odv->getType()==intDV)
                                            pConstraint2->add_lhs(-1.*m_bigM,mvp);
                                        else
                                            pConstraint2->add_lhs(-1.,mvp);
                                        pConstraint2->set_rhs(make_pair(0.,true));
                                        pOut->add_constraint(pConstraint2);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    pOut->add_constraints(vecNACs.begin(),vecNACs.end());
    
    
    auto stop = chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<chrono::seconds>(stop - start);
    
    cout << endl;
    cout << "Total time to approximate: " << duration.count() << " seconds" << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    return pOut;
}



pair<double, map<string, ROCPPExpr_Ptr> > PiecewiseDecisionRule::calculateMeanAndProb(const ROCPPOptModelIF_Ptr pModel, string partition, const map<string,pair<double,double> > &allMap, double allArea)
{
    map<string, ROCPPExpr_Ptr> mapFromUncToMean;
    double prob(1.0);
    
    map<string,pair<double,double> >::const_iterator u_it = allMap.begin();
    
    for(; u_it != allMap.end(); u_it++)
    {
        ROCPPExpr_Ptr mean(new LHSExpression());
        ROCPPExpr_Ptr range(new LHSExpression());
        
        string uncNme(u_it->first);
        uint pos(m_pPartConstructor->getPos(partition, uncNme));
        
        if (pos == 1)
        {
            ROCPPExpr_Ptr lb(new LHSExpression());
            lb->add(allMap.find(uncNme)->second.first);
            mean->add(0.5, lb);
            range->add(-1.0, lb);
        }
        else{
            mean->add(0.5, m_pPartConstructor->getBp(make_pair(uncNme, pos-1)));
            range->add(-1.0, m_pPartConstructor->getBp(make_pair(uncNme, pos-1)));
        }
        
        if ( (!m_pPartConstructor->hasPartition(uncNme)) || (m_pPartConstructor->getNumSubsets(uncNme) == pos))
        {
            ROCPPExpr_Ptr ub(new LHSExpression());
            ub->add(allMap.find(uncNme)->second.second);
            mean->add(0.5, ub);
            range->add(1.0, ub);
        }
        else
        {
            mean->add(0.5, m_pPartConstructor->getBp(make_pair(uncNme, pos)));
            range->add(1.0, m_pPartConstructor->getBp(make_pair(uncNme, pos)));
        }
        
        if (range->getNumVars() != 0)
            throw MyException("Can not deal with the adaptive break points setting now");
        
        prob *= range->getSumConstantTerms();
        
        mapFromUncToMean.insert(make_pair(uncNme, mean));
        
    }
    
    
    prob = prob/allArea;
    
    return make_pair(prob, mapFromUncToMean);
}

ROCPPExpr_Ptr PiecewiseDecisionRule::getStochasticObj(const pair<double, map<string, ROCPPExpr_Ptr> > &meanAndProb, ROCPPObjectiveIF_Ptr oldObj, string partitionSubsetNme)
{
    if (!(oldObj->getObjType() == simpleObj))
        throw MyException("Objective function cannot be nonlinear here; linearize first");
    
    
    ROCPPSimpleObjective_Ptr oldObjSimple( static_pointer_cast<ROCPPSimpleObjective>(oldObj) );
    
    ROCPPExpr_Ptr oldObjSimpleExpr(oldObjSimple->getSimpleObj());
    
    double probability(meanAndProb.first);
    map<string, ROCPPExpr_Ptr> mapFromUncToMean(meanAndProb.second);
    
    
    ROCPPExpr_Ptr outObjExp (new ROCPPExpr());
    
    // get the variable converter
    map<string,ROCPPVarIF_Ptr> mapFromOldToNewVar(m_VariableMap[ partitionSubsetNme ]);
    
    outObjExp->add( probability * oldObjSimpleExpr->mapExprVars(mapFromOldToNewVar)->mapUncs(mapFromUncToMean) );
    
    
//    ROCPPExpr_Ptr partObj(oldObjExpr->mapUncs(mapFromUncToMean));
//    map<string,ROCPPVarIF_Ptr> mapVar;
//
//    dvMapType::const_iterator v_it(partObj->varsBegin());
//    for ( ; v_it != partObj->varsEnd(); v_it++)
//    {
//        if (newObj[numObj-1]->varIsInvolved(v_it->second))
//            mapVar.insert(make_pair(v_it->first, newObj[numObj-1]->getVar(v_it->first)));
//    }
//
//    ROCPPExpr_Ptr objToAdd(partObj->mapExprVars(mapVar));
//    newObj[numObj-1]->add(probability, objToAdd);
    
    return outObjExp;
}


void PiecewiseDecisionRule::createVariableMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pMiddle, vector<ROCPPConstraint_Ptr>& vecNACs)
{

    // iterate through the variables in pMiddle
    for (OptimizationModelIF::varsIterator vit = pMiddle->varsBegin(); vit != pMiddle->varsEnd(); vit++)
    {
        
        if (vit->second->isAdaptive())
            throw MyException("should not have any adaptive variables left");
        
        // -- find time stage of variable that this variable parameterizes (this is for the decision independent non-anticipativity constraints)
        
        uint varTS(0);
        
        
        ROCPPVarIF_Ptr dv;
        
        string bla(vit->second->getName());
        
        pair<bool,ROCPPVarIF_Ptr> tmpdvpair( findOrigVariable(vit->second) );
        
        
        if (tmpdvpair.first==true)
        {
            dv = tmpdvpair.second;
            varTS = dv->getTimeStage();
        }
        else
        {
            bool isdef( pIn->varIsDefined( vit->second->getName() ) );
            if (isdef)
            {
                dv = pIn->getVar(vit->second->getName());
                varTS = dv->getTimeStage();
                if (varTS!=1)
                    throw MyException("why is varTS not equal to 1?");
            }
            else
                throw MyException("where does this variable " + vit->second->getName() + " come from?");
        }
        
        // iterate through the partitions
        for (PartitionConstructorIF::const_iterator pit = m_pPartConstructor->begin(); pit != m_pPartConstructor->end(); pit++)
        {
            
            string basicPartition( m_pPartConverter->getBasicPartition( pit->second, varTS, pIn , m_pCVA->getMemory() ) );
            
            //to avoid having the same variable defined over different partitions (unless we want explicit NACs)
            
            ROCPPVarIF_Ptr c_var;
            
            if ( !dv->isAdaptive() )
            {
                c_var = vit->second;
            }
            else
            {
                if ( (basicPartition==(pit->first)) || (m_useExplicitNACs) ) // this partition is a basic partition
                {
                    if (vit->second->getType()==contDV)
                        c_var = ROCPPVarIF_Ptr(new VariableDouble( vit->second->getName()+"_"+((*pit).first), vit->second->getLB(), vit->second->getUB()));
                    else if (vit->second->getType() == intDV)
                        c_var = ROCPPVarIF_Ptr(new VariableInt( vit->second->getName()+"_"+((*pit).first), vit->second->getLB(), vit->second->getUB()));
                    else if (vit->second->getType() == boolDV)
                        c_var = ROCPPVarIF_Ptr(new VariableBool( vit->second->getName()+"_"+((*pit).first), vit->second->getLB(), vit->second->getUB()));
                    
                    
                    // if we want explicit NACs, add them
                    if ( (m_useExplicitNACs) && (basicPartition!=(pit->first)) )
                    {
                        map<string, map<string,ROCPPVarIF_Ptr> >::const_iterator tmpit (m_VariableMap.find(basicPartition) );
                        if (tmpit==m_VariableMap.end())
                            throw MyException("this should not be happening: is the basic partition >= current partition?");
                        
                        map<string,ROCPPVarIF_Ptr>::const_iterator fit ( tmpit->second.find( vit->second->getName() ) );
                        
                        if (fit==tmpit->second.end())
                            throw MyException("unexpected error - variable not found");
                        
                        ROCPPClassicConstraint_Ptr pCstr( new EqConstraint(false,true) );
                        
                        pCstr->add_lhs(1., fit->second);
                        pCstr->add_lhs(-1., c_var);
                        pCstr->set_rhs(make_pair(0.,true));
                        
                        vecNACs.push_back(pCstr);
                    }
                }
                else
                {
                    map<string, map<string,ROCPPVarIF_Ptr> >::const_iterator tmpit (m_VariableMap.find(basicPartition) );
                    if (tmpit==m_VariableMap.end())
                        throw MyException("this should not be happening: is the basic partition >= current partition?");
                    
                    map<string,ROCPPVarIF_Ptr>::const_iterator fit ( tmpit->second.find( vit->second->getName() ) );
                    
                    if (fit==tmpit->second.end())
                        throw MyException("unexpected error - variable not found");
                    
                    c_var = fit->second;
                }
            }
            m_VariableMap[(*pit).first].insert( make_pair (vit->second->getName(), c_var) );
        }
    }
}





//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string PiecewiseDecisionRule::getSolutionApproachParameters(string delimiter) const
{
    string out("Prepartition");
    out += delimiter;
    out += m_numPartitionsStr;
    out += delimiter;
    
    return out;
}

pair<bool,ROCPPVarIF_Ptr> PiecewiseDecisionRule::findOrigVariable(ROCPPVarIF_Ptr newdv) const
{
    // 1. try to find it in the reverse map of ldr
    ROCPPVarIF_Ptr dv;
    bool found(false);
    
    VariableConverterIF::const_iterator_inv it_ldr ( m_pCVA->findInv ( newdv->getName() ) );
    if (it_ldr != m_pCVA->endInv() )
    {
        dv = it_ldr->second;
        found = true;
        //varTS = dv->getTimeStage();
    }
    else
    {
        // 2. if not found, try to find it in the reverse map of cdr
        VariableConverterIF::const_iterator_inv it_cdr ( m_pDVA->findInv ( newdv->getName() ) );
        if (it_cdr != m_pDVA->endInv() )
        {
            dv = it_cdr->second;
            found = true;
        }
    }
    
    return make_pair(found,dv);
}

ROCPPVarIF_Ptr PiecewiseDecisionRule::getVarOnPartition(string partition, string origVarName) const
{
    
    map<string, map<string,ROCPPVarIF_Ptr> >::const_iterator it (m_VariableMap.find(partition));
    if (it==m_VariableMap.end())
        throw MyException("subset of partition " + partition + " not found in map");
    
    map<string,ROCPPVarIF_Ptr>::const_iterator subit(it->second.find(origVarName));
    
    if (subit==it->second.end())
        throw MyException("decision variable " + origVarName + " not found on subset " + partition + " of partition");
    
    return subit->second;
}

//???: Question here
void PiecewiseDecisionRule::getWsSolutions(ROCPPOptModelIF_Ptr pIn, const map<string, double> &warmStartResults, const map<string,uint> &wsMap, map<string, double> &wsSolutions)
{
    if ( (m_pPartConstructor->getNumSubsets()!=1) && (pIn->getNumBoolVars()) )
    {
        
        for (map<string, map<string,ROCPPVarIF_Ptr> >::const_iterator p_it = m_VariableMap.begin(); p_it != m_VariableMap.end(); p_it++)
        {
            
            uint cnt(0);
            string mappedPartition;
            
            for (UncertainOptimizationModel::uncertaintiesIterator u_it = pIn->uncertaintiesBegin(); u_it!=pIn->uncertaintiesEnd(); u_it++)
            {
                if ( u_it->second->isObservable() )
                {
                    // find the maximum number of partitions in the current partitions and in the warm start partitions
                    uint cpart = m_pPartConstructor->getNumSubsets(u_it->second->getName());
                    map<string,uint>::const_iterator tmp_it( wsMap.find(u_it->second->getName()) );
                    
                    uint mpart = tmp_it->second;
                    if (cpart==mpart)
                        mappedPartition += p_it->first.substr(cnt*m_pPartConverter->getNumEls(),m_pPartConverter->getNumEls());
                    else
                    {
                        if ((static_cast<int>(cpart) % static_cast<int>(mpart)) != 0)
                            throw MyException("unusable warm start");
                        
                        int div = static_cast<int>(cpart) / static_cast<int>(mpart);
                        //double tmp = lexical_cast<double>(p_it->first.substr(cnt*m_pPartConverter->getNumEls(),m_pPartConverter->getNumEls()));
                        double tmp = stod(p_it->first.substr(cnt*m_pPartConverter->getNumEls(),m_pPartConverter->getNumEls()));
                        double cEl = ceil( tmp / static_cast<double>(div) );
                        mappedPartition+=m_pPartConverter->convertPartitionToString(static_cast<uint>(cEl));
                    }
                    cnt++;
                }
            }
            
            
            for (map<string,ROCPPVarIF_Ptr>::const_iterator v_it = p_it->second.begin(); v_it != p_it->second.end(); v_it++)
            {
                // find the mappedPartition in m_variableMap
                map< string , map<string, ROCPPVarIF_Ptr> >::const_iterator mapped_it = m_VariableMap.find( mappedPartition );
                
                if (mapped_it==m_VariableMap.end())
                    throw MyException("mappedPartition not found");
                
                // find the name of the variable in mapped partition
                
                map<string, ROCPPVarIF_Ptr>::const_iterator mv_it ( mapped_it->second.find(v_it->first) );
                
                if (mv_it == mapped_it->second.end())
                    throw MyException("variable not found in submap of m_VariableMap");
                
                map<string, double>::const_iterator r_it(warmStartResults.find(mv_it->second->getName() ) );
                
                if (r_it == warmStartResults.end())
                    throw MyException("variable not found in warm start results");
                
                wsSolutions.insert( make_pair(v_it->second->getName(), r_it->second) );
                
            }
        }
    }
    else{
        cout << "Static or LP model will return empty map";
    }
}

void PiecewiseDecisionRule::calculateSolution(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv, string partition)
{
    map<string, ROCPPVarIF_Ptr> variableOnPartition = m_VariableMap.find(partition)->second;
    
    map<string, double> variableValue;
    map<string, ROCPPVarIF_Ptr>::const_iterator variable = variableOnPartition.begin();
    map<string, double>::const_iterator result;
    
    for(; variable != variableOnPartition.end(); variable++)
    {
        result = resultIn.find(variable->second->getName());
        if(result == resultIn.end())
            throw MyException("No such variable: " + variable->second->getName());
        
        variableValue.insert(make_pair(variable->first, result->second));
    }
    
    string dvName = dv->getName();
    if(dv->getType() == contDV)
    {
        cout << "On subset " << partition << ": ";
        m_pCVA->printOut(pIn, variableValue, dv);
    }
    else
    {
        cout << "On subset " << partition << ": ";
        m_pDVA->printOut(pIn, variableValue, dv);
    }
}

void PiecewiseDecisionRule::calculateSolution(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc, string partition)
{
    map<string, ROCPPVarIF_Ptr> variableOnPartition = m_VariableMap.find(partition)->second;
    
    map<string, double> variableValue;
    map<string, ROCPPVarIF_Ptr>::const_iterator variable = variableOnPartition.begin();
    map<string, double>::const_iterator result;
    
    for(; variable != variableOnPartition.end(); variable++)
    {
        result = resultIn.find(variable->second->getName());
        if(result == resultIn.end())
            throw MyException("No such variable: " + variable->second->getName());
        
        variableValue.insert(make_pair(variable->first, result->second));
    }
    
    uint t;
    string name = unc->getName();
    
    ROCPPVarIF_Ptr meas;
    double value;
    
    for(t = 1; t < pIn->getNumTimeStages(); t++)
    {
        meas = pIn->getMeasVar(name, t);
        value = variableValue.find(meas->getName())->second;
        if(value > 0.5){
            cout << "Uncertain parameter " << name <<" on subset " << partition << " is observed at stage " << t << endl;
            return;
        }
    }
    
    cout << "Uncertain parameter " << name <<" on subset " << partition << " is never observed" << endl;
    
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void PiecewiseDecisionRule::printParametersToScreen() const
{
    cout << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "========================== APPROXIMATION PARAMS =========================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    
    cout << "Solution Approach: Prepartitioning" << endl;
    cout << "Breakpoint Configuration: " << m_numPartitionsStr << endl;
    
    cout << "=========================================================================== " << endl;
    cout << endl;
}

void PiecewiseDecisionRule::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv, map<ROCPPUnc_Ptr, uint> partitionIn)
{
    map<string, uint> stringPartition;
    
    map<ROCPPUnc_Ptr, uint>::const_iterator tmp = partitionIn.begin();
    for(;tmp != partitionIn.end(); tmp++)
        stringPartition.insert(make_pair(tmp->first->getName(), tmp->second));
    
    string partition = m_pPartConverter->convertPartitionToString(stringPartition, pIn);
    
    calculateSolution(pIn, resultIn, dv, partition);
}

void PiecewiseDecisionRule::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv)
{
    map<string, map<string,ROCPPVarIF_Ptr> >::const_iterator partitionIn;
    for(partitionIn = m_VariableMap.begin(); partitionIn != m_VariableMap.end(); partitionIn++)
    {
        calculateSolution(pIn, resultIn, dv, partitionIn->first);
    }
}

void PiecewiseDecisionRule::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc, map<ROCPPUnc_Ptr, uint> partitionIn)
{
    map<string, uint> stringPartition;
    
    map<ROCPPUnc_Ptr, uint>::const_iterator tmp = partitionIn.begin();
    for(;tmp != partitionIn.end(); tmp++)
        stringPartition.insert(make_pair(tmp->first->getName(), tmp->second));
    
    string partition = m_pPartConverter->convertPartitionToString(stringPartition, pIn);
    
    calculateSolution(pIn, resultIn, unc, partition);
}

void PiecewiseDecisionRule::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc)
{
    map<string, map<string,ROCPPVarIF_Ptr> >::const_iterator partitionIn;
    for(partitionIn = m_VariableMap.begin(); partitionIn != m_VariableMap.end(); partitionIn++)
    {
        calculateSolution(pIn, resultIn, unc, partitionIn->first);
    }
}
