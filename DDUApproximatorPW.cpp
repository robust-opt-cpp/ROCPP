//
//  DDUApproximatorPW.cpp
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
#include "OptModelConverters.hpp"
#include "RobustifyEngine.hpp"
#include "helpersOpt.hpp"
#include "DecisionRule.hpp"
#include "FileRelations.hpp"
#include "DDUApproximator.hpp"
#include "DDUApproximatorPW.hpp"
#include "IndexSetCreator.hpp"
#include <iomanip>
#include <chrono>


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% PIECEWISE APPROXIMATOR %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//PiecewiseApproximator::PiecewiseApproximator(ROCPPOptModelIF_Ptr pIn, string numPartitionsStr, string wsPartitionsStr, uint numBits, double bigM, bool useExplicitNACs, string folder) : m_bigM(bigM), m_useExplicitNACs(useExplicitNACs), m_folder(folder)
PiecewiseApproximator::PiecewiseApproximator(ROCPPOptModelIF_Ptr pIn, string numPartitionsStr, uint numBits, double bigM, bool useExplicitNACs, string folder) : m_bigM(bigM), m_useExplicitNACs(useExplicitNACs), m_folder(folder)
{
    
    // ==============================================================================================
    // ============================== BUILD PARTITION MAPS ==========================================
    // ==============================================================================================
    
    map<string,uint> numPartitionsMap;
    
    uint ccnt(0);
    string dummyPartition;
    
    if (pIn->isUncertainOptimizationModel())
    {
        ROCPPUncOptModel_Ptr pModelUnc = static_pointer_cast<UncertainOptimizationModel>(pIn);
        
        u_long remainderNumPartitionStr = numPartitionsStr.size() % pIn->getUncContainer()->getNumObsUncertainties();

          if ( (remainderNumPartitionStr != 0) )
            throw MyException("your partition enconding is incorrect; both the partitions string and the warm start partitions string must have a length that is a multiple of the number of uncertain parameters in the problem");
        
        u_long numElsNumPartitionStr = numPartitionsStr.size() / pIn->getUncContainer()->getNumObsUncertainties();

        ROCPPParConverter_Ptr pPartConverter (new PartitionConverter((uint)(numElsNumPartitionStr) ));
        
        
        // original parameters
        for (UncertainOptimizationModel::uncertaintiesIterator u_it=pModelUnc->uncertaintiesBegin(); u_it!=pModelUnc->uncertaintiesEnd(); u_it++)
        {
            if ( u_it->second->isObservable() )
            {
                string str1(numPartitionsStr.substr(ccnt*pPartConverter->getNumEls(),pPartConverter->getNumEls()));
                numPartitionsMap.insert(pair<string,uint>(u_it->first,stoi(str1)));

                ccnt++;
                dummyPartition+=pPartConverter->convertPartitionToString(1);
            }
            else
            {
                numPartitionsMap.insert(pair<string,uint>(u_it->first,1));
            }
        }
        
        if (numPartitionsStr.length()!=dummyPartition.length())
            throw MyException("numPartitionsStr and dummyPartition should have the same length");
    }
    
    InitializeMe(pIn,numPartitionsMap,numBits,bigM,useExplicitNACs,folder);
}


PiecewiseApproximator::PiecewiseApproximator(ROCPPOptModelIF_Ptr pIn, const map<string,uint> &numPartitionsMap, uint numBits, double bigM, bool useExplicitNACs, string folder)  : m_bigM(bigM), m_useExplicitNACs(useExplicitNACs), m_folder(folder)
{
    map<string, uint>::const_iterator pMap;
    
    map<string, uint> numMap;
    
    for (UncertainOptimizationModel::uncertaintiesIterator u_it=pIn->uncertaintiesBegin(); u_it!=pIn->uncertaintiesEnd(); u_it++)
    {
        pMap = numPartitionsMap.find(u_it->second->getName());
        
        if ( u_it->second->isObservable() && pMap != numPartitionsMap.end())
            numMap.insert(pair<string,uint>(pMap->first,pMap->second));
        else
            numMap.insert(pair<string,uint>(u_it->first,1));
    }
    
    InitializeMe(pIn, numMap,numBits,bigM,useExplicitNACs,folder);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void PiecewiseApproximator::InitializeMe(ROCPPOptModelIF_Ptr pIn, const map<string,uint> &numPartitionsMap, uint numBits, double bigM, bool useExplicitNACs, string folder)
{
    
    m_bigM=bigM;
    m_numBits=numBits;
    m_useExplicitNACs=useExplicitNACs;
    m_folder=folder;
    // partition constructor
    
    m_pPartConstructor = ROCPPParConstructor_Ptr(new StaticPartitionConstructor(numPartitionsMap));
    
    // decision rule approximators
    
    // do DDU appoximation using piecewise constant decision rule
    m_pCVA = ROCPPContinuousVarsDR_Ptr(new LinearDecisionRule());
    m_pDVA = ROCPPDiscreteVarsDR_Ptr(new ConstantDecisionRule());
    
    // -- partition related --
    
    // find the maximum value inside partitionsMap -- this will determine the number of elements needed in PartitionConverter
    
    map<string,uint>::const_iterator mit(max_element(numPartitionsMap.begin(), numPartitionsMap.end(),
                                                     [] (const std::pair<string,uint>& a, const std::pair<string,uint>& b)->bool{ return a.second < b.second; }));
    
    m_pPartConverter = ROCPPParConverter_Ptr(new PartitionConverter( (to_string(mit->second)).size() ) );
    m_numPartitionsStr = m_pPartConverter->convertPartitionToString(numPartitionsMap, pIn);
    
    
    // ******************** MI to MB converter ******************************************************
    
    m_pMItoMB_Bilinear = ROCPPMItoMB_Ptr(new BinaryConverter());
    
    // ******************** bilinear term reformulation ******************************************************
    m_pBTR = ROCPPBilinearReform_Ptr(new BTR_bigM());
    
    
    // ******************* uncertainty set real var approximator ***********************************
    m_pUSRVA = ROCPPUncSetRealVarApprox_Ptr( new UncertaintySetRealVarApproximator(numBits));
    m_pBPA = ROCPPUncSetRealVarApprox_Ptr( new UncertaintySetRealVarApproximator(0));
}

void PiecewiseApproximator::createVariableMap(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pMiddle, vector<ROCPPConstraint_Ptr >& vecNACs)
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
        
        pair<bool,ROCPPVarIF_Ptr > tmpdvpair( findOrigVariable(vit->second) );
        
        
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
                        map<string, map<string,ROCPPVarIF_Ptr > >::const_iterator tmpit (m_VariableMap.find(basicPartition) );
                        if (tmpit==m_VariableMap.end())
                            throw MyException("this should not be happening: is the basic partition >= current partition?");
                        
                        map<string,ROCPPVarIF_Ptr >::const_iterator fit ( tmpit->second.find( vit->second->getName() ) );
                        
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
                    map<string, map<string,ROCPPVarIF_Ptr > >::const_iterator tmpit (m_VariableMap.find(basicPartition) );
                    if (tmpit==m_VariableMap.end())
                        throw MyException("this should not be happening: is the basic partition >= current partition?");
                    
                    map<string,ROCPPVarIF_Ptr >::const_iterator fit ( tmpit->second.find( vit->second->getName() ) );
                    
                    if (fit==tmpit->second.end())
                        throw MyException("unexpected error - variable not found");
                    
                    c_var = fit->second;
                }
            }
            m_VariableMap[(*pit).first].insert( make_pair (vit->second->getName(), c_var) );
        }
    }
}

ROCPPMISOCP_Ptr PiecewiseApproximator::approx(ROCPPOptModelIF_Ptr pIn)
{
    
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "================== APPROXIMATING USING PREPARTITIONING ==================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;

    auto start = chrono::high_resolution_clock::now();
    
    // first write the problem in epigraph form!!!!
    
    ROCPPOptModelIF_Ptr pModel(pIn->Clone());
    
    if (pIn->getObjType() == robust)
        pModel->add_epigraph();
    else
    {
        if( ! pIn->hasRectangularUncertaintySet())
            throw MyException("Stochastice model must have rectanular uncertainty set");
        
        vector<ROCPPExpr_Ptr > objs(pIn->getObj()->getObj());
        
        vector<ROCPPExpr_Ptr >::const_iterator obj(objs.begin());
        
        for(; obj != objs.end(); obj++)
        {
            if((*obj)->hasProdsUncertainties())
                throw MyException("Can not deal with products of uncertainties at this time");
        }
    }
    
    string solver="gurobi";
    map<string,pair<double,double> > OAmargSupp;
    
    ROCPPOptModelIF_Ptr pOutTmp( new Bilinear_MISOCP() );
    
    ROCPPOptModelIF_Ptr pInNew( m_pUSRVA->convertVar(pModel,true) );
    
    
    ROCPPOptModelIF_Ptr pBTRModel;
    if (pModel->isUncertainOptimizationModel())
        pBTRModel = InstanciateModel(pInNew->getType(),pInNew->getNumTimeStages(),pInNew->getObjType());
    else
        pBTRModel = InstanciateModel(pInNew->getType(),pInNew->getNumTimeStages(),robust);
    
    for (OptimizationModelIF::constraintIterator cit = pInNew->constraintBegin(); cit != pInNew->constraintEnd(); cit++)
    {
        pBTRModel->add_constraint(*cit);
    }
    
    pBTRModel->set_objective(pInNew->getObj() );
    pBTRModel->set_ddu(pInNew);
    
    // do linear decision rule
    ROCPPOptModelIF_Ptr pLDRModel( m_pCVA->convertVar(pBTRModel, true) );
    
    // do constant decision rule
    ROCPPOptModelIF_Ptr pMiddle( m_pDVA->convertVar(pLDRModel, true) );
    
    // check that the problem does not have any more adaptive variables
    if ( pMiddle->getNumAdaptiveVars() != 0 )
        throw MyException("Adaptive variables should have been eliminated by now");
    
    // calculate marginal support of approximated model
    map<string,pair<double,double> > margSupp;
    
    // prepare the partition constructor
    // only deal with constraints
    m_pPartConstructor->getReady(pMiddle,m_pPartConverter,m_pMItoMB_Bilinear,margSupp, OAmargSupp,solver);
    
    
    // add to pMiddle any additional constraints provided by m_pPartConstructor(in adaptive constructor)
    for (PartitionConstructorIF::addconstraints_iterator cit = m_pPartConstructor->ACbegin(); cit != m_pPartConstructor->ACend(); cit++)
        pMiddle->add_constraint(*cit);
    
    // create translation map for the breakpoint variables (if any)
    // w_i
    ROCPPconstdvContainer_Ptr bpdvs( m_pPartConstructor->getBPDVContainer() );// );ROCPPdvContainer_Ptr( new dvContainer() )
    map<string,ROCPPExpr_Ptr >  BPDVTranslationMap;
    vector<ROCPPConstraint_Ptr > BPtoAdd;
    m_pBPA->createTranslationMap(*bpdvs,BPDVTranslationMap,BPtoAdd);
    
    for (vector<ROCPPConstraint_Ptr >::const_iterator cit = BPtoAdd.begin(); cit != BPtoAdd.end(); cit++)
        pOutTmp->add_constraint(*cit);
    
    // create one to one converter using BPDVTranslationMap
    ROCPPO2EVarConverter_Ptr pO2OBPDVS( ROCPPO2EVarConverter_Ptr( new PredefO2EVariableConverter(BPDVTranslationMap) ) );
    pO2OBPDVS->createInverseMap(*bpdvs);
    
    vector<ROCPPConstraint_Ptr > vecNACs;
    createVariableMap(pInNew,pMiddle,vecNACs);
    
    // robustify model on each partition
    uint dualvarscnt(0);
    uint partitionsCnt(1);
    
    map<string, ROCPPVarIF_Ptr > inverseVarMapAll;
    
    // only for stochastic
    vector<ROCPPExpr_Ptr > newObj;
    double allArea(1.0);
    map<string, pair<double, double> > allMap;
    
    if (pIn->getObjType() == stochastic) {
        findWholeMarginalSupport(pIn, m_pPartConstructor->getNumPartitionsMap(), allMap);
        
        map<string, pair<double, double> >::const_iterator m_it(margSupp.begin());
        for(; m_it != margSupp.end(); m_it++)
            allMap.insert((*m_it));
        
        allArea = calculateArea(allMap);
    }
    
    for (PartitionConstructorIF::const_iterator pit = m_pPartConstructor->begin(); pit != m_pPartConstructor->end(); pit++, partitionsCnt++)
    {
        long partCnt = m_pPartConstructor->getNumSubsets() / 20;
        
        if(partCnt < 10)
            partCnt = 1;
        
        if (partitionsCnt % partCnt == 0)
            cout << "Robustyfying constraints on partition " << partitionsCnt << " (total partitions: " << m_pPartConstructor->getNumSubsets() << ")"<< endl;
        
        // create a new optimization model
        ROCPPOptModelIF_Ptr pTmp( new UncertainSingleStageOptimizationModel() );
        
        // add to it the constraints of pMiddle that are not deterministic; for the deterministic constraints, if it is the first time we go through the loop, add them to the output directly
        for (OptimizationModelIF::constraintIterator cit = pMiddle->constraintBegin(); cit != pMiddle->constraintEnd(); cit++)
        {
            pTmp->add_constraint(*cit);
        }
        
        // add to the problem the constraints specific to this partition (mapping the breakpoint variables)
        for (PartitionConstructorIF::usconstraints_iterator cit = m_pPartConstructor->USCbegin( (*pit).first ); cit != m_pPartConstructor->USCend( (*pit).first ); cit++)
            pTmp->add_constraint( pO2OBPDVS->convertVar(*cit) );
        
        pTmp->set_objective(pMiddle->getObj() );
        pTmp->set_ddu(pMiddle);
        
        // map the variables to variables over this partition
        // Y_(ij) -> Y_(ij)^s
        ROCPPO2OVarConverter_Ptr pO2OVC( new PredefO2OVariableConverter( m_VariableMap[ (*pit).first ] ) );
        ROCPPOptModelIF_Ptr pTmp2( pO2OVC->convertVar(pTmp,true) );
        
        inverseVarMapAll.insert(pO2OVC->beginInv(), pO2OVC->endInv() );
        
        // approximate real-valued variables affecting the uncertainty set
        ROCPPOptModelIF_Ptr pTmp3( m_pUSRVA->convertVar(pTmp2) );
        
        // convert problem to uncertain single-stage problem (no bilinearities)
        ROCPPUncSSOptModel_Ptr pTmp4( convertToUSSOM(pTmp3) );
        
        if (pIn->getObjType() == stochastic)
        {
            ROCPPObjectiveIF_Ptr oldObj(pTmp4->getObj());
            pair<double, map<string, ROCPPExpr_Ptr > > meanAndProb(calculateMeanAndProb(pTmp4, pit->first, allMap, allArea));
            getStochasticObj(meanAndProb, oldObj, newObj);
        }
        
        // robustify pTmp4
        // generate the constraint with cone. robust MBLP->standard MBLP
        ROCPPRobustifyEngine_Ptr m_pRE (new RobustifyEngine(dualvarscnt,pit->first));
        ROCPPBilinMISOCP_Ptr pRobustTmp( m_pRE->robustify(pTmp4)  );
        
        for (OptimizationModelIF::constraintIterator cit = pRobustTmp->constraintBegin(); cit != pRobustTmp->constraintEnd(); cit++)
            pOutTmp->add_constraint( *cit );
    }
    
    if (pIn->getObjType() == robust)
        pOutTmp->set_objective(pMiddle->getObj() );
    
    else
    {
        if(newObj.size() == 0)
            throw MyException("The objective size should not be 0");
        
        if(newObj.size() == 1)
        {
            ROCPPObjectiveIF_Ptr objToSet( new SimpleObjective(newObj[0] ) );
            pOutTmp->set_objective(objToSet);
        }
        else
        {
            ROCPPObjectiveIF_Ptr objToSet( new MaxObjective(newObj) );
            pOutTmp->set_objective(objToSet);
        }

    }
    
    pOutTmp->set_ddu(pMiddle);
    
    // Eliminate integer terms appearing in bilinearities
    ROCPPOptModelIF_Ptr pOutTmp2( m_pMItoMB_Bilinear->convertVar( ROCPPOptModelIF_Ptr(pOutTmp) ) );
    
    // then, eliminate bilinearities between binary and other terms
    ROCPPOptModelIF_Ptr pOutTmp3( m_pBTR->linearize(pOutTmp2) );
    
    // convert resulting problem to MISOCP
    ROCPPMISOCP_Ptr pOut( convertToMISOCP(pOutTmp3) );
    
    if(pIn->isDDUOptimizationModel() )
    {
        ROCPPDDUOptModel_Ptr pIn_DDU( static_pointer_cast<DDUOptimizationModel>(pIn));
        
        ROCPPLinearDR_Ptr pLDR(static_pointer_cast<LinearDecisionRule>(m_pCVA));
        
        // ---------------- DECISION-DEPENDENT NON-ANTICIPATIVITY CONSTRAINTS -----------------------------------------------
        
        // |Y_{t,ij}^p| <= M x_{t-1,j}^p \forall i,j,p,t
        for (PartitionConstructorIF::const_iterator mp_it=m_pPartConstructor->begin(); mp_it!=m_pPartConstructor->end(); mp_it++)
        {
            for (OneToExprVariableConverterIF::const_iterator tmldr_it=pLDR->begin(); tmldr_it!=pLDR->end(); tmldr_it++)
            {
                ROCPPVarIF_Ptr odv( pIn_DDU->getVar( tmldr_it->first ) );//Y_{t,i}
                
                for (DDUOptimizationModel::dduIterator ddu_it = pIn_DDU->dduBegin(); ddu_it != pIn_DDU->dduEnd(); ddu_it++)
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
        
        for (DDUOptimizationModel::dduIterator ddu_it=pIn_DDU->dduBegin(); ddu_it != pIn_DDU->dduEnd(); ddu_it++)
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
                    
                    if(tmp == 0)//find the first(basic) partition
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
    cout << "Total time to approximate and robustify: " << duration.count() << " seconds" << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    return pOut;
}


ROCPPOptModelIF_Ptr PiecewiseApproximator::fixBinaryVariableValues(ROCPPOptModelIF_Ptr pKadaptModel, const map<string,bool> &varValues) const
{
    throw MyException("Not implement yet.");
}

pair<double, map<string, ROCPPExpr_Ptr > > PiecewiseApproximator::calculateMeanAndProb(const ROCPPOptModelIF_Ptr pModel, string partition, const map<string,pair<double,double> > &allMap, double allArea)
{
    map<string, ROCPPExpr_Ptr > mapFromUncToMean;
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

void PiecewiseApproximator::getStochasticObj(const pair<double, map<string, ROCPPExpr_Ptr > > meanAndProb, const ROCPPObjectiveIF_Ptr oldObj, vector<ROCPPExpr_Ptr > &newObj)
{
    double probability(meanAndProb.first);
    map<string, ROCPPExpr_Ptr > mapFromUncToMean(meanAndProb.second);
    
    bool firstPartition(newObj.size() == 0);
    
    size_t objSize(oldObj->getNumTermsMaxObjective());
    
    if ((!firstPartition) && (newObj.size() != objSize))
        throw MyException("Something wrong happened before");
    
    for (uint numObj = 1; numObj <= objSize; numObj++) {
        if (firstPartition) {
            ROCPPExpr_Ptr newObjExpr(new LHSExpression());
            
            ROCPPExpr_Ptr oldObjExpr;
            oldObjExpr = oldObj->getObj(numObj);
            
            newObjExpr->add(probability, oldObjExpr->mapUncs(mapFromUncToMean));
            newObj.push_back(newObjExpr);
        }
        else{
            ROCPPExpr_Ptr oldObjExpr;
            oldObjExpr = oldObj->getObj(numObj);
            
            ROCPPExpr_Ptr partObj(oldObjExpr->mapUncs(mapFromUncToMean));
            map<string,ROCPPVarIF_Ptr > mapVar;
            
            dvMapType::const_iterator v_it(partObj->varsBegin());
            for ( ; v_it != partObj->varsEnd(); v_it++)
            {
                if (newObj[numObj-1]->varIsInvolved(v_it->second))
                    mapVar.insert(make_pair(v_it->first, newObj[numObj-1]->getVar(v_it->first)));
            }
            
            ROCPPExpr_Ptr objToAdd(partObj->mapExprVars(mapVar));
            newObj[numObj-1]->add(probability, objToAdd);
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string PiecewiseApproximator::getSolutionApproachParameters(string delimiter) const
{
    string out("Prepartition");
    out += delimiter;
    out += m_numPartitionsStr;
    out += delimiter;
    
    return out;
}

pair<bool,ROCPPVarIF_Ptr > PiecewiseApproximator::findOrigVariable(ROCPPVarIF_Ptr newdv) const
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

ROCPPVarIF_Ptr PiecewiseApproximator::getVarOnPartition(string partition, string origVarName) const
{
    
    map<string, map<string,ROCPPVarIF_Ptr > >::const_iterator it (m_VariableMap.find(partition));
    if (it==m_VariableMap.end())
        throw MyException("subset of partition " + partition + " not found in map");
    
    map<string,ROCPPVarIF_Ptr >::const_iterator subit(it->second.find(origVarName));
    
    if (subit==it->second.end())
        throw MyException("decision variable " + origVarName + " not found on subset " + partition + " of partition");
    
    return subit->second;
}

//???: Question here
void PiecewiseApproximator::getWsSolutions(ROCPPOptModelIF_Ptr pIn, const map<string, double> &warmStartResults, const map<string,uint> &wsMap, map<string, double> &wsSolutions)
{
    if ( (m_pPartConstructor->getNumSubsets()!=1) && (pIn->getNumBoolVars()) )
    {
        
        for (map<string, map<string,ROCPPVarIF_Ptr > >::const_iterator p_it = m_VariableMap.begin(); p_it != m_VariableMap.end(); p_it++)
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
            
            
            for (map<string,ROCPPVarIF_Ptr >::const_iterator v_it = p_it->second.begin(); v_it != p_it->second.end(); v_it++)
            {
                // find the mappedPartition in m_variableMap
                map< string , map<string, ROCPPVarIF_Ptr > >::const_iterator mapped_it = m_VariableMap.find( mappedPartition );
                
                if (mapped_it==m_VariableMap.end())
                    throw MyException("mappedPartition not found");
                
                // find the name of the variable in mapped partition
                
                map<string, ROCPPVarIF_Ptr >::const_iterator mv_it ( mapped_it->second.find(v_it->first) );
                
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

void PiecewiseApproximator::calculateSolution(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv, string partition)
{
    map<string, ROCPPVarIF_Ptr > variableOnPartition = m_VariableMap.find(partition)->second;
    
    map<string, double> variableValue;
    map<string, ROCPPVarIF_Ptr >::const_iterator variable = variableOnPartition.begin();
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

void PiecewiseApproximator::calculateSolution(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc, string partition)
{
    map<string, ROCPPVarIF_Ptr > variableOnPartition = m_VariableMap.find(partition)->second;
    
    map<string, double> variableValue;
    map<string, ROCPPVarIF_Ptr >::const_iterator variable = variableOnPartition.begin();
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

void PiecewiseApproximator::printParametersToScreen() const
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

void PiecewiseApproximator::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv, map<ROCPPUnc_Ptr, uint> partitionIn)
{
    map<string, uint> stringPartition;
    
    map<ROCPPUnc_Ptr, uint>::const_iterator tmp = partitionIn.begin();
    for(;tmp != partitionIn.end(); tmp++)
        stringPartition.insert(make_pair(tmp->first->getName(), tmp->second));
    
    string partition = m_pPartConverter->convertPartitionToString(stringPartition, pIn);
    
    calculateSolution(pIn, resultIn, dv, partition);
}

void PiecewiseApproximator::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv)
{
    map<string, map<string,ROCPPVarIF_Ptr > >::const_iterator partitionIn;
    for(partitionIn = m_VariableMap.begin(); partitionIn != m_VariableMap.end(); partitionIn++)
    {
        calculateSolution(pIn, resultIn, dv, partitionIn->first);
    }
}

void PiecewiseApproximator::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc, map<ROCPPUnc_Ptr, uint> partitionIn)
{
    map<string, uint> stringPartition;
    
    map<ROCPPUnc_Ptr, uint>::const_iterator tmp = partitionIn.begin();
    for(;tmp != partitionIn.end(); tmp++)
        stringPartition.insert(make_pair(tmp->first->getName(), tmp->second));
    
    string partition = m_pPartConverter->convertPartitionToString(stringPartition, pIn);
    
    calculateSolution(pIn, resultIn, unc, partition);
}

void PiecewiseApproximator::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc)
{
    map<string, map<string,ROCPPVarIF_Ptr > >::const_iterator partitionIn;
    for(partitionIn = m_VariableMap.begin(); partitionIn != m_VariableMap.end(); partitionIn++)
    {
        calculateSolution(pIn, resultIn, unc, partitionIn->first);
    }
}
