//
//  DDUApproximatorOther.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 The University of Southern California. All Rights Reserved.
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
#include "DDUApproximatorOther.hpp"
#include "IndexSetCreator.hpp"
#include <iomanip>
#include <chrono>




//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% K-ADAPTABILITY PARTITION ENCODER %%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

KadaptabilityPartitionEncoderMS::KadaptabilityPartitionEncoderMS(ROCPPOptModelIF_Ptr pIn, string numPartitionsStr)
{
    u_long remainder(numPartitionsStr.size() % pIn->getNumTimeStages());
    if (remainder!=0)
        throw MyException("In K-adaptability approximator constructor, the input string you provide to encode the value of K for each period must have a length that is a multiple of the number of time-periods in the problem");
    
    
    u_long numEls = numPartitionsStr.size() / pIn->getNumTimeStages();
    
    
    //map<uint,uint> numPartitionsMap;
    
    for (uint t=0; t<pIn->getNumTimeStages(); t++)
    {
        if (t==0)
            m_numPartitionsMap.insert(make_pair(1,1));
        else
        {
            string str(numPartitionsStr.substr(t*numEls,numEls));
            //m_numPartitionsMap.insert(make_pair(t+1,lexical_cast<uint>(str)));
            m_numPartitionsMap.insert(make_pair(t+1,stoi(str)));
        }
    }
    
    getReady();
    
    //uint tmp=0;
}

KadaptabilityPartitionEncoderMS::KadaptabilityPartitionEncoderMS(const map<uint,uint> &numPartitionsMap) :
    m_numPartitionsMap(numPartitionsMap)
{
    getReady();
}

KadaptabilityPartitionEncoderMS::KadaptabilityPartitionEncoderMS(uint T, uint K)
{
    for (uint t=1; t<=T; t++)
    {
        m_numPartitionsMap.insert(make_pair(t,K));
    }
    getReady();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void KadaptabilityPartitionEncoderMS::getReady()
{
    // first, change the number of subset for time t=1 to Kt=1
    if (m_numPartitionsMap.begin()->first == 1)
        m_numPartitionsMap.begin()->second = 1;
    
    // check that the map provided is correct
    uint min_t(m_numPartitionsMap.begin()->first);
    uint max_t(m_numPartitionsMap.rbegin()->first);

    if (min_t != 1)
      throw MyException("partitions map for K-adaptability should start with 1");

    if (m_numPartitionsMap.size() != max_t-min_t+1)
      throw MyException("partitions map for K-adaptability should provide a value for Kt for all t starting from 1");
    
    
    uint T(m_numPartitionsMap.rbegin()->first);
    
    for (uint t=1; t<=T; t++)
    {
        
        map<string, map<uint, uint> > submapPartitionNme_to_mapTime_to_k;
        
        vector<vector<uint> > BPconfigs;
        map<uint,uint> subPartitionsMap;
        
        map<uint,uint>::const_iterator tmp=m_numPartitionsMap.find(t);
        tmp++;
        
        for ( map<uint,uint>::const_iterator pit = m_numPartitionsMap.begin(); pit != tmp; pit++)
            subPartitionsMap.insert(make_pair(pit->first, pit->second));
        
        createAllPartitionsKadapt(BPconfigs, t, subPartitionsMap);
        
        for (vector<vector<uint> >::const_iterator it = BPconfigs.begin(); it != BPconfigs.end(); it++)
        {
            string PartitionName(convertPartitionToString(*it));
            
            map<uint, uint> mapTimeTok;
            
            for (uint st=1; st<=t; st++)
            {
                mapTimeTok.insert(make_pair(st, (*it)[st-1]));
            }
            
            submapPartitionNme_to_mapTime_to_k.insert(make_pair(PartitionName, mapTimeTok));
            
            if (t==T)
                m_mapPartitionNme_to_mapTime_to_k.insert(make_pair(PartitionName, mapTimeTok));
            
        }
        
        m_mapt_mapPartitionNme_to_mapTime_to_k.insert(make_pair(t, submapPartitionNme_to_mapTime_to_k));
        
    }
    
}

string KadaptabilityPartitionEncoderMS::convertPartitionToString(const vector<uint> &partition) const
{
    string out = "(";
    for (vector<uint>::const_iterator it = partition.begin(); it != partition.end(); it++)
    {
        if (it == partition.begin())
        {
            out += to_string(*it);
        }
        else
        {
            out += "-";
            out += to_string(*it);
        }
            
    }
    out += ")";
    return out;
}

string KadaptabilityPartitionEncoderMS::convertPartitionToString(const map<uint,uint> &partitionMap) const
{
    vector<uint> tmp;
    for (map<uint,uint>::const_iterator it = partitionMap.begin(); it != partitionMap.end(); it++)
    {
        tmp.push_back(it->second);
    }
    return convertPartitionToString(tmp);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

uint KadaptabilityPartitionEncoderMS::getkt(string partition, uint t) const
{
    map< uint , map< string, map<uint, uint> > >::const_iterator tit ( m_mapt_mapPartitionNme_to_mapTime_to_k.find(t) );
    
    if (tit == m_mapt_mapPartitionNme_to_mapTime_to_k.end() )
        throw MyException("time period not found in m_mapt_mapPartitionNme_to_mapTime_to_k");
    
    map< string, map<uint, uint> >::const_iterator pit = tit->second.find(partition);
    
    if (pit == tit->second.end())
        throw MyException("time period not found in submap of m_mapt_mapPartitionNme_to_mapTime_to_k");
    
    map<uint, uint>::const_iterator tit2( pit->second.find(t) );
    
    if (tit2 == pit->second.end() )
        throw MyException("time period not found in submap of submap of m_mapt_mapPartitionNme_to_mapTime_to_k");
    
    return tit2->second;
    
}

string KadaptabilityPartitionEncoderMS::getPartitionSubset(const map<uint,uint> &partitionMap, uint t) const
{
    map<uint,uint> submap;
    for (map<uint,uint>::const_iterator it = partitionMap.begin(); it != partitionMap.end(); it++)
    {
        if (it->first <= t)
            submap.insert(make_pair(it->first, it->second));
    }
    return convertPartitionToString(submap);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%% K-adaptability APPROXIMATOR MULTI-STAGE %%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

KadaptabilityApproximatorMS::KadaptabilityApproximatorMS(ROCPPOptModelIF_Ptr pIn, uint K, double bigM, double epsilon, string folder) : m_folder(folder), m_epsilon(epsilon)
{
    ROCPPKadaptEncoder_Ptr pPartitionEncoder (new KadaptabilityPartitionEncoderMS(pIn->getNumTimeStages(),K));
    m_pPartitionEncoder=pPartitionEncoder;
    m_pBTR = ROCPPBilinearReform_Ptr(new BTR_bigM("bl", "", 0, bigM));
}

KadaptabilityApproximatorMS::KadaptabilityApproximatorMS(ROCPPKadaptEncoder_Ptr pPartitionEncoder, double bigM, double epsilon, string folder) : m_pPartitionEncoder(pPartitionEncoder), m_folder(folder), m_epsilon(epsilon)
{
    m_pBTR = ROCPPBilinearReform_Ptr(new BTR_bigM("bl", "", 0, bigM));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% Compatibility Functions %%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void KadaptabilityApproximatorMS::checkCompatability(ROCPPOptModelIF_Ptr pIn)
{
    
    OptimizationModelIF::varsIterator pVar = pIn->varsBegin();
    for(; pVar != pIn->varsEnd(); pVar++)
    {
        if (!(pVar->second->isBooleanVar()))
            throw MyException("Multi-stage K-adaptability approximator only works on problems with boolean variables");
    }
    
    if(pIn->getObjType() != robust)
        throw MyException("Multi-stage K-adaptability approximator only works on robust problems");
    
}

void KadaptabilityApproximatorMS::checkWsCompatability(const map<uint,uint> &wsMap)
{
    if(wsMap.size() != m_pPartitionEncoder->getT())
        throw MyException("The time span of k must equal to the time stage of the model");
    
    bool isSame = true;
    
    for(uint t=1; t<=m_pPartitionEncoder->getT(); t++){
        if(wsMap.find(t)->second > m_pPartitionEncoder->getKt(t))
            throw MyException("The warm start k can not excess the original k");
        else if (wsMap.find(t)->second < m_pPartitionEncoder->getKt(t))
            isSame = false;
    }
    
    if(isSame)
        throw MyException("The warm start k can equal to the original k");
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void KadaptabilityApproximatorMS::createVariableAndUncMap(dvContainer::const_iterator varsBegin, dvContainer::const_iterator varsEnd, uncContainer::const_iterator uncBegin, uncContainer::const_iterator uncEnd)
{
    
    uint numPart(1);
    size_t totalPart(m_pPartitionEncoder->getKmapSize());
    
    for (KadaptabilityPartitionEncoderMS::Kmap_iterator kmit = m_pPartitionEncoder->Kmap_iteratorBegin(); kmit != m_pPartitionEncoder->Kmap_iteratorEnd(); kmit++)
    {
        cout << "Mapping decision variables for contingency plan " << numPart << " of total " << totalPart << " plans." << endl;
        numPart++;
        
        map<string, ROCPPVarIF_Ptr > subMap;
        
        for (dvContainer::const_iterator v_it = varsBegin; v_it != varsEnd; v_it++)
        {
            string subPartitionString( m_pPartitionEncoder->getPartitionSubset(kmit->second, v_it->second->getTimeStage()));
            
            ROCPPVarIF_Ptr newdv;
            if(v_it->second->isAdaptive())
                newdv = ROCPPVarIF_Ptr(new VariableBool(  v_it->second->getName() + "_" + subPartitionString  , v_it->second->getLB(), v_it->second->getUB() ) );
            else
                newdv = v_it->second;

            subMap.insert( make_pair( v_it->second->getName() , newdv ) );
            
            m_DVmap[v_it->second->getName()].insert(make_pair(subPartitionString,newdv));
        }
        
        m_mapPartitionEnc_mapOrigDVtoDVonPartition.insert(make_pair(kmit->first, subMap));

    }
         
         
    
    // ---- first create decision variables -----
    
    
//    // iterate through the decision variables
//
//    for (dvContainer::const_iterator v_it = varsBegin; v_it != varsEnd; v_it++)
//    {
//
//        uint t(v_it->second->getTimeStage());
//
//        uint Kt(m_pPartitionEncoder->getKt(t));
//
//        map<uint, ROCPPVarIF_Ptr > subDVmap;
//
//        // for each value of k_t associated with the time-stage t of the decision variable, create a decision variable
//        for (uint kt = 1; kt <= Kt; kt++)
//        {
//            ROCPPVarIF_Ptr newdv;
//            if(v_it->second->isAdaptive())
//                newdv = ROCPPVarIF_Ptr(new VariableBool(  v_it->second->getName() + "_(" + lexical_cast<string>(kt) +")" , v_it->second->getLB(), v_it->second->getUB() ) );
//            else
//                newdv = v_it->second;
//
//            subDVmap.insert( make_pair( kt , newdv ) );
//        }
//
//        m_DVmap.insert(make_pair(v_it->second->getName(), subDVmap));
//
//    }
//
//
//    // finally, put the decision variables inside all the right places in m_mapPartitionEnc_mapOrigDVtoDVonPartition
//
//    for (KadaptabilityPartitionEncoderMS::Kmap_iterator kmit = m_pPartitionEncoder->Kmap_iteratorBegin();
//         kmit != m_pPartitionEncoder->Kmap_iteratorEnd(); kmit++)
//    {
//
//        map<string, ROCPPVarIF_Ptr > subMap;
//
//        for (dvContainer::const_iterator v_it = varsBegin; v_it != varsEnd; v_it++)
//        {
//            // find the value of kt on this partition where t is the time stage of the decision variable
//
//            uint dvtp(v_it->second->getTimeStage());
//
//            map<uint,uint>::const_iterator tmpit(kmit->second.find(dvtp));
//
//            if (tmpit==kmit->second.end())
//                throw MyException("time period not found in Kmap encoder");
//
//            uint kt(tmpit->second);
//
//            // find the decision variable associated with kt in DV map
//            map<string, map<uint, ROCPPVarIF_Ptr > >::const_iterator v_it2 (m_DVmap.find(v_it->first));
//
//            if (v_it2==m_DVmap.end())
//                throw MyException("Decision variable not found in DVmap");
//
//            // in the submap find the value of kt
//
//            map<uint, ROCPPVarIF_Ptr >::const_iterator kt_it(v_it2->second.find(kt));
//
//            if (kt_it==v_it2->second.end())
//                throw MyException("kt value not found in DVmap submap");
//
//            // add decision variable to map
//            subMap.insert(make_pair(v_it->first, kt_it->second));
//
//        }
//
//        m_mapPartitionEnc_mapOrigDVtoDVonPartition.insert(make_pair(kmit->first, subMap));
//
//    }
    
    // ---- second create uncertain parameters -----
    
    cout << "Mapping uncertain parameters for each partition" << endl;
    
    map< string , map< pair<uint,string> , ROCPPUnc_Ptr > > UncMap; // map from original uncertainty name to t and partition value to new uncertain parameter
    
    // iterate through the uncertain parameters
    
    for (uncContainer::const_iterator u_it = uncBegin; u_it != uncEnd; u_it++)
    {
        
        map< pair<uint,string> , ROCPPUnc_Ptr > subUncMap;
        
        // iterate through all the time periods and all subsets for that time period
        for (KadaptabilityPartitionEncoderMS::Klargemap_iterator tit = m_pPartitionEncoder->Klargemap_iteratorBegin(); tit != m_pPartitionEncoder->Klargemap_iteratorEnd(); tit++)
        {
            
            for (map< string, map<uint, uint> >::const_iterator pit = tit->second.begin(); pit !=
                 tit->second.end(); pit++)
            {
                uint t( tit->first );
                ROCPPUnc_Ptr newunc( new UncertaintyIF(  u_it->second->getName() + "_" + to_string(t) + "_" + pit->first , u_it->second->isObservable() ) );
                
                subUncMap.insert( make_pair( make_pair(t,pit->first) , newunc ) );
            }
        }
        
        UncMap.insert(make_pair(u_it->first, subUncMap));
    }
    
    
    
    // finally, put the uncertain parameters inside all the right places in m_mapPartitionEncandt_mapOrigUnctoUnconPartition
    
    for (KadaptabilityPartitionEncoderMS::Klargemap_iterator tit = m_pPartitionEncoder->Klargemap_iteratorBegin();
         tit != m_pPartitionEncoder->Klargemap_iteratorEnd(); tit++)
    {
        
        for (map< string, map<uint, uint> >::const_iterator pit = tit->second.begin(); pit !=
             tit->second.end(); pit++)
        {
            map<string, ROCPPUnc_Ptr > subMap;
            
            for (uncContainer::const_iterator u_it = uncBegin; u_it != uncEnd; u_it++)
            {
                // find the uncertain parameter in UncMap
                map<string, map< pair<uint,string> , ROCPPUnc_Ptr > >::const_iterator u_it2 (UncMap.find(u_it->first));
                
                if (u_it2==UncMap.end())
                    throw MyException("Uncertainty not found in UncMap");
                
                // in the submap find the pair (t and partition)
                
                map< pair<uint,string> , ROCPPUnc_Ptr >::const_iterator kt_it(u_it2->second.find(make_pair(tit->first,pit->first)));
                
                if (kt_it==u_it2->second.end())
                    throw MyException("partition not found in DVmap submap");
                
                // add uncertain parameter to map
                subMap.insert(make_pair(u_it->first, kt_it->second));
                
            }
            
            m_mapPartitionEncandt_mapOrigUnctoUnconPartition.insert(make_pair(make_pair(pit->first,tit->first), subMap));
            
        }
        
        
    }
}

ROCPPMISOCP_Ptr KadaptabilityApproximatorMS::approx(ROCPPOptModelIF_Ptr pIn)
{
    cout << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "================== APPROXIMATING USING K-ADAPTABILITY ===================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    auto start = chrono::high_resolution_clock::now();

    
    checkCompatability(pIn);
    
    // Mark: change here
    
    if(pIn->getNumTimeStages() != m_pPartitionEncoder->getT())
        throw MyException("The time span of k must equal the number of time stages in the model");
    
    bool isUnc(false);
    
    OptimizationModelIF::constraintIterator pCstr = pIn->constraintBegin();
    for(; pCstr != pIn->constraintEnd(); pCstr++)
    {
        if(!((*pCstr)->isDeterministic()) && !((*pCstr)->definesUncertaintySet() ) )
        {
            if((pIn->getNumTimeStages() != 2) )
                throw MyException("Multi-stage K-adaptability approximator only works on problems with objective uncertainty");
            else{
                isUnc = true;
                break;
            }
        }
    }
    
    ROCPPMISOCP_Ptr pOut;
    if(isUnc)
        pOut = approxCstrUnc(pIn);
    else
        pOut = approxObjUnc(pIn);
    
    auto stop = chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<chrono::seconds>(stop - start);
    
    cout << endl;
    cout << "Total time to approximate and robustify: " << duration.count() << " seconds" << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    
    return pOut;
}

ROCPPMISOCP_Ptr KadaptabilityApproximatorMS::approxObjUnc(ROCPPOptModelIF_Ptr pIn)
{
    checkCompatability(pIn);
    
    createVariableAndUncMap(pIn->varsBegin(), pIn->varsEnd(), pIn->uncertaintiesBegin(), pIn->uncertaintiesEnd());
    
    
    // **** first, create all for possible elements from the set {1,...,I}^K where I is the number of terms in the objective *****
    
    vector<vector<uint> > elements;
    
    size_t numSubsets(1);
    
    for (KadaptabilityPartitionEncoderMS::numPartitionMap_iterator kmit = m_pPartitionEncoder->numPartitionMap_iteratorBegin(); kmit != m_pPartitionEncoder->numPartitionMap_iteratorEnd(); kmit++)
    {
        numSubsets *= kmit->second;
    }
    
    createAllVectorsElements1toI(elements, numSubsets, pIn->getObj()->getNumTermsMaxObjective() );
    
    // **** initialize the problem ****

    ROCPPBilinMISOCP_Ptr pBilinearMISOCPout( new Bilinear_MISOCP());

    // add all the deterministic constraints from the original problem to pBilinearMISOCPout

    for (OptimizationModelIF::constraintIterator c_it = pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
    {
        if ( (*c_it)->isDeterministic() && (*c_it)->getNumAdaptiveVars()!=0 )
        {
            // need to map the variables in this case
            for ( KadaptabilityPartitionEncoderMS::Kmap_iterator kit = m_pPartitionEncoder->Kmap_iteratorBegin();
                 kit != m_pPartitionEncoder->Kmap_iteratorEnd(); kit++ )
            {
                // find the submaps that refer to this choice of k
                map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator dvMapit( m_mapPartitionEnc_mapOrigDVtoDVonPartition.find(kit->first) ) ;

                if (dvMapit == m_mapPartitionEnc_mapOrigDVtoDVonPartition.end())
                    throw MyException("contingency plan not found in decision variable map");

                ROCPPConstraint_Ptr newcstr (  (*c_it)->mapVars(dvMapit->second) );
                //newcstr = newcstr->mapVars( dvMapit->second );


                pBilinearMISOCPout->add_constraint(newcstr);
            }

        }
        else if ((*c_it)->isDeterministic())
        {
            // map the variables to the variables on the basic partition
            string basicPartition(m_pPartitionEncoder->getBasicPartition());
            
            // find the submaps that refer to this choice of k
            map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator dvMapit( m_mapPartitionEnc_mapOrigDVtoDVonPartition.find(basicPartition) ) ;
            
            if (dvMapit == m_mapPartitionEnc_mapOrigDVtoDVonPartition.end())
                throw MyException("contingency plan not found in decision variable map");
            
            ROCPPConstraint_Ptr newcstr ( (*c_it)->mapVars(dvMapit->second) );

            pBilinearMISOCPout->add_constraint(newcstr);
        }
        else if ( !(*c_it)->definesUncertaintySet() && !(*c_it)->isDeterministic() )
            throw MyException("there shouldn‘t be any uncertain constraints in this type of problem");
    }
    

    pBilinearMISOCPout = static_pointer_cast<Bilinear_MISOCP>( addProblemSpecificConstraints(pBilinearMISOCPout) );
    
    // *** for each element from elements, create a problem to robustify, robustify it, and add the robustified constraints to the final output problem ****

    uint dualvarscnt(0);
    uint elcnt(0);

    ROCPPVarIF_Ptr pEpi(new VariableDouble("epigraph") );
    ROCPPExpr_Ptr newObjFun(new LHSExpression() );
    newObjFun->add(1.0, pEpi);
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective(newObjFun) );
    pBilinearMISOCPout->set_objective(newObj);

    ROCPPUnc_Ptr pEpigraphUnc (new UncertaintyIF( "tau_unc"));

    //for all objective function
    for (vector<vector<uint> >::const_iterator vit = elements.begin(); vit!=elements.end(); vit++)
    {
        elcnt++;

        if(elcnt % 10 == 0)
            cout << "Creating K-adaptability problem on subset " << elcnt << " of " << elements.size() << endl;

        // **** first, we are going to write the problem as a single stage robut problem with decision-dependent uncertainty set ****
        ROCPPUncSSOptModel_Ptr pRobust ( new UncertainSingleStageOptimizationModel(robust) );

        string elstring(to_string(elcnt));

        // create an uncertain parameter to play the role of the epigraph

        // add to the robust problem the only uncertain constraint (for this choice of element i)

        // add one epigraph constraint to the problem
        ROCPPConstraint_Ptr pcstr_epi( new IneqConstraint() );
        pcstr_epi->add_lhs(1., pEpigraphUnc);
        pcstr_epi->add_lhs(-1., pEpi);
        pcstr_epi->set_rhs(make_pair(0.,true));
        pRobust->add_constraint(pcstr_epi);

        
        uint kcount(0);
        size_t partitionsize(m_pPartitionEncoder->getKmapSize());
        
        for ( KadaptabilityPartitionEncoderMS::Kmap_iterator kit = m_pPartitionEncoder->Kmap_iteratorBegin();
                    kit != m_pPartitionEncoder->Kmap_iteratorEnd(); kit++ )
        {
            kcount++;
            
            cout << "Approximate on contingency plan " << kcount << " of total " << partitionsize << " plans" << endl;
            
            // find the submaps that refer to this choice of k
            map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator dvMapit( m_mapPartitionEnc_mapOrigDVtoDVonPartition.find(kit->first) ) ;
            
            if (dvMapit == m_mapPartitionEnc_mapOrigDVtoDVonPartition.end())
                throw MyException("k value not found in dv map");
            
            map< pair<string, uint> , map<string, ROCPPUnc_Ptr > >::const_iterator uncMapit( m_mapPartitionEncandt_mapOrigUnctoUnconPartition.find( make_pair(kit->first,m_pPartitionEncoder->getT() ) ) );
                
            if (uncMapit == m_mapPartitionEncandt_mapOrigUnctoUnconPartition.end())
                throw MyException("k value not found in unc map");
            
            
            ROCPPConstraint_Ptr pcstr ( new IneqConstraint(true,false) );
            
            ROCPPExpr_Ptr tmp ( pIn->getObj()->getObj((*vit)[kcount-1]) );
            
            
            tmp = tmp->mapExprVars(dvMapit->second);
            tmp = tmp->mapExprUnc(uncMapit->second);
            *tmp *= -1.;

            pcstr->add_lhs(tmp);
            pcstr->add_lhs(1., pEpigraphUnc);

            pcstr->set_rhs(make_pair(0.,true));

            pRobust->add_constraint(pcstr);
            
        

            // add the uncertainty set constraints saying that each of the uncertain parameters on the partition must be in the uncertainty set
        
            
            for (uint t=1; t<=m_pPartitionEncoder->getT(); t++)
            {
                // find the value of the partition up to t
                
                string partitionsub ( m_pPartitionEncoder->getPartitionSubset(kit->second,t));
                
                // find the variable map
                
                map< pair<string,uint> , map<string, ROCPPUnc_Ptr > >::const_iterator uncMapit = m_mapPartitionEncandt_mapOrigUnctoUnconPartition.find(make_pair(partitionsub,t));
                
                if (uncMapit==m_mapPartitionEncandt_mapOrigUnctoUnconPartition.end())
                    throw MyException("pair not found in the uncertainty partition map");
                
                // add the uncertainty set constraints saying that each of the uncertain parameters on the partition must be in the uncertainty set

                for (OptimizationModelIF::constraintIterator c_it = pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
                {
                    if ( (*c_it)->definesUncertaintySet() )
                    {
                        ROCPPConstraint_Ptr newcstr (  (*c_it)->mapUnc(uncMapit->second) );
                        newcstr = newcstr->mapVars( dvMapit->second );

                        pRobust->add_constraint(newcstr);
                    }
                }
                
            }
            
        }
        
        
        // add the non-anticipativity constraints in the uncertainty set
        
        for (KadaptabilityPartitionEncoderMS::Klargemap_iterator tit = m_pPartitionEncoder->Klargemap_iteratorBegin();
             tit != m_pPartitionEncoder->Klargemap_iteratorEnd(); tit++)
        {
            uint t(tit->first);
            
            if (t>1)
            {
                uint prevt(t-1);
                
                for (map< string, map<uint, uint> >::const_iterator pit = tit->second.begin(); pit != tit->second.end(); pit++)
                {
                    string currentPartition(pit->first);
                    string previousPartition(m_pPartitionEncoder->getPartitionSubset(pit->second, prevt));
                    
                    
                    for (map<string, ROCPPUnc_Ptr >::const_iterator u_it = pIn->uncertaintiesBegin(); u_it != pIn->uncertaintiesEnd(); u_it++)
                    {
                        // find the uncertain parameter for time t for this partition
                        ROCPPUnc_Ptr u_current(getUncOnPartition(u_it->second->getName(), currentPartition, t));

                        // find the uncertain parameter for time t-1 for previous partition
                        ROCPPUnc_Ptr u_prev(getUncOnPartition(u_it->second->getName(), previousPartition, prevt));
                        

                        
                        if (pIn->isDDU(u_it->second->getName()))
                        {
                            
                            // find the measurement variable that will help define the DDNAC
                            ROCPPVarIF_Ptr mv ( pIn->getMeasVar(u_it->second->getName(), t-1));

                            // find the measurement variable on this partition
                            map<string, map<string, ROCPPVarIF_Ptr > >::const_iterator dvMapit( m_DVmap.find(mv->getName()) );

                            string deb(m_pPartitionEncoder->getPartitionSubset(pit->second, prevt));
                            map<string, ROCPPVarIF_Ptr >::const_iterator mv_it_k ( dvMapit->second.find( m_pPartitionEncoder->getPartitionSubset(pit->second, prevt) ) );

                            if (mv_it_k == dvMapit->second.end() )
                                throw MyException("decision variable not found in dv map");

                            ROCPPVarIF_Ptr mvp(mv_it_k->second);
                            
                            
                            // create the NACs

                            ROCPPConstraint_Ptr nac (  new EqConstraint(true,false) );
                            nac->add_lhs(1., u_current ,mvp );
                            nac->add_lhs(-1., u_prev ,mvp );
                            nac->set_rhs(make_pair(0.,true));
                            pRobust->add_constraint(nac);
                            
                        }
                        else if (pIn->isObservable(u_it->second->getName()))
                        {
                            if (t > pIn->getFirstStageObservable( u_it->second->getName() ))
                            {
                                
                                ROCPPConstraint_Ptr nac (  new EqConstraint(true,false) );
                                nac->add_lhs(1., u_current );
                                nac->add_lhs(-1., u_prev );
                                nac->set_rhs(make_pair(0.,true));
                                pRobust->add_constraint(nac);

                            }
                        }
                    }
                    
                }
                
            }
        }
        
        

        ROCPPUncSSOptModel_Ptr pRobust2(convertToUSSOM(pRobust));
        
        //uint tmp (pRobust2->getNumUncertaintySetConstraints());
        //uint tmp2 (pRobust2->getNumConstraints());
        
        // **** robustify the problem ***

        ROCPPRobustifyEngine_Ptr m_pRE (new RobustifyEngine(dualvarscnt,elstring));

        ROCPPBilinMISOCP_Ptr pBilinearMISOCP( m_pRE->robustify(pRobust2)  );

        // **** linearize the bilinear terms ****

        ROCPPOptModelIF_Ptr pMISOCP( m_pBTR->linearize(pBilinearMISOCP) );

        // *** add the robustified constraints to the output problem

        for (OptimizationModelIF::constraintIterator cit=pMISOCP->constraintBegin(); cit!=pMISOCP->constraintEnd(); cit++)
            pBilinearMISOCPout->add_constraint(*cit);
    }
    
    // **** linearize the bilinear terms ****

    ROCPPOptModelIF_Ptr pOutTmp( m_pBTR->linearize(pBilinearMISOCPout) );

    ROCPPMISOCP_Ptr pOut( convertToMISOCP(pOutTmp) );
    
    // *** return ***
    
    return pOut;
}

ROCPPMISOCP_Ptr KadaptabilityApproximatorMS::approxCstrUnc(ROCPPOptModelIF_Ptr pIn)
{
    
    // **** initialize the problem ****
    
    ROCPPBilinMISOCP_Ptr pBilinearMISOCPout( new Bilinear_MISOCP());
    
    checkCompatability(pIn);
    
    createVariableAndUncMap(pIn->varsBegin(), pIn->varsEnd(), pIn->uncertaintiesBegin(), pIn->uncertaintiesEnd());
    
    // add all the deterministic constraints from the original problem to pBilinearMISOCPout, store the uncertain constraint
    
    uint numUncCstr = 0;
    vector<ROCPPConstraint_Ptr > uncCstr;
    
    uint m_K = m_pPartitionEncoder->getKt(2);
    
    for (OptimizationModelIF::constraintIterator c_it = pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
    {
        if ( (*c_it)->isDeterministic())
        {
            if ( (*c_it)->getNumAdaptiveVars()!=0 ){
                // need to map the variables in this case
                for ( uint k = 1; k<= m_K; k++ )
                {
                    vector<uint> partition;
                    partition.push_back(1); partition.push_back(k);
                    string partitionNme = m_pPartitionEncoder->convertPartitionToString(partition);
                    // find the submaps that refer to this choice of k
                    map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator dvMapit( m_mapPartitionEnc_mapOrigDVtoDVonPartition.find(partitionNme) ) ;
                    
                    if (dvMapit==m_mapPartitionEnc_mapOrigDVtoDVonPartition.end())
                        throw MyException("k value not found in one of the maps");
                    
                    ROCPPConstraint_Ptr newcstr (  (*c_it)->mapVars(dvMapit->second) );
                    newcstr = newcstr->mapVars( dvMapit->second );
                    
                    
                    pBilinearMISOCPout->add_constraint(newcstr);
                }
            }
            else{
                    pBilinearMISOCPout->add_constraint(*c_it);
            }
        }
        else if ( !(*c_it)->definesUncertaintySet() && !(*c_it)->isDeterministic() ){
            uncCstr.push_back(*c_it);
            numUncCstr += 1;
        }
    }
    
    // **** create all for possible elements from the set {0,1,...,L}^K where L is the number of constraint with uncertainties *****
    
    vector<vector<uint> > elements;
    createAllVectorsElements1toI(elements, m_K, numUncCstr+1);
    
    pBilinearMISOCPout = static_pointer_cast<Bilinear_MISOCP>( addProblemSpecificConstraints(pBilinearMISOCPout) );
    
    // *** for each element from elements, create a problem to robustify, robustify it, and add the robustified constraints to the final output problem ****
    
    uint dualvarscnt(0);
    uint elcnt(0);
    
    ROCPPVarIF_Ptr pEpi(new VariableDouble("epigraph") );
    ROCPPExpr_Ptr newObjFun(new LHSExpression() );
    newObjFun->add(1.0, pEpi);
    ROCPPObjectiveIF_Ptr newObj(new SimpleObjective(newObjFun) );
    pBilinearMISOCPout->set_objective(newObj);
    
    //for all uncertainty sets
    for (vector<vector<uint> >::const_iterator vit = elements.begin(); vit!=elements.end(); vit++)
    {
        if ((*vit).size()<m_K)
            throw MyException("something is wrong with the construction of the i element vector");
        
        elcnt++;
        
        if(elcnt % 10 == 0)
            cout << "Creating K-adaptability problem on uncertainty set " << elcnt << " of " << elements.size() << endl;
        
        // at least one l_k = 0
        if(!allPositive(*vit))
        {
            // **** first, we are going to write the problem as a single stage robut problem with decision-dependent uncertainty set ****
            ROCPPUncSSOptModel_Ptr pRobust ( new UncertainSingleStageOptimizationModel(robust) );
            
            string elstring(to_string(elcnt));
            
            for (OptimizationModelIF::constraintIterator c_it = pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
            {
                if ( (*c_it)->definesUncertaintySet() && (*c_it)->getNumAdaptiveVars()==0 )
                {
                    pRobust->add_constraint(*c_it);
                }
            }
            
            // create lambda expression
            ROCPPExpr_Ptr lambda( new LHSExpression() );
            // create objective expression
            ROCPPExpr_Ptr obj_l( new LHSExpression() );
            
            for ( uint k = 1; k <= m_K; k++ )
            {
                // find the submaps that refer to this choice of k
                vector<uint> partition;
                partition.push_back(1); partition.push_back(k);
                string partitionNme = m_pPartitionEncoder->convertPartitionToString(partition);
                
                map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator dvMapit( m_mapPartitionEnc_mapOrigDVtoDVonPartition.find(partitionNme) ) ;
                map< pair<string,uint>, map<string, ROCPPUnc_Ptr > >::const_iterator uncMapit( m_mapPartitionEncandt_mapOrigUnctoUnconPartition.find(make_pair(partitionNme, 2)) );
                
                if (dvMapit==m_mapPartitionEnc_mapOrigDVtoDVonPartition.end() || uncMapit==m_mapPartitionEncandt_mapOrigUnctoUnconPartition.end())
                    throw MyException("k value not found in one of the maps");
                
                // add the uncertainty set constraints saying that each of the uncertain parameters on the partition must be in the uncertainty set
                
                for (OptimizationModelIF::constraintIterator c_it = pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
                {
                    if ( (*c_it)->definesUncertaintySet() )
                    {
                        
                        ROCPPConstraint_Ptr newcstr (  (*c_it)->mapUnc(uncMapit->second) );
                        newcstr = newcstr->mapVars( dvMapit->second );
                        
                        pRobust->add_constraint(newcstr);
                    }
                }
                
                // if l_k = 0
                uint l_k = (*vit)[k-1] - 1;
                if( !l_k ){
                    ROCPPExpr_Ptr tmp ( pIn->getObj()->getObj(1) );
                    ROCPPVarIF_Ptr lambda_k (new VariableDouble( "lambda_"+to_string(k)+"_"+elstring, 0., 1.) );
                    
                    tmp = tmp->mapExprVars(dvMapit->second);
                    tmp = tmp->mapExprUnc(uncMapit->second);
                    *tmp *= lambda_k;
                    
                    obj_l->add(1.0, tmp);
                    lambda->add(1.0, lambda_k);
                    
                    for(vector<ROCPPConstraint_Ptr >::const_iterator c_it = uncCstr.begin(); c_it != uncCstr.end(); c_it++)
                    {
                        ROCPPConstraint_Ptr newcstr ( (*c_it)->mapUnc(uncMapit->second) );
                        newcstr = newcstr->mapVars( dvMapit->second );
                        newcstr->setParams(true, false);
                        pRobust->add_constraint(newcstr);
                    }
                }
                
                // if l_k > 0
                else{
                    ROCPPConstraint_Ptr oldCstr ( uncCstr[l_k-1] );
                    if (!oldCstr->isClassicConstraint()) {
                        throw MyException("only deal with classic constrain");
                    }
                    
                    oldCstr = oldCstr->mapUnc(uncMapit->second);
                    oldCstr = oldCstr->mapVars(dvMapit->second);
                    ROCPPClassicConstraint_Ptr Cstr = dynamic_pointer_cast<ClassicConstraintIF>(oldCstr);
                    ROCPPConstraint_Ptr newCstr(new IneqConstraint(true));
                    
                    newCstr->add_lhs(-1.0, Cstr->getLHS());
                    newCstr->add_lhs(Cstr->get_rhs().first);
                    newCstr->set_rhs(make_pair(-1*m_epsilon, false));
                    pRobust->add_constraint(newCstr);
                }
                
                for (OptimizationModelIF::uncertaintiesIterator u_it = pIn->uncertaintiesBegin(); u_it != pIn->uncertaintiesEnd(); u_it++)
                {
                    
                    if (pIn->isDDU(u_it->second->getName()))
                    {
                        ROCPPVarIF_Ptr mv ( pIn->getMeasVar(u_it->second->getName(), 1));
                        map<string, ROCPPUnc_Ptr >::const_iterator u_it_k ( uncMapit->second.find(u_it->second->getName()) );
                        
                        if (u_it_k==uncMapit->second.end())
                            throw MyException("Uncertain parameter not found");
                        
                        ROCPPConstraint_Ptr nac (  new EqConstraint(true,false) );
                        nac->add_lhs(1., u_it->second ,mv );
                        nac->add_lhs(-1., u_it_k->second ,mv );
                        nac->set_rhs(make_pair(0.,true));
                        pRobust->add_constraint(nac);
                    }
                    else
                    {
                        if (pIn->isObservable(u_it->second->getName()))
                        {
                            map<string, ROCPPUnc_Ptr >::const_iterator u_it_k ( uncMapit->second.find(u_it->second->getName()) );
                            
                            if (u_it_k==uncMapit->second.end())
                                throw MyException("Uncertain parameter not found");
                            
                            ROCPPConstraint_Ptr nac (  new EqConstraint(true,false) );
                            nac->add_lhs(1., u_it->second );
                            nac->add_lhs(-1., u_it_k->second );
                            nac->set_rhs(make_pair(0.,true));
                            pRobust->add_constraint(nac);
                        }
                    }
                }
            }
            
            //epigraph for objective function
            ROCPPConstraint_Ptr pcstr ( new IneqConstraint() );
            pcstr->add_lhs(obj_l);
            pcstr->add_lhs(-1.0, pEpi);
            pcstr->set_rhs(make_pair(0.,true));
            
            pRobust->add_constraint(pcstr);
            
            //constraint for lambda
            ROCPPConstraint_Ptr lcstr ( new EqConstraint(false, false) );
            lcstr->add_lhs(lambda);
            lcstr->set_rhs(make_pair(1.,false));
            
            pRobust->add_constraint(lcstr);
            
            //convert to bilinear term and discrete it
            ROCPPOptModelIF_Ptr pMISOCP( m_pBTR->linearize(pRobust) );
            
            pRobust = convertToUSSOM(pMISOCP);
            
            //robustify the problem
            ROCPPRobustifyEngine_Ptr m_pRE (new RobustifyEngine(dualvarscnt,elstring) );
            
            ROCPPBilinMISOCP_Ptr pBilinearMISOCP( m_pRE->robustify(pRobust) );
            
            // *** add the robustified constraints to the output problem
            
            for (OptimizationModelIF::constraintIterator cit=pBilinearMISOCP->constraintBegin(); cit!=pBilinearMISOCP->constraintEnd(); cit++)
                pBilinearMISOCPout->add_constraint(*cit);
            
        }
        
        // all l_k > 0
        else{
            // **** first, we are going to write the problem as a single stage robut problem with decision-dependent uncertainty set ****
            ROCPPUncSSOptModel_Ptr pRobust ( new UncertainSingleStageOptimizationModel(robust) );
            
            string elstring(to_string(elcnt));
            
            // add infeasible constraint to the problem
            ROCPPConstraint_Ptr pcstr_inf( new IneqConstraint() );
            pcstr_inf->add_lhs(1.);
            pcstr_inf->set_rhs(make_pair(0.,true));
            pRobust->add_constraint(pcstr_inf);
            
            for (OptimizationModelIF::constraintIterator c_it = pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
            {
                if ( (*c_it)->definesUncertaintySet() && (*c_it)->getNumAdaptiveVars()==0 )
                {
                    pRobust->add_constraint(*c_it);
                }
            }
            
            for ( uint k = 1; k <= m_K; k++ )
            {
                // find the submaps that refer to this choice of k
                vector<uint> partition;
                partition.push_back(1); partition.push_back(k);
                string partitionNme = m_pPartitionEncoder->convertPartitionToString(partition);
                
                map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator dvMapit( m_mapPartitionEnc_mapOrigDVtoDVonPartition.find(partitionNme) ) ;
                map< pair<string,uint>, map<string, ROCPPUnc_Ptr > >::const_iterator uncMapit( m_mapPartitionEncandt_mapOrigUnctoUnconPartition.find(make_pair(partitionNme, 2)) );
                
                if (dvMapit==m_mapPartitionEnc_mapOrigDVtoDVonPartition.end() || uncMapit==m_mapPartitionEncandt_mapOrigUnctoUnconPartition.end())
                    throw MyException("k value not found in one of the maps");
                
                // add the uncertainty set constraints saying that each of the uncertain parameters on the partition must be in the uncertainty set
                
                for (OptimizationModelIF::constraintIterator c_it = pIn->constraintBegin(); c_it != pIn->constraintEnd(); c_it++)
                {
                    if ( (*c_it)->definesUncertaintySet() )
                    {
                        
                        ROCPPConstraint_Ptr newcstr (  (*c_it)->mapUnc(uncMapit->second) );
                        newcstr = newcstr->mapVars( dvMapit->second );
                        
                        pRobust->add_constraint(newcstr);
                    }
                }
                
                ROCPPConstraint_Ptr oldCstr ( uncCstr[ (*vit)[k-1] - 2 ] );
                if (!oldCstr->isClassicConstraint()) {
                    throw MyException("only deal with classic constrain");
                }
                
                ROCPPClassicConstraint_Ptr Cstr = dynamic_pointer_cast<ClassicConstraintIF>(oldCstr);
                
                ROCPPConstraint_Ptr newCstr(new IneqConstraint(true) );
                
                newCstr->add_lhs(-1.0, Cstr->getLHS() );
                newCstr->add_lhs(Cstr->get_rhs().first);
                newCstr->set_rhs(make_pair(-1*m_epsilon, false) );
                newCstr = newCstr->mapVars(dvMapit->second);
                newCstr = newCstr->mapUnc(uncMapit->second);
                pRobust->add_constraint(newCstr);
                
                for (OptimizationModelIF::uncertaintiesIterator u_it = pIn->uncertaintiesBegin(); u_it != pIn->uncertaintiesEnd(); u_it++)
                {
                    
                    if (pIn->isDDU(u_it->second->getName()))
                    {
                        ROCPPVarIF_Ptr mv ( pIn->getMeasVar(u_it->second->getName(), 1));
                        map<string, ROCPPUnc_Ptr >::const_iterator u_it_k ( uncMapit->second.find(u_it->second->getName()) );
                        
                        if (u_it_k==uncMapit->second.end())
                            throw MyException("Uncertain parameter not found");
                        
                        ROCPPConstraint_Ptr nac (  new EqConstraint(true,false) );
                        nac->add_lhs(1., u_it->second ,mv );
                        nac->add_lhs(-1., u_it_k->second ,mv );
                        nac->set_rhs(make_pair(0.,true));
                        pRobust->add_constraint(nac);
                    }
                    else
                    {
                        if (pIn->isObservable(u_it->second->getName()))
                        {
                            map<string, ROCPPUnc_Ptr >::const_iterator u_it_k ( uncMapit->second.find(u_it->second->getName()) );
                            
                            if (u_it_k==uncMapit->second.end())
                                throw MyException("Uncertain parameter not found");
                            
                            ROCPPConstraint_Ptr nac (  new EqConstraint(true,false) );
                            nac->add_lhs(1., u_it->second );
                            nac->add_lhs(-1., u_it_k->second );
                            nac->set_rhs(make_pair(0.,true));
                            pRobust->add_constraint(nac);
                        }
                    }
                }
            }
            
            ROCPPUncSSOptModel_Ptr pRobust2(convertToUSSOM(pRobust));
            
            // **** robustify the problem ***
            
            ROCPPRobustifyEngine_Ptr m_pRE (new RobustifyEngine(dualvarscnt,elstring));
            
            ROCPPBilinMISOCP_Ptr pBilinearMISOCP( m_pRE->robustify(pRobust2, false)  );
            
            // *** add the robustified constraints to the output problem
            
            for (OptimizationModelIF::constraintIterator cit=pBilinearMISOCP->constraintBegin(); cit!=pBilinearMISOCP->constraintEnd(); cit++)
                pBilinearMISOCPout->add_constraint(*cit);
        }
    }
    
    // **** linearize the bilinear terms ****
    
    ROCPPOptModelIF_Ptr pOutTmp( m_pBTR->linearize(pBilinearMISOCPout) );
    
    ROCPPMISOCP_Ptr pOut( convertToMISOCP(pOutTmp) );
    
    // *** return ***
    
    return pOut;
}
/*
ROCPPOptModelIF_Ptr KadaptabilityApproximatorMS::fixSecondStageVariablesToWarmStart(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pKadaptModel, string folderName, string slnName, uint K)
{
    map<string, vector<double> > valsMap;
    readCSV(folderName, slnName+"All", valsMap, ' ', ".sol", true);
    
    map<string, double> wsResults;
    map<string, vector<double> >::const_iterator val = valsMap.begin();
    
    for(; val != valsMap.end(); val++)
        wsResults.insert(make_pair(val->first, val->second[0]));
    
    return fixSecondStageVariablesToWarmStart(pIn, pKadaptModel, wsResults, K);
}

ROCPPOptModelIF_Ptr KadaptabilityApproximatorMS::fixSecondStageVariablesToWarmStart(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pKadaptModel, string folderName, string slnName, const map<uint,uint> &wsMap)
{
    map<string, vector<double> > valsMap;
    readCSV(folderName, slnName+"All", valsMap, ' ',".sol", true);
    
    map<string, double> wsResults;
    map<string, vector<double> >::const_iterator val = valsMap.begin();
    
    for(; val != valsMap.end(); val++)
        wsResults.insert(make_pair(val->first, val->second[0]));
    
    return fixSecondStageVariablesToWarmStart(pIn, pKadaptModel, wsResults, wsMap);
}

ROCPPOptModelIF_Ptr KadaptabilityApproximatorMS::fixSecondStageVariablesToWarmStart(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pKadaptModel, const map<string,double> &warmStartResults, uint K)
{
    uint T(pIn->getNumTimeStages());

    map<uint, uint> wsMap;

//    wsMap.insert(make_pair(1, 1));
//
//    for(uint t=2; t<= T; t++)
//        wsMap.insert(make_pair(t, K));
//
    return fixSecondStageVariablesToWarmStart(pIn, pKadaptModel, warmStartResults, wsMap);
}
*/
ROCPPOptModelIF_Ptr KadaptabilityApproximatorMS::fixSecondStageVariablesToWarmStart(ROCPPOptModelIF_Ptr pIn, ROCPPOptModelIF_Ptr pKadaptModel, const map<string,double> &warmStartResults, const map<uint,uint> &wsMap)
{
    checkWsCompatability(wsMap);

    // populate a map with the variables and their values (assumes they are binary)
    map<string,bool> varValues;
    
    // get the name of the largest warm start partition
    string wsStr(m_pPartitionEncoder->convertPartitionToString(wsMap));
    
    for (map<string, map<string, ROCPPVarIF_Ptr > >::const_iterator kit = m_DVmap.begin(); kit != m_DVmap.end(); kit++)
    {

        // for all second stage variables in the problem
        for (map<string, ROCPPVarIF_Ptr >::const_iterator vit = kit->second.begin(); vit != kit->second.end(); vit++)
        {
            if (wsStr.compare(vit->first) != -1 ) // if the name of the variable is NOT the same as the name of the variable on the partition (it is an adaptive variable) and is the variable with small k to be found
            {
                // create the variable name
                string varname (vit->second->getName());
                
                // find the variable in the warm start solution
                map<string, double> ::const_iterator wsit( warmStartResults.find(vit->second->getName() ) );
                if (wsit == warmStartResults.end())
                    throw MyException("variable not found in the warm start");

                // find the variable value
                double dbval(wsit->second);

                // check that it is boolean
                bool varIsZero( DoublesAreEssentiallyEqual(dbval, 0., 0.00001) );
                bool varIsOne( DoublesAreEssentiallyEqual(dbval, 1., 0.00001) );

                if ( (!varIsOne) && (!varIsZero) )
                    throw MyException("unsure if you are zero or 1");

                if ( (varIsOne) && (varIsZero) )
                    throw MyException("unsure if you are zero or 1");

                if (varIsZero)
                    varValues.insert(make_pair(varname, 0));
                else
                    varValues.insert(make_pair(varname, 1));

            }
        }
    }

    // when we are done populating, fix the variable values
    ROCPPOptModelIF_Ptr pOut = fixBinaryVariableValues(pKadaptModel, varValues);
    return pOut;
}

ROCPPOptModelIF_Ptr KadaptabilityApproximatorMS::fixBinaryVariableValues(ROCPPOptModelIF_Ptr pKadaptModel, const map<string,bool> &varValues) const
{
//    map<string, bool>::const_iterator varValue = varValues.begin();
//    for(; varValue != varValues.end(); varValue++)
//    {
//        cout << varValue->first+'=' << varValue->second << endl;
//    }
//
    // we will not add a constraint, instead we completely get rid of the variable using PredefO2EVariableConverter

    map<string,ROCPPExpr_Ptr > translationMap;

    // populate the translation map

    for (map<string,bool >::const_iterator mit = varValues.begin(); mit != varValues.end(); mit++)
    {
        // try to find the variable in the problem

        ROCPPVarIF_Ptr var (pKadaptModel->getVar(mit->first));

        // find its value and add the value to an expression
        //???:bool to double
        double value;
        if(mit->second)
            value = 1.0;
        else
            value = 0.0;
        //double value (lexical_cast<double>(mit->second));
        ROCPPExpr_Ptr expr( new LHSExpression() );
        expr->add(value);

        // add the variable and its value to the translation map
        translationMap.insert(make_pair(var->getName(), expr));

    }

    // create the variable converter
    ROCPPO2EVarConverterIF_Ptr varConverter (new PredefO2EVariableConverter(translationMap));

    // let the variable converter do its magic

    ROCPPOptModelIF_Ptr pOut;
    pOut = varConverter->convertVar(pKadaptModel);
    
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

uint KadaptabilityPartitionEncoderMS::getKt(uint t) const
{
    numPartitionMap_iterator it(m_numPartitionsMap.find(t));
    if (it==numPartitionMap_iteratorEnd())
        throw MyException("Time-stage not found in K-adaptability partition encoder map.");
    
    return it->second;
}

string KadaptabilityApproximatorMS::getSolutionApproachParameters(string delimiter) const
{
    string out("Kadapt");
    out += delimiter;
    out += m_pPartitionEncoder->getMaxPartitionString();
    out += delimiter;
    out += m_pPartitionEncoder->getMaxPartitionString();
    return out;
}

ROCPPVarIF_Ptr KadaptabilityApproximatorMS::getDVonPartition(string partition, string OrigDVname) const
{
    map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator pit( m_mapPartitionEnc_mapOrigDVtoDVonPartition.find(partition) );
    
    if (pit == m_mapPartitionEnc_mapOrigDVtoDVonPartition.end())
        throw MyException("partition not found");
    
    map<string, ROCPPVarIF_Ptr >::const_iterator vit( pit->second.find(OrigDVname) );
    
    if (vit == pit->second.end() )
        throw MyException("variable not found");
    
    return (vit->second);
    
}

ROCPPUnc_Ptr KadaptabilityApproximatorMS::getUncOnPartition(string OrigUncName, string partition, uint t) const
{
    
    // in the submap find the pair (t and partition)
    
    map< pair<string,uint> , map<string, ROCPPUnc_Ptr > >::const_iterator um_it2(m_mapPartitionEncandt_mapOrigUnctoUnconPartition.find(make_pair(partition,t)));
    
    
    if (um_it2==m_mapPartitionEncandt_mapOrigUnctoUnconPartition.end())
        throw MyException("Uncertainty not found in m_mapPartitionEncandt_mapOrigUnctoUnconPartition");
    
    // in the subsub map find the uncertain parameter
    
    map< string , ROCPPUnc_Ptr >::const_iterator u_it2(um_it2->second.find(OrigUncName));
    
    if (u_it2==um_it2->second.end())
        throw MyException("uncertainty not found in m_mapPartitionEncandt_mapOrigUnctoUnconPartition submap");
    
    return u_it2->second;
}
/*
map<string, double> KadaptabilityApproximatorMS::getWsSolutions(ROCPPOptModelIF_Ptr pIn, const map<string, double> &warmStartResults, uint K) const
{
    uint T(pIn->getNumTimeStages());
    
    map<uint, uint> wsMap;
    
    wsMap.insert(make_pair(1, 1));
    
    for(uint t=2; t<= T; t++)
        wsMap.insert(make_pair(t, K));
    
    return getWsSolutions(pIn, warmStartResults, wsMap);
}
*/
void KadaptabilityApproximatorMS::getWsSolutions(ROCPPOptModelIF_Ptr pIn, const map<string, double> &warmStartResults, const map<uint,uint> &wsMap, map<string, double> &wsSolutions)
{
    
    checkWsCompatability(wsMap);
    string wsStr(m_pPartitionEncoder->convertPartitionToString(wsMap));
    
    bool previous;

    for (map<string, map<string, ROCPPVarIF_Ptr > >::const_iterator kit = m_DVmap.begin(); kit != m_DVmap.end(); kit++)
    {
         for (map<string, ROCPPVarIF_Ptr >::const_iterator vit = kit->second.begin(); vit != kit->second.end(); vit++)
         {
             // create the variable name
             string varname (vit->second->getName());

             map<string, double> ::const_iterator wsit( warmStartResults.find(varname) );
             if (wsit != warmStartResults.end())
             {
                 // find the variable value
                 double dbval(wsit->second);

                 // check that it is boolean
                 bool varIsZero( DoublesAreEssentiallyEqual(dbval, 0., 0.00001) );
                 bool varIsOne( DoublesAreEssentiallyEqual(dbval, 1., 0.00001) );

                 if ( (!varIsOne) && (!varIsZero) )
                     throw MyException("unsure if you are zero or 1");

                 if ( (varIsOne) && (varIsZero) )
                     throw MyException("unsure if you are zero or 1");

                 if (varIsZero){
                     wsSolutions.insert(make_pair(varname, 0.));
                     previous = 0;
                 }
                 else{
                     wsSolutions.insert(make_pair(varname, 1.));
                     previous = 1;
                 }
             }
             else
             {
                 if (vit->first.compare(wsStr) == 1)
                 {
                     wsSolutions.insert(make_pair(varname, previous));
                 }
                 else
                     throw MyException("variable not found in the warm start");
             }
         }
    }

    for(OptimizationModelIF::varsIterator var = pIn->varsBegin(); var != pIn->varsEnd(); var++)
    {
        if ( ! var->second->isAdaptive()) {
            string varname (var->second->getName());

            // find the variable value
            map<string, double> ::const_iterator wsit( warmStartResults.find(varname) );
            if (wsit == warmStartResults.end())
                throw MyException("variable not found in the warm start");

            double dbval(wsit->second);

            // check that it is boolean
            bool varIsZero( DoublesAreEssentiallyEqual(dbval, 0., 0.00001) );
            bool varIsOne( DoublesAreEssentiallyEqual(dbval, 1., 0.00001) );

            if ( (!varIsOne) && (!varIsZero) )
                throw MyException("unsure if you are zero or 1");

            if ( (varIsOne) && (varIsZero) )
                throw MyException("unsure if you are zero or 1");

            if (varIsZero){
                wsSolutions.insert(make_pair(varname, 0));
            }
            else{
                wsSolutions.insert(make_pair(varname, 1));
            }
        }
        else
        {
            if(m_DVmap.find(var->first) == m_DVmap.end())
                throw MyException("Not all bool variables are found");
        }
    }
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Print Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void KadaptabilityApproximatorMS::printParametersToScreen() const
{
    cout << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "========================== APPROXIMATION PARAMS =========================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    
    cout << "Solution Approach: K-adaptability" << endl;
    cout << "K-adaptability Configuration: " << m_pPartitionEncoder->getMaxPartitionString() << endl;
    
    cout << "=========================================================================== " << endl;
    cout << endl;
    
}

void KadaptabilityApproximatorMS::printOut(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv, string partition)
{
    map<string, map<string, ROCPPVarIF_Ptr > >::const_iterator dvMap(m_DVmap.find(dv->getName()));
    string dvName = dvMap->second.find(partition)->second->getName();
    double value = resultIn.find(dvName)->second;

    if (dv->isIntegerVar() || dv->isBooleanVar())
        cout << "Value of variable " << dv->getName() << " in contingency plan " << partition << " is: " << (int)(round(value)) << endl;
    else
        cout << "Value of variable " << dv->getName() << " in contingency plan " << partition << " is: " << value << endl;
}

void KadaptabilityApproximatorMS::printOut(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPVarIF_Ptr dv)
{
    
    map<string, map<string, ROCPPVarIF_Ptr > >::const_iterator vit ( m_DVmap.find(dv->getName()) );
    
    if (vit==m_DVmap.end())
        throw MyException("Decision variable not found");
    
    for (map<string, ROCPPVarIF_Ptr >::const_iterator pit = vit->second.begin(); pit != vit->second.end(); pit++)
    {
        printOut(pIn, resultIn, dv, pit->first);
    }
    
}

void KadaptabilityApproximatorMS::printOut(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc, string partition)
{
    if(pIn->getType() != dduType)
        throw MyException("Non ddu type model does not have measurement variables");
    
    if(!pIn->isDDU(unc->getName()))
        throw MyException("The uncertain parameter " + unc->getName() + " does not have a time of revelation that is decision-dependent");
    
    
    map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator pit = m_mapPartitionEnc_mapOrigDVtoDVonPartition.find(partition);
    
    if (pit == m_mapPartitionEnc_mapOrigDVtoDVonPartition.end())
        throw MyException("Partition subset not found");
    
    for(uint t = pIn->getFirstStageObservable(unc->getName()); t <= pIn->getLastStageObservable(unc->getName()); t++)
    {
        ROCPPVarIF_Ptr dv = pIn->getMeasVar(unc->getName(), t);
        map<string, ROCPPVarIF_Ptr >::const_iterator measVarOnSubset_it(pit->second.find(dv->getName()));
        
        if (measVarOnSubset_it==pit->second.end())
            throw MyException("Decision variable not found on subset");
        
        ROCPPVarIF_Ptr measVarOnSubset = measVarOnSubset_it->second;
        
        //???: double to bool
        //MARK: check the value
        //bool value = lexical_cast<bool>(resultIn.find(measVarOnSubset->getName())->second);
        
        double value(resultIn.find(measVarOnSubset->getName())->second);
        bool observed;
        if (abs(value - 1.0) <= 0.01)
            observed = true;
        else if (abs(value - 0.0) <= 0.01)
            observed = false;
        else
            throw MyException("Wrong result for boolean variable.");
        if (observed)
        {
            cout << "Parameter " << unc->getName() << " under contingency plan " << partition << " is observed at time " << t << endl;
            return;
        }
        
    }
    
    cout << "Parameter " << unc->getName() << " under contingency plan " << partition << " is never observed" << endl;
    
}

void KadaptabilityApproximatorMS::printOut(ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc)
{
    
    for (map< string, map<string, ROCPPVarIF_Ptr > >::const_iterator pit = m_mapPartitionEnc_mapOrigDVtoDVonPartition.begin(); pit != m_mapPartitionEnc_mapOrigDVtoDVonPartition.end(); pit++)
    {
        printOut(pIn, resultIn, unc, pit->first);
    }
    
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%% LINEAR DESICION RULE APPROXIMATOR %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

LDRCDRApproximator::LDRCDRApproximator(ROCPPOptModelIF_Ptr pIn, uint memory, uint numBits, double bigM, string folder):
m_bigM(bigM), m_numBits(numBits), m_folder(folder)
{
    
    // do DDU appoximation using piecewise constant decision rule
    m_pCVA = ROCPPContinuousVarsDR_Ptr(new LinearDecisionRule());
    m_pDVA = ROCPPDiscreteVarsDR_Ptr(new ConstantDecisionRule());
    
    // ******************** MI to MB converter ******************************************************
    m_pMItoMB_Bilinear = ROCPPMItoMB_Ptr(new BinaryConverter());
    
    // ******************** bilinear term reformulation ******************************************************
    m_pBTR = ROCPPBilinearReform_Ptr(new BTR_bigM("bl", "", 0, bigM));
    
    // ******************* uncertainty set real var approximator ***********************************
    m_pUSRVA = ROCPPUncSetRealVarApprox_Ptr( new UncertaintySetRealVarApproximator(numBits));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ROCPPMISOCP_Ptr LDRCDRApproximator::approx(ROCPPOptModelIF_Ptr pIn)
{
    cout << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "==================== APPROXIMATING USING LDR AND CDR ====================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    auto start = chrono::high_resolution_clock::now();
    
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
    
    pBTRModel->set_objective(pInNew->getObj());
    pBTRModel->set_ddu(pInNew);
    
    // do linear decision rule
    ROCPPOptModelIF_Ptr pLDRModel( m_pCVA->convertVar(pBTRModel, true) );
    
    // do constant decision rule
    ROCPPOptModelIF_Ptr pMiddle( m_pDVA->convertVar(pLDRModel, true) );
    
    if ( pMiddle->getNumAdaptiveVars() != 0 )
        throw MyException("Adaptive variables should have been eliminated by now");
    
    if (pIn->getObjType() == stochastic)
        pMiddle->getExpectation();
    
    ROCPPUncSSOptModel_Ptr pRob( convertToUSSOM(pMiddle) );
    
    // robustify pRob
    // generate the constraint with cone. robust MBLP->standard MBLP
    ROCPPRobustifyEngine_Ptr m_pRE (new RobustifyEngine());
    ROCPPBilinMISOCP_Ptr pRobustTmp( m_pRE->robustify(pRob)  );
    
    // Eliminate integer terms appearing in bilinearities
    ROCPPOptModelIF_Ptr pOutTmp2( m_pMItoMB_Bilinear->convertVar( ROCPPOptModelIF_Ptr(pRobustTmp) ) );
    
    // then, eliminate bilinearities between binary and other terms
    ROCPPOptModelIF_Ptr pOutTmp3( m_pBTR->linearize(pOutTmp2) );
    
    // convert resulting problem to MISOCP
    ROCPPMISOCP_Ptr pOut( convertToMISOCP(pOutTmp3) );
    
    if(pIn->isDDUOptimizationModel() )
    {
        ROCPPDDUOptModel_Ptr pIn_DDU( static_pointer_cast<DDUOptimizationModel>(pIn));
        
        ROCPPLinearDR_Ptr pLDR(static_pointer_cast<LinearDecisionRule>(m_pCVA));
        
        // ---------------- DECISION-DEPENDENT NON-ANTICIPATIVITY CONSTRAINTS -----------------------------------------------
        
        // |Y_{t,ij}| <= M x_{t-1,j} \forall i,j,t
        for (OneToExprVariableConverterIF::const_iterator tmldr_it=pLDR->begin(); tmldr_it!=pLDR->end(); tmldr_it++)
        {
            ROCPPVarIF_Ptr odv( pIn_DDU->getVar( tmldr_it->first ) );//Y_{t,i}
            
            for (DDUOptimizationModel::dduIterator ddu_it = pIn_DDU->dduBegin(); ddu_it != pIn_DDU->dduEnd(); ddu_it++)
            {
                ROCPPVarIF_Ptr mv( pIn_DDU->getMeasVar(ddu_it->first,odv->getTimeStage()-1) );//x_{t-1, j}
                
                ROCPPVarIF_Ptr ldrCoeff ( pLDR->getCoeffDV( odv->getName(),ddu_it->second->getName()) );//Y_{t, ij}
                
                // add non-anticipativity constraints
                ROCPPConstraint_Ptr pConstraint1( new IneqConstraint(false,true) );
                pConstraint1->add_lhs(1.,ldrCoeff);
                pConstraint1->add_lhs(-1.*m_bigM,mv);
                pConstraint1->set_rhs(make_pair(0.,true));
                pOut->add_constraint(pConstraint1);
                
                ROCPPConstraint_Ptr pConstraint2( new IneqConstraint(false,true) );
                pConstraint2->add_lhs(-1.,ldrCoeff);
                pConstraint2->add_lhs(-1.*m_bigM,mv);
                pConstraint2->set_rhs(make_pair(0.,true));
                pOut->add_constraint(pConstraint2);
            }
        }
    }
    auto stop = chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<chrono::seconds>(stop - start);
    
    cout << endl;
    cout << "Total time to approximate and robustify: " << duration.count() << " seconds" << endl;
    cout << "=========================================================================== " << endl;
    cout << endl;
    
    
    return pOut;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string LDRCDRApproximator::getSolutionApproachParameters(string delimiter) const
{
    string out("LDR & CDR");
    
    return out;
}

void LDRCDRApproximator::printParametersToScreen() const
{
    cout << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "========================== APPROXIMATION PARAMS =========================== " << endl;
    cout << "=========================================================================== " << endl;
    cout << "=========================================================================== " << endl;
    
    cout << "Solution Approach: LDR & CDR" << endl;
    
    cout << "=========================================================================== " << endl;
    cout << endl;
}

void LDRCDRApproximator::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &variableValue, ROCPPVarIF_Ptr dv)
{
    if(dv->isRealVar())
        m_pCVA->printOut(pIn, variableValue, dv);
    else
        m_pDVA->printOut(pIn, variableValue, dv);
}

void LDRCDRApproximator::printOut(const ROCPPOptModelIF_Ptr pIn, const map<string, double> &resultIn, ROCPPUnc_Ptr unc)
{
    uint t;
    string name = unc->getName();
    
    if (!pIn->isDDU(name))
        throw MyException("Uncertain parameter "+ name + " does not have a time of revelation that is decision-dependent");
    
    ROCPPVarIF_Ptr meas;
    //bool value;
    double value;
    
    for(t = 1; t < pIn->getNumTimeStages(); t++)
    {
        meas = pIn->getMeasVar(name, t);
        //???:print out
        //value = lexical_cast<bool>(resultIn.find(meas->getName())->second);
        value = resultIn.find(meas->getName())->second;
        bool observed;
        if (abs(value - 1.0) <= 0.01)
            observed = true;
        else if (abs(value - 0.0) <= 0.01)
            observed = false;
        else
            throw MyException("Wrong result for boolean variable.");
        
        if(observed){
            cout << "Uncertain parameter " << name << " is observed at stage " << t << endl;
            return;
        }
    }
    
    cout << "Uncertain parameter " << name << " is never observed" << endl;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% TOOL FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void createAllPartitionsKadapt(vector<vector<uint> > &BPconfigs, uint T, const map<uint, uint> &numPartitionsMap)
{
    BPconfigs.clear();
    
    
    uint t=1;
    
    map<uint, uint>::const_iterator it = numPartitionsMap.find(t);
    if (it==numPartitionsMap.end())
        throw MyException("could not find time stage 1 in map for ");
    
    for (uint candidate=1; candidate <= it->second; candidate++)
    {
        vector<uint>tmp;
        tmp.push_back(candidate);
        BPconfigs.push_back(tmp);
    }
    
    while (t < T) {
        
        t = t+1;
        appendPartitionsKadapt(BPconfigs, t, numPartitionsMap);
        
    }
}

void appendPartitionsKadapt(vector<vector<uint> > &BPconfigs, uint t, const map<uint, uint> &numPartitionsMap)
{
    map<uint, uint>::const_iterator it = numPartitionsMap.find(t);

    vector<vector<uint> > BPconfigsTmp;

    for (vector<vector<uint> >::iterator vit = BPconfigs.begin(); vit != BPconfigs.end(); vit++)
    {
        for (uint candidate=1; candidate <= it->second; candidate++)
        {
            vector<uint> cvec = *vit;
            cvec.push_back(candidate);
            BPconfigsTmp.push_back(cvec);
        }
    }

    BPconfigs = BPconfigsTmp;

}
