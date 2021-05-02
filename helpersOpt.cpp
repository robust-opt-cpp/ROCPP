//
//  helpersOpt.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "Exceptions.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "ObjectiveFunction.hpp"
#include "OptimizationModel.hpp"
#include "ReformulationOrchestrator.hpp"
#include "VariableConverter.hpp"
#include "OptModelConverters.hpp"
#include "UncertaintyConverter.hpp"
#include "helpersOpt.hpp"
#include "SolverModeller.hpp"
#ifdef USE_GUROBI
#include "GurobiModeller.hpp"
#elif defined(USE_SCIP)
#include "SCIPModeller.hpp"
#endif

void findMarginalSupportUncertaintySet(ROCPPOptModelIF_Ptr pModelIn, map<string,pair<double,double> > &margSupp, ROCPPMItoMB_Ptr pMIMBConverter, const map<string,uint> &numPartitionsMap, string solver,bool onlyObsUnc, bool onlyUncInMap, const map<string,pair<double,double> >&outerApproxMargSupp)
{
    
    
    cout << "-------------------------------------------------------------------" << endl;
    cout << "-------------------------------------------------------------------" << endl;
    cout << "-------- Calculating marginal support of uncertainty set ----------" << endl;
    cout << "-------------------------------------------------------------------" << endl;
    cout << "-------------------------------------------------------------------" << endl;
    cout << endl;
    
    
    margSupp.clear();
    
    if (!pModelIn->isUncertainOptimizationModel())
        throw MyException("findMarginalSupportUncertaintySet would not work");
    
    ROCPPUncOptModel_Ptr pInUnc = static_pointer_cast<UncertainOptimizationModel>(pModelIn);
    
    ROCPPOptModelIF_Ptr pSuppProblem( new DeterministicOptimizationModel() );
    
    ROCPPOptModelIF_Ptr pModelTmp ( pMIMBConverter->convertVar(pInUnc) );

    ROCPPBTR_bigM_Ptr pBTR1( new BTR_bigM( "bl", "", 0 ) );
    
    ROCPPOptModelIF_Ptr pModel ( pBTR1->linearize(pModelTmp, outerApproxMargSupp) );
    
    // -----------------------------------------------------------------------------------------------------------------------------
    // build translation map for uncertainty converter
    map<string,ROCPPVarIF_Ptr>  translationMapUnc;
    
    // create a decision variable named after each of the uncertain parameters and add it to translation map
    for (UncertainOptimizationModel::uncertaintiesIterator u_it=pInUnc->uncertaintiesBegin(); u_it!=pInUnc->uncertaintiesEnd();u_it++)
    {
        ROCPPVarIF_Ptr uvar(new VariableDouble( (*u_it).second->getName() ) );
        translationMapUnc[u_it->second->getName()] = uvar;
    }
    
    map<string,ROCPPVarIF_Ptr>  translationMapVar;
    
    // create a non-adaptive variable of the same type as the variable in the original problem
    for (OptimizationModelIF::varsIterator v_it=pModel->varsBegin(); v_it!=pModel->varsEnd();v_it++)
    {
        if (v_it->second->isBooleanVar())
        {
            ROCPPVarIF_Ptr var(new VariableBool( (*v_it).second->getName(), v_it->second->getLB(), v_it->second->getUB() ));
            translationMapVar[v_it->second->getName()] = var;
        }
        else if (v_it->second->isIntegerVar())
        {
            ROCPPVarIF_Ptr var(new VariableInt( (*v_it).second->getName(),  v_it->second->getLB(), v_it->second->getUB() ));
            translationMapVar[v_it->second->getName()] = var;
        }
        else if (v_it->second->isRealVar())
        {
            ROCPPVarIF_Ptr var(new VariableDouble( (*v_it).second->getName(),  v_it->second->getLB(), v_it->second->getUB() ));
            translationMapVar[v_it->second->getName()] = var;
        }
        else
            throw MyException("unknown decision variable type in findMarginalSupp");
    }
    
    UncToVariableConverter U2Vconverter( translationMapUnc );
    PredefO2OVariableConverter V2Vconverter( translationMapVar );
    
    // -----------------------------------------------------------------------------------------------------------------------------
    // find the variables that affect the uncertainty set
    dvContainer dvcUS;
    pInUnc->getVarsAffectingUncSet(dvcUS);
    
    // iterate through the deterministic constraints in the model and find any variables involved in constraints that involve variables that are involved in the uncertainty set
    dvContainer dvsToAdd;
    for (OptimizationModelIF::constraintIterator cit = pInUnc->constraintBegin(); cit != pInUnc->constraintEnd(); cit++)
    {
        if ( (!(*cit)->definesUncertaintySet()) && ( (*cit)->isDeterministic() ) )
        {
            
            if ( (*cit)->AnyVarIsInvolved(dvcUS) )
                dvsToAdd += *((*cit)->getDVContainer());
        }
    }
    
    dvsToAdd += dvcUS;
    
    for (OptimizationModelIF::constraintIterator cit = pInUnc->constraintBegin(); cit != pInUnc->constraintEnd(); cit++)
    {
        if ( (!(*cit)->definesUncertaintySet()) && ( (*cit)->isDeterministic() ) )
        {
            if ( (*cit)->AnyVarIsInvolved(dvsToAdd) )
                pSuppProblem->add_constraint( V2Vconverter.convertVar(*cit) );
        }
    }
    
    for (OptimizationModelIF::constraintIterator c_it = pModel->constraintBegin(); c_it != pModel->constraintEnd(); c_it++)
    {
        if ( ((*c_it)->definesUncertaintySet()) )
        {
            // create new constraint from this constraint but that does not define uncertainty set
            if (!(*c_it)->isClassicConstraint())
                throw MyException("uncertainty set should only contain classic constraints");
            
            
            ROCPPConstraintIF_Ptr pTmpCstr ( U2Vconverter.convert( *c_it ) );
            ROCPPConstraintIF_Ptr pTmpCstr2 ( V2Vconverter.convertVar(pTmpCstr) );
            
            
            if (!pTmpCstr2->isClassicConstraint())
                throw MyException("this should not have happened");
            
            
            ROCPPClassicConstraint_Ptr pClassic ( static_pointer_cast<ClassicConstraintIF>(pTmpCstr2) );
            
            ROCPPClassicConstraint_Ptr pToAdd;
            if (pClassic->isEqConstraint())
                pToAdd = ROCPPClassicConstraint_Ptr( new EqConstraint() );
            else
                pToAdd = ROCPPClassicConstraint_Ptr( new IneqConstraint(false) );
            
            pToAdd->set_rhs( pClassic->get_rhs() );
            pToAdd->add_lhs( pClassic->getLHS() );
            
            pSuppProblem->add_constraint(pToAdd);
        }
    }
    
    // also add bounds using the outer approximation of the uncertainty set; map uncertainties to variables in the process
    for (map<string,pair<double,double> >::const_iterator oait = outerApproxMargSupp.begin(); oait != outerApproxMargSupp.end(); oait++)
    {
        // try to find the uncertainty mapped as a decision variable
        UncertaintyConverterIF::const_iterator uit = U2Vconverter.find(oait->first);
        // if is was found
        if (uit != U2Vconverter.end())
        {
            ROCPPClassicConstraint_Ptr pCstrl(new IneqConstraint());
            ROCPPClassicConstraint_Ptr pCstrg(new IneqConstraint());
            
            ROCPPExpr_Ptr pMapped( uit->second );
            ROCPPExpr_Ptr pMappedNeg( pMapped->Clone() );
            *pMappedNeg *= -1.;
            
            pCstrl->add_lhs( pMapped );
            pCstrg->add_lhs( pMappedNeg );
            
            pCstrl->set_rhs( make_pair(oait->second.second, false) );
            pCstrg->set_rhs( make_pair(-1.*oait->second.first, false) );
            
            pSuppProblem->add_constraint(pCstrl);
            pSuppProblem->add_constraint(pCstrg);
            
        }
    }
    
    // convert mixed-integer bilinear problem to mixed-binary bilinear
    ROCPPOptModelIF_Ptr pSuppProblem2 = pMIMBConverter->convertVar(pSuppProblem);
    
    ROCPPBTR_bigM_Ptr pBTR2( new BTR_bigM( "bl", "", pBTR1->getAuxVarCnt() ) );
    ROCPPOptModelIF_Ptr pSuppProblem3( pBTR2->linearize(pSuppProblem2,outerApproxMargSupp) );
    
    // covert to MISOCP and subsequently to CPLEXMISOCP
    ROCPPMISOCP_Ptr pSuppProblem4 ( convertToMISOCP( pSuppProblem3 ) );
    
    ROCPPCPLEXMISOCP_Ptr pSuppProblem5 ( new CPLEXMISOCP( pSuppProblem4 ) );
    
    // iterate one by one through the decision variables of the new problem (uncertain parameters of old problem)
    for (UncertainOptimizationModel::uncertaintiesIterator u_it=pInUnc->uncertaintiesBegin(); u_it!=pInUnc->uncertaintiesEnd();u_it++)
    {
        
        // find the number of breakpoints in this direction
        map<string,uint>::const_iterator numBP_it( numPartitionsMap.find(u_it->second->getName()) );
        if ( (!onlyUncInMap) && (numBP_it==numPartitionsMap.end()) )
            throw MyException("unc not found in numPartitionsMap");
        
        
        if ( ( onlyUncInMap && (numBP_it!=numPartitionsMap.end()) ) || (!onlyUncInMap) )
        {
            bool tmp1( onlyObsUnc && (u_it->second->isObservable()) );
            bool tmp2( !onlyObsUnc );
            
            
            if ( ( tmp1 || tmp2 ) && (numBP_it->second>1) )
            {
                /*
                cout << "-------------------------------------------------------------------" << endl;
                cout << "-------------------------------------------------------------------" << endl;
                cout << "Calculating marginal support for " << u_it->second->getName() << endl;
                cout << "-------------------------------------------------------------------" << endl;
                cout << "-------------------------------------------------------------------" << endl;
                cout << endl;
                */
                
                
                SolverParams sparams = SolverParams();
                ROCPPSolver_Ptr pModeller;
//                if (solver == "gurobi")
//                    pModeller =  ROCPPSolver_Ptr(new GurobiModeller(sparams, false) );
//                else if (solver == "SCIP")
//                    pModeller =  ROCPPSolver_Ptr(new SCIPModeller(sparams, false) );
//                else
//                    throw MyException("Can not find your solver.");
#ifdef USE_GUROBI
                pModeller =  ROCPPSolver_Ptr(new GurobiModeller(sparams, false) );
#elif defined(USE_SCIP)
                pModeller =  ROCPPSolver_Ptr(new SCIPModeller(sparams, false) );
#else
                throw MyException("Can not find your solver.");
#endif
                
                double opt_lb;
                uint status_lb;
                double opt_ub;
                uint status_ub;
                
                // check if the variable is defined in pSuppProblem5
                if (pSuppProblem5->varIsDefined(u_it->second->getName()))
                {
                    ROCPPVarIF_Ptr cvarunc = pSuppProblem5->getVar(u_it->second->getName());
                    
                    
//                    cout << "Calculating marginal support for " << u_it->second->getName() << ": lower bound" << endl;
//                    cout << "-------------------------------------------------------------------" << endl;
                    
                    ROCPPExpr_Ptr uncObjPos(new LHSExpression() );
                    uncObjPos->add(1.0, cvarunc);
                    ROCPPObjectiveIF_Ptr ObjPos(new SimpleObjective(uncObjPos) );
                    pSuppProblem5->set_objective(ObjPos);
                    
                    pModeller->solve(pSuppProblem5,false,"",false);
                    //pModeller->solve(pSuppProblem5);
                    status_lb = pModeller->getOptStatus();
                    opt_lb = pModeller->getOptValue();
//                    cout << "Calculating marginal support for " << u_it->second->getName() << ": upper bound" << endl;
//                    cout << "-------------------------------------------------------------------" << endl;
                    
                    ROCPPExpr_Ptr uncObjNeg(new LHSExpression() );
                    uncObjNeg->add(-1.0, cvarunc);
                    ROCPPObjectiveIF_Ptr ObjNeg(new SimpleObjective(uncObjNeg) );
                    pSuppProblem5->set_objective(ObjNeg);
                    
                    pModeller->solve(pSuppProblem5,false,"",false);
                    //pModeller->solve(pSuppProblem5);
                    status_ub = pModeller->getOptStatus();
                    opt_ub = pModeller->getOptValue();
                }
                // if it isn't, throw
                else
                {
                    throw MyException("no more positive and negative parts: it should have been found");
                }
                
                opt_ub *= -1.0;
#ifdef USE_GUROBI
                if ( (status_lb == GRB_OPTIMAL) && (status_ub == GRB_OPTIMAL) )
                {
                    if (opt_lb > opt_ub )
                        throw MyException("an unexpected error occurred when computing the marginal supports");
                    
                    margSupp.insert( pair<string,pair<double,double> >( u_it->second->getName() ,pair<double,double>( opt_lb, opt_ub ) ));
                }
                else
                    throw MyException("an error occured when finding the marginal supports: either empty or unbounded");
#elif defined(USE_SCIP)
                if ( (status_lb == SCIP_STATUS_OPTIMAL) && (status_ub == SCIP_STATUS_OPTIMAL) )
                {
                    if (opt_lb > opt_ub )
                        throw MyException("an unexpected error occurred when computing the marginal supports");
                    
                    margSupp.insert( pair<string,pair<double,double> >( u_it->second->getName() ,pair<double,double>( opt_lb, opt_ub ) ));
                }
                else
                    throw MyException("an error occured when finding the marginal supports: either empty or unbounded");
#endif
            }
        }
    }
    
    cout << "Done calculating the marginal support." << endl;
}


void findWholeMarginalSupport(ROCPPOptModelIF_Ptr pModelIn, const map<string,uint> &numPartitionsMap, map<string,pair<double,double> > &margSupp)
{
    ROCPPMItoMB_Ptr pMIMBConverter( new BinaryConverter() );
    
    map<string,uint> numPartitionsMap2;
    
    for (ObjectiveFunctionIF::uncsIterator u_it = pModelIn->getObj()->uncBegin(); u_it != pModelIn->getObj()->uncEnd(); u_it++ )
    {
        if ( (numPartitionsMap.find(u_it->first) == numPartitionsMap.end()) || (numPartitionsMap.find(u_it->first)->second <= 1))
            numPartitionsMap2.insert(make_pair(u_it->second->getName() , 2));
        else
            numPartitionsMap2.insert(make_pair(u_it->second->getName() , 1));
    }
    
    
    findMarginalSupportUncertaintySet(pModelIn, margSupp, pMIMBConverter,numPartitionsMap2,"gurobi",false,true);
}

double calculateArea(const map<string,pair<double,double> > &allMap)
{
    double area(1.0);
    
    map<string,pair<double,double> >::const_iterator m_it(allMap.begin());
    for (; m_it != allMap.end(); m_it++) {
        if (DoublesAreEssentiallyEqual(m_it->second.second, m_it->second.first, 0.0001))
            continue;
        area *= (m_it->second.second - m_it->second.first);
    }
    
    return area;
}
