//
//  SCIPModeller.cpp
//  ROCPP
//
//  Created by 靳晴 on 1/16/21.
//
#ifdef USE_SCIP
#include "SCIPModeller.hpp"
#include "Exceptions.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "ObjectiveFunction.hpp"
#include "OptimizationModel.hpp"
#include "OptModelConverters.hpp"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%% SCIP MODELLER %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SCIPModeller::SCIPModeller(const SolverParams &pSParams, bool useLazyNACs) : SolverModellerIF(pSParams) {}

SCIPModeller::SCIPModeller() : SolverModellerIF(SolverParams()) {}

void SCIPModeller::Reset()
{
    m_pSCIPVC.Reset();
    m_pCstr.clear();
    m_problemSolved = false;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Getter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double SCIPModeller::getMIPGap() const
{
    if(!m_problemSolved)
        throw MyException("No problem is solved now");
    
    if(m_results.m_isMIP){
        cout << "Warning: No such result in SCIP." << endl;
        return 0.0;
    }
    else{
        cout << "No integer variable in this problem." << endl;
        return 0.0;
    }
}

double SCIPModeller::getOptValue() const
{
    if(getOptStatus() == 11)
        return m_results.m_optValue;
    else {
        cout << "No optimal value in status: " + to_string(getOptStatus() ) << endl;
        return 0.0;
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void SCIPModeller::solve(boost::shared_ptr<OptimizationModelIF> pModelIn, bool writeSlnToFile, string fileName, bool writeSlnToConsle, const map<string, double>& WSvars, const map<string,int>& priorities, bool deleteModel)
{
    Reset();
    
    boost::shared_ptr<CPLEXMISOCP> pCplex;
    
    if(pModelIn->getType() == cplexmisocpType)
        pCplex = boost::dynamic_pointer_cast<CPLEXMISOCP>(pModelIn);
    
    else{
        boost::shared_ptr<MISOCP> pInNew(new MISOCP());
        
        if(pModelIn->getType() == misocpType){
            pInNew = boost::dynamic_pointer_cast<MISOCP>(pModelIn);
            pCplex = boost::shared_ptr<CPLEXMISOCP>( new CPLEXMISOCP(pInNew) );
        }
        
        else{
            throw MyException("Wrong model type, please use the function convertToMISOCP() to convert and check your model before solving");
        }
        
    }
    
    solveSCIPModel(pCplex, writeSlnToConsle);
    
    // other stuff
    if (deleteModel)
        delete pModelIn.get();
    
    m_problemSolved = true;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%% Setter Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool SCIPModeller::setParameters(SCIP* scip) const
{
    if (getEpAGapLimit().first)
        SCIP_CALL( SCIPsetRealParam(scip, "limits/absgap", getEpAGapLimit().second) );

    if (getEpOptLimit().first)
        SCIP_CALL( SCIPsetRealParam(scip, "numerics/dualfeastol", getEpOptLimit().second) );
    
    if (getEpIntLimit().first)
        throw MyException("Not sure about this in SCIP, please set to default");
    
    if (getTimeLimit().first)
        SCIP_CALL( SCIPsetRealParam(scip, "limits/time", getTimeLimit().second) );
    
    if (getEpGapLimit().first)
        SCIP_CALL( SCIPsetRealParam(scip, "limits/gap", getEpGapLimit().second) );
    
    if (getEpRHSLimit().first)
        SCIP_CALL( SCIPsetRealParam(scip, "numerics/feastol", getEpRHSLimit().second) );
    
    return true;
}

bool SCIPModeller::solveSCIPModel(boost::shared_ptr<CPLEXMISOCP> pCplex, bool writeSlnToConsle)
{
    //Setting up the SCIP environment
    SCIP* scip = nullptr; /* Declaring the scip environment*/
    SCIP_CALL( SCIPcreate(&scip) ); /*Creating the SCIP environment */
    SCIP_CALL( SCIPincludeDefaultPlugins(scip) ); /* include default plugins */
    SCIP_CALL( SCIPcreateProbBasic(scip, "myModel") ); /* creating the SCIP Problem. */
    SCIP_CALL( SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE) );/* Minimization problem by default */
    

    cout << "Creating model" << endl;
    createSCIPmodel(pCplex, scip);
    
    // set parameter
    setParameters(scip);
    
    // set display
    if(!writeSlnToConsle)
        SCIP_CALL( SCIPsetIntParam(scip, "display/verblevel", 0) );
    else
        SCIP_CALL( SCIPsetIntParam(scip, "display/verblevel", 1) );
    
    // solve
    cout << "start solving" << endl;
    SCIP_CALL( SCIPsolve(scip) );
    cout << "solved" << endl;
    // get solution status
    SCIP_STATUS solutionstatus = SCIPgetStatus( scip );
    // get solution
    SCIP_SOL* sol;
    sol = SCIPgetBestSol( scip );
    
    // get solving time
    double realSolveTime( SCIPsolGetTime(sol));
    cout << "Solver time: " << realSolveTime/60 << " minutes" << endl;
    m_results.m_solveTime = realSolveTime;
    
    // output results
    m_results.m_optStatus = solutionstatus;
    if (solutionstatus == SCIP_STATUS_OPTIMAL)
    {
        cout << "\nOpt: " << SCIPgetSolOrigObj(scip, sol) << endl;
        cout << endl;
        m_results.m_optValue = SCIPsolGetOrigObj(sol);
    }
    else{
        cout << "\nStatus: " << solutionstatus << endl;
        m_results.m_optValue = SCIPsolGetOrigObj(sol);
    }
    
    
    if ( (m_pSCIPVC.m_boolVarMap.size() + m_pSCIPVC.m_intVarMap.size() ) > 0)
        m_results.m_isMIP = true;
    else
        m_results.m_isMIP = false;
    
    m_solution.clear();
    
    // get solution of variables
    map<string,SCIP_Var*>::iterator var(m_pSCIPVC.m_allVarsMap.begin());
    
    for(; var != m_pSCIPVC.m_allVarsMap.end(); var++)
    {
        pair<string, double> tmp;
        tmp.first = var->first;
        tmp.second = SCIPgetSolVal(scip, sol, var->second);
        m_solution.insert(tmp);
        SCIP_CALL( SCIPreleaseVar(scip, &(var->second)));
    }
    
    for( auto &cstr : m_pCstr )
    {
       SCIP_CALL( SCIPreleaseCons(scip, &cstr) );
    }
    Reset();
    
    SCIP_CALL( SCIPfree(&scip) );
    
    return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% Creater Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool SCIPModeller::createSCIPmodel(boost::shared_ptr<CPLEXMISOCP> pModel, SCIP* scip)
{
    // ================================================================================
    // =========== CREATE VARIABLES ===================================================
    // ================================================================================
    
    addSCIPdecisionVars(pModel, scip);
    
    // ================================================================================
    // =========== CREATE CONSTRAINTS =================================================
    // ================================================================================
    
    uint cnt(0);
    for (OptimizationModelIF::constraintIterator c_it=pModel->constraintBegin(); c_it != pModel->constraintEnd(); c_it++)
    {
        cnt++;
        if ( cnt % 100 == 0)
            cout << cnt << " constraints modelled from " << pModel->getNumConstraints() << endl;
        
        addConstraint( scip, *c_it, cnt);
    }
    return true;
}

bool SCIPModeller::addSCIPdecisionVars(boost::shared_ptr<CPLEXMISOCP> pModel, SCIP* scip)
{
    // SCIP can only solve MILP problem with linear objective function
    boost::shared_ptr<LHSExpression> obj(pModel->getObj()->getObj(1));
    
    if(!obj->isLinear())
        throw MyException("SCIP can only solve MILP problem with linear objective function");
    
    map<string, double> varCoeff;
    double objOffSet(0.0);
    // iterate through the terms and find the coefficient of the decision variables
    LHSExpression::const_iterator t_it = obj->begin();
    for(; t_it != obj->end(); t_it++)
    {
        if((*t_it)->getNumVars() == 0)
            objOffSet += (*t_it)->getCoeff();
        
        string name((*t_it)->getDVContainer()->begin()->first);
        varCoeff.insert(make_pair(name, (*t_it)->getCoeff()));
    }
    
    double coeff;
    map<string, double>::const_iterator c_it;
    for (OptimizationModelIF::varsIterator v_it=pModel->varsBegin(); v_it != pModel->varsEnd(); v_it++)
    {
        // find the coefficient of the decision variables
        c_it = varCoeff.find(v_it->first);
        if(c_it != varCoeff.end())
            coeff = c_it->second;
        else
            coeff = 0.0;
        
        double lb( v_it->second->getLB());
        double ub( v_it->second->getUB());
        
        SCIP_VAR* var = nullptr;
        if (v_it->second->getType() == contDV)
        {
            SCIP_CALL(SCIPcreateVarBasic(scip, &var, v_it->second->getName().c_str(), lb, ub, coeff, SCIP_VARTYPE_CONTINUOUS));
            SCIP_CALL(SCIPaddVar(scip, var));
            
            m_pSCIPVC.m_contVarMap.insert( make_pair( (v_it->second->getName()), var ) );
            m_pSCIPVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), var ) );
        }
        else if (v_it->second->getType() == boolDV)
        {
            SCIP_CALL(SCIPcreateVarBasic(scip, & var, v_it->second->getName().c_str(), lb, ub, coeff, SCIP_VARTYPE_BINARY));
            SCIP_CALL(SCIPaddVar(scip, var));
            
            m_pSCIPVC.m_boolVarMap.insert( make_pair( (v_it->second->getName()), var ) );
            m_pSCIPVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), var ) );
        }
        else if (v_it->second->getType() == intDV)
        {
            SCIP_CALL(SCIPcreateVarBasic(scip, & var, v_it->second->getName().c_str(), lb, ub, coeff, SCIP_VARTYPE_INTEGER));
            SCIP_CALL(SCIPaddVar(scip, var));
            
            m_pSCIPVC.m_intVarMap.insert( make_pair( (v_it->second->getName()), var ) );
            m_pSCIPVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), var ) );
        }
        else throw MyException("unexpected type");
    }
    
    if(objOffSet != 0.0)
        cout << "The offset of the objective function is not counted" << endl;
    return true;
}

bool SCIPModeller::addConstraint(SCIP* scip, boost::shared_ptr<ConstraintIF> pCstrIn, uint num)
{
    if ( pCstrIn->isClassicConstraint() )
    {
        boost::shared_ptr<ClassicConstraintIF> pClassic ( boost::static_pointer_cast<ClassicConstraintIF> (pCstrIn) );
        
        SCIP_CONS* cons = nullptr;
        
        if (pClassic->isLinear())
        {
            // decide the lhs and the rhs
            double lhs, rhs((pClassic->get_rhs()).first);
            if (pClassic->isEqConstraint())
                lhs = rhs;
            else
                lhs = -SCIPinfinity(scip);
            
            // set name of the constraint base on the number
            string name("cstr" + to_string(num));
            SCIP_CALL( SCIPcreateConsBasicLinear(scip, &cons, name.c_str(),
                                                  0, nullptr, nullptr, lhs, rhs) );
            
            for (ClassicConstraintIF::const_iterator tit = pClassic->begin(); tit != pClassic->end(); tit++)
            {
                if (!(*tit)->isProductTerm() )
                    throw MyException("cplexmisocp cannot have non-prod terms");
                
                boost::shared_ptr<ProductTerm> pPT = boost::static_pointer_cast<ProductTerm>(*tit);
                if (pPT->getNumUncertainties() != 0)
                    throw MyException("cplexmisocp should not involve uncertainties");

                if (pPT->getNumVars()==0)
                    rhs -= pPT->getCoeff();
                
                else
                {
                    map<string, SCIP_VAR*>::const_iterator vit (m_pSCIPVC.m_allVarsMap.find( pPT->varsBegin()->second->getName()));
                    if (vit == m_pSCIPVC.m_allVarsMap.end())
                        throw MyException("SCIPVar not found");
                    
                    SCIP_Var* var1 = vit->second;
                    
                    if ( pPT->isLinear() )
                        SCIP_CALL( SCIPaddCoefLinear(scip, cons, var1, pPT->getCoeff()) );
                    else throw ("unacceptable term type");
                }
            }
            
            if (pClassic->isEqConstraint())
                lhs = rhs;
            SCIP_CALL( SCIPchgRhsLinear(scip, cons, rhs));
            SCIP_CALL( SCIPchgLhsLinear(scip, cons, lhs));
            
            SCIP_CALL( SCIPaddCons(scip, cons) );
            
            m_pCstr.push_back(cons);
        }
        else if (pClassic->isQuadratic()){
            if(SCIPgetNNlpis(scip) == 0)
                throw ("No avaliable NLP solver");
            // decide the lhs and the rhs
            double lhs, rhs((pClassic->get_rhs()).first);
            if (pClassic->isEqConstraint())
                throw MyException("cannot have quadratic equality");
            else
                lhs = -SCIPinfinity(scip);
            
            // set name of the constraint base on the number
            string name("cstr" + to_string(num));
            SCIP_CALL( SCIPcreateConsBasicQuadratic(scip, &cons, name.c_str(),
                                                  0, nullptr, nullptr, 0, nullptr, nullptr, nullptr, lhs, rhs) );
            for (ClassicConstraintIF::const_iterator tit = pClassic->begin(); tit != pClassic->end(); tit++)
            {
                if (!(*tit)->isProductTerm() )
                    throw MyException("cplexmisocp cannot have non-prod terms");
                
                boost::shared_ptr<ProductTerm> pPT = boost::static_pointer_cast<ProductTerm>(*tit);
                if (pPT->getNumUncertainties() != 0)
                    throw MyException("cplexmisocp should not involve uncertainties");

                if (pPT->getNumVars()==0)
                    SCIPaddConstantQuadratic(scip, cons, pPT->getCoeff() );
                else{
                    map<string, SCIP_VAR*>::const_iterator vit;
                    vit = m_pSCIPVC.m_allVarsMap.find( pPT->varsBegin()->second->getName());
                    if (vit == m_pSCIPVC.m_allVarsMap.end())
                        throw MyException("SCIPVar not found");
                    
                    SCIP_Var* var1 = vit->second;
                    if(pPT->isLinear())
                    {
                        SCIP_CALL( SCIPaddLinearVarQuadratic(scip, cons, var1, pPT->getCoeff()) );
                    }
                    else if ( pPT->isQuadratic() )
                    {
                        vit = m_pSCIPVC.m_allVarsMap.find( (++pPT->varsBegin())->second->getName());
                        if (vit == m_pSCIPVC.m_allVarsMap.end())
                            throw MyException("GRBVar not found");
                        
                        SCIP_Var* var2 = vit->second;
                        SCIP_CALL( SCIPaddBilinTermQuadratic(scip, cons, var1, var2, pPT->getCoeff()) );
                    }
                    else throw ("unacceptable term type");
                }
            }
            SCIP_CALL( SCIPaddCons(scip, cons) );
            m_pCstr.push_back(cons);
        }
        else
           throw MyException("unknown constraint type");
    }
    else if ( pCstrIn->isSOSConstraint() )
    {
        throw MyException("not supported in Gurobi modeller at this time");
    }
    else if ( pCstrIn->isIfThenConstraint() )
    {
        throw MyException("If then constraints not supported by Gurobi");
    }
    else throw MyException("unknown constraint type");
    
    return true;
}

#endif
