//
//  GurobiModeller.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#include "IncludeFiles.hpp"
#include "Exceptions.hpp"
#include "DecisionVariable.hpp"
#include "Uncertainty.hpp"
#include "ConstraintTerm.hpp"
#include "Constraint.hpp"
#include "ObjectiveFunction.hpp"
#include "OptimizationModel.hpp"
#include "OptModelConverters.hpp"
#include "GurobiModeller.hpp"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%% GUROBI MODELLER %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% Constructors & Destructors %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

GurobiModeller::GurobiModeller(const SolverParams &pSParams, bool useLazyNACs) : SolverModellerIF(pSParams) {}

GurobiModeller::GurobiModeller() : SolverModellerIF(SolverParams()) {}

void GurobiModeller::Reset()
{
    m_pGurobiVC.Reset();
    m_problemSolved = false;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% Doer Functions %%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void GurobiModeller::solve(boost::shared_ptr<OptimizationModelIF> pModelIn, bool writeSlnToFile,  string fileName,  bool writeSlnToConsole, const map<string, double>& WSvars, const map<string,int>& priorities, bool deleteModel)
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
    
    try
    {
        GRBEnv env = GRBEnv();
        
        // create decision variables and model
        
        GRBModel model = GRBModel(env);
        
        if(writeSlnToConsole){
            cout << "Adding decision variables" << endl;
            addGUROBIdecisionVars(pCplex, env, model);
            cout << "Done adding decision variables" << endl;
            
            cout<<"Creating GUROBI model..."<<endl;
            createGUROBImodel(pCplex,env,model);
            cout<<"Done creating GUROBI model"<<endl;
        }
        else
        {
            model.set(GRB_IntParam_LogToConsole, 0);
            addGUROBIdecisionVars(pCplex, env, model);
            createGUROBImodel(pCplex,env,model);
        }
        
        model.update();
        
        if(WSvars.size())
            addGUROBIwarmstart(WSvars);
        
        // set priorities
        setPriorities(m_pGurobiVC.m_allVarsMap, priorities);
        
        // set parameters
        setParameters(model);
        
        // other stuff
        if (deleteModel)
            delete pModelIn.get();
        
        
        if (writeSlnToFile)
            model.write(fileName+".lp");
        
        
        // solve
        
        model.optimize();
        double realSolveTime( model.get( GRB_DoubleAttr_Runtime ) );
        cout << "Solver time: " << realSolveTime/60 << " minutes" << endl;
        
        
        // output results
        
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
        {
            cout << "\n Opt: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
            cout << endl;
        }
        else
            cout << "\n Status: " << model.get(GRB_IntAttr_Status) << endl;
        
        
        if ( (model.get(GRB_IntAttr_NumIntVars) + model.get(GRB_IntAttr_NumBinVars) )>0){
            m_results.m_isMIP = true;
            m_results.m_MIPGap = model.get(GRB_DoubleAttr_MIPGap);
        }
        else{
            m_results.m_isMIP = false;
            m_results.m_MIPGap = 0.;
        }
        
        m_results.m_optStatus = model.get(GRB_IntAttr_Status);
        m_results.m_optValue = model.get(GRB_DoubleAttr_ObjVal);
        m_results.m_solveTime = realSolveTime;
        
        saveResults(model, make_pair(writeSlnToFile, fileName));
        
        GRBVar* allVar = model.getVars();
        int numVar = model.get(GRB_IntAttr_NumVars);
        int i;
        
        m_solution.clear();
        
        for(i = 0; i < numVar; i++)
        {
            pair<string, double> tmp;
            tmp.first = allVar[i].get(GRB_StringAttr_VarName);
            tmp.second = allVar[i].get(GRB_DoubleAttr_X);
            m_solution.insert(tmp);
        }
    }
    catch(GRBException e){
        std::string err("Gurobi MyException caught: ");
        err += boost::lexical_cast<string>( e.getMessage() );
        cin.clear();
        cout << "Error (GurobiException): " << err << endl << "Press enter to exit" << endl;
        cin.ignore(1,0);
        exit( EXIT_FAILURE );
    }
    catch(MyException& e){
        std::string err("MyException caught: ");
        err+= boost::lexical_cast<string>( e.what() );
        cin.clear();
        cout<<"Error (std): " << err << endl << "Press enter to exit" << endl;
        cin.ignore(1,0);
        exit( EXIT_FAILURE );
    }
    
    m_problemSolved = true;

}

void GurobiModeller::setParameters(GRBModel &Model) const
{
    
    Model.getEnv().set(GRB_IntParam_Threads, getThreadsToRun());
    
    Model.getEnv().set(GRB_IntParam_OutputFlag,boost::lexical_cast<int>(m_pSParams.getVerbose()));
    
    if (getEpAGapLimit().first)
        Model.getEnv().set(GRB_DoubleParam_MIPGapAbs,getEpAGapLimit().second);
    
    if (getEpOptLimit().first)
        Model.getEnv().set(GRB_DoubleParam_OptimalityTol,getEpOptLimit().second);
    
    if (getEpRHSLimit().first)
        Model.getEnv().set(GRB_DoubleParam_FeasibilityTol,getEpRHSLimit().second);
    
    if (getEpIntLimit().first)
        Model.getEnv().set(GRB_DoubleParam_IntFeasTol,max(getEpIntLimit().second,1.e-9));
    
    if (getTimeLimit().first)
        Model.getEnv().set(GRB_DoubleParam_TimeLimit,getTimeLimit().second);
    
    if (getEpGapLimit().first)
        Model.getEnv().set(GRB_DoubleParam_MIPGap,getEpGapLimit().second);
}

void GurobiModeller::setPriorities(map<string,GRBVar>& VarMap, const map<string,int>& priorities)
{
    for (map<string,int>::const_iterator pit = priorities.begin(); pit != priorities.end(); pit++)
    {
        map<string,GRBVar>::iterator vit = VarMap.find(pit->first);
        
        if (vit == VarMap.end() )
            throw MyException("GRBVar not found in map");
        
        vit->second.set(GRB_IntAttr_BranchPriority,pit->second);
    }
}

void GurobiModeller::saveResults(GRBModel& model, pair<bool,string> writeSlnToFile) const
{
    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
    {
        if (writeSlnToFile.first)
        {
            model.write((writeSlnToFile.second+"All.sol").c_str());
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%% Creater Functions %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void GurobiModeller::createGUROBImodel(boost::shared_ptr<CPLEXMISOCP> pModel,GRBEnv &env,GRBModel &Model)
{
    // ================================================================================
    // =========== CREATE CONSTRAINTS =================================================
    // ================================================================================
    
    
    uint cnt(0);
    for (OptimizationModelIF::constraintIterator c_it=pModel->constraintBegin(); c_it != pModel->constraintEnd(); c_it++)
    {
        cnt++;
        if ( cnt % 10000 == 0)
            cout << cnt << " constraints modelled from " << pModel->getNumConstraints() << endl;
        
        addConstraint( env, Model, *c_it );
        
    }
    
    
    // ================================================================================
    // =========== CREATE OBJECTIVE ===================================================
    // ================================================================================
    
    addObjective(env, Model, pModel->getObj() );
    
}

void GurobiModeller::addGUROBIdecisionVars(boost::shared_ptr<CPLEXMISOCP> pModel, GRBEnv &env, GRBModel &Model)
{
    // ------- create variables --------------------------
    
    for (OptimizationModelIF::varsIterator v_it=pModel->varsBegin(); v_it != pModel->varsEnd(); v_it++)
    {
        if (v_it->second->getType() == contDV)
        {
            double lb( v_it->second->getLB() );
            double ub( v_it->second->getUB() );
            
            GRBVar x;
            
            if ( (lb > -INFINITY) && (ub < INFINITY) )
                x = Model.addVar( lb, ub, 0., GRB_CONTINUOUS,  v_it->second->getName().c_str() );
            else if ( ( lb == -INFINITY ) && (ub < INFINITY) )
                x = Model.addVar( -1.*GRB_INFINITY, ub, 0., GRB_CONTINUOUS,  v_it->second->getName().c_str() );
            else if ( ( lb > -INFINITY ) && (ub == INFINITY) )
                x = Model.addVar( lb, GRB_INFINITY, 0., GRB_CONTINUOUS,  v_it->second->getName().c_str() );
            else
                x = Model.addVar( -1.*GRB_INFINITY, GRB_INFINITY, 0., GRB_CONTINUOUS,  v_it->second->getName().c_str() );
            
            
            m_pGurobiVC.m_contVarMap.insert( make_pair( (v_it->second->getName()), x ) );
            m_pGurobiVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), x ) );
        }
        else if (v_it->second->getType() == boolDV)
        {
            double lb( v_it->second->getLB() );
            double ub( v_it->second->getUB() );
            
            GRBVar x;
            
            x = Model.addVar( lb, ub, 0., GRB_BINARY,  v_it->second->getName().c_str() );
            
            m_pGurobiVC.m_boolVarMap.insert( make_pair( (v_it->second->getName()), x ) );
            m_pGurobiVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), x ) );
        }
        else if (v_it->second->getType() == intDV)
        {
            double lb( v_it->second->getLB() );
            double ub( v_it->second->getUB() );
            
            GRBVar x;
                 
            if ( (lb > -INFINITY) && (ub < INFINITY) )
                x = Model.addVar( lb, ub, 0., GRB_INTEGER,  v_it->second->getName().c_str() );
            else if ( ( lb == -INFINITY ) && (ub < INFINITY) )
                x = Model.addVar( -1.*GRB_INFINITY, ub, 0., GRB_INTEGER,  v_it->second->getName().c_str() );
            else if ( ( lb > -INFINITY ) && (ub == INFINITY) )
                x = Model.addVar( lb, GRB_INFINITY, 0., GRB_INTEGER,  v_it->second->getName().c_str() );
            else
                x = Model.addVar( -1.*GRB_INFINITY, GRB_INFINITY, 0., GRB_INTEGER,  v_it->second->getName().c_str() );
            
            m_pGurobiVC.m_intVarMap.insert( make_pair( (v_it->second->getName()), x ) );
            m_pGurobiVC.m_allVarsMap.insert( make_pair( (v_it->second->getName()), x ) );
        }
        else throw MyException("unexpected type");
    }
    
}

void GurobiModeller::addConstraint(GRBEnv &env, GRBModel& Model,boost::shared_ptr<ConstraintIF> pCstrIn) const
{
    if ( pCstrIn->isClassicConstraint() )
    {
        boost::shared_ptr<ClassicConstraintIF> pClassic ( boost::static_pointer_cast<ClassicConstraintIF> (pCstrIn) );
        
        if (pClassic->isLinear())
        {
            uint useNAC(0);
            
            //With a value of 1, the constraint can be used to cut off a feasible solution, but it won't necessarily be pulled in if another lazy constraint also cuts off the solution. With a value of 2, all lazy constraints that are violated by a feasible solution will be pulled into the model. With a value of 3, lazy constraints that cut off the relaxation solution at the root node are also pulled in.
            if(pClassic->isNAC() && m_pSParams.useLazyNACs())
                useNAC = 2;
            
            GRBLinExpr lhs_expr=0.;
            
            for (ClassicConstraintIF::const_iterator tit = pClassic->begin(); tit != pClassic->end(); tit++)
            {
                if (!(*tit)->isProductTerm() )
                    throw MyException("cplexmisocp cannot have non-prod terms");
                
                boost::shared_ptr<ProductTerm> pPT = boost::static_pointer_cast<ProductTerm>(*tit);
                if (pPT->getNumUncertainties() != 0)
                    throw MyException("cplexmisocp should not involve uncertainties");
                
                
                if (pPT->getNumVars()==0)
                    lhs_expr += pPT->getCoeff() ;
                else
                {
                    map<string,GRBVar>::const_iterator vit (m_pGurobiVC.m_allVarsMap.find( pPT->varsBegin()->second->getName()));
                    if (vit == m_pGurobiVC.m_allVarsMap.end())
                        throw MyException("GRBVar not found");
                    
                    GRBVar var1 = vit->second;
                    
                    if ( pPT->isLinear() )
                        lhs_expr += pPT->getCoeff() * var1;
                    else throw ("unacceptable term type");
                    
                }
            }
            
            
            double rhs = (pClassic->get_rhs()).first;
            
            if (pClassic->isEqConstraint())
            {
                Model.addConstr(lhs_expr, GRB_EQUAL, rhs).set(GRB_IntAttr_Lazy, useNAC);
            }
            else
            {
                Model.addConstr(lhs_expr, GRB_LESS_EQUAL, rhs).set(GRB_IntAttr_Lazy, useNAC);
            }
        }
        else if (pClassic->isQuadratic())
        {
            GRBQuadExpr lhs_expr=0.;
            
            for (ClassicConstraintIF::const_iterator tit = pClassic->begin(); tit != pClassic->end(); tit++)
            {
                if (!(*tit)->isProductTerm() )
                    throw MyException("cplexmisocp cannot have non-prod terms");
                
                boost::shared_ptr<ProductTerm> pPT = boost::static_pointer_cast<ProductTerm>(*tit);
                if (pPT->getNumUncertainties() != 0)
                    throw MyException("cplexmisocp should not involve uncertainties");
                
                
                if (pPT->getNumVars()==0)
                    lhs_expr.addConstant( pPT->getCoeff() );
                else
                {
                    GRBVar var1;
                    GRBVar var2;
                    
                    map<string,GRBVar>::const_iterator vit (m_pGurobiVC.m_allVarsMap.find( pPT->varsBegin()->second->getName()));
                    if (vit == m_pGurobiVC.m_allVarsMap.end())
                        throw MyException("GRBVar not found");
                    
                    var1 = vit->second;
                    
                    if ( pPT->isLinear() )
                        lhs_expr.addTerm(pPT->getCoeff(), var1);
                    else if ( pPT->isQuadratic() )
                    {
                        vit = ( m_pGurobiVC.m_allVarsMap.find( (++pPT->varsBegin())->second->getName()));
                        if (vit == m_pGurobiVC.m_allVarsMap.end())
                            throw MyException("GRBVar not found");
                        var2 =vit->second;
                        lhs_expr.addTerm(pPT->getCoeff(), var1, var2);
                    }
                    else throw ("unacceptable term type");
                    
                }
            }
            
            
            double rhs = (pClassic->get_rhs()).first;
            
            if (pClassic->isEqConstraint())
            {
                throw MyException("cannot have quadratic equality");
            }
            else
            {
                Model.addQConstr(lhs_expr, GRB_LESS_EQUAL, rhs );
            }
        }
        else throw ("unknown constraint type");
        
        
        
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
    
}

void GurobiModeller::addObjective(GRBEnv &env, GRBModel& Model, boost::shared_ptr<ObjectiveFunctionIF> pObjIn) const
{
    
    if (pObjIn->getObjType()!=simpleObj)
        throw MyException("Gurobi Modeller requires a linear objective function");

    
    boost::shared_ptr<LHSExpression> pObj( pObjIn->getObj(1) );
    
    GRBLinExpr obj = 0.;
    
    for(LHSExpression::const_iterator tit = pObj->begin(); tit != pObj->end(); tit++)
    {
        if (!(*tit)->isProductTerm() )
            throw MyException("objective cannot have non-prod terms");
        
        if( (*tit)->hasNonlinearities() )
            throw MyException("Objective should be linear");
        
        boost::shared_ptr<ProductTerm> pPT = boost::static_pointer_cast<ProductTerm>(*tit);
        if (pPT->getNumUncertainties() != 0)
            throw MyException("cplexmisocp should not involve uncertainties");
        
        
        if (pPT->getNumVars()==0)
            obj += pPT->getCoeff() ;
        else
        {
            map<string,GRBVar>::const_iterator vit (m_pGurobiVC.m_allVarsMap.find( pPT->varsBegin()->second->getName()));
            if (vit == m_pGurobiVC.m_allVarsMap.end())
                throw MyException("GRBVar not found");
            
            GRBVar var1 = vit->second;
            
            if ( pPT->isLinear() )
                obj += pPT->getCoeff() * var1;
            else throw ("unacceptable term type");
            
        }
    }
    
    Model.setObjective(obj);
}

void GurobiModeller::addGUROBIwarmstart(const map<string,double>& WSvars) const
{
    map<string,GRBVar>::const_iterator boolVar = m_pGurobiVC.m_boolVarMap.begin();
    vector<GRBVar> wsVars;
    vector<double> wsVals;
    
    for(; boolVar != m_pGurobiVC.m_boolVarMap.end(); boolVar++)
    {
        wsVars.push_back(boolVar->second);
        
        map<string,double>::const_iterator val(WSvars.find(boolVar->first));
        
        if ( val != WSvars.end() ) {
            wsVals.push_back(val->second);
        }
        else{
            throw MyException("Not all bool variables are found");
        }
    }
    
    for (uint i=0; i<wsVars.size(); i++)
    {
        wsVars[i].set(GRB_DoubleAttr_Start, wsVals[i]);
    }
}
