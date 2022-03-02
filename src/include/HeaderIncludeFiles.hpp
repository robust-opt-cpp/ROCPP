/*
 * ROCPP/GurobiModeller.hpp
 *
 * This software is Copyright © 2020 The University of Southern California. All Rights Reserved.
 * Authors: Phebe Vayanos, Qing Jin, George Elissaios
 *
 * This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Paper: "ROC++: Robust Optimization in C++"
 * Homepage: https://sites.google.com/usc.edu/robust-opt-cpp/home
 */

#ifndef HeaderIncludeFiles_hpp
#define HeaderIncludeFiles_hpp


// ONLY INCLUDE IN THIS FILE HEADER FILES THAT ARE USED IN ALMOST ALL ***HEADER FILES***


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%% STD LIBRARY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include <iostream>
#include <map>
#include <memory>
#include <cmath>
#include <string>

using namespace std;

#ifndef uint
#define uint unsigned int
#endif

#ifndef u_long
#define u_long unsigned long
#endif

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% DECISION VARIABLE TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class DecisionVariableIF;
typedef DecisionVariableIF ROCPPVarIF;
typedef shared_ptr<ROCPPVarIF> ROCPPVarIF_Ptr;

class VariableIF;
typedef VariableIF ROCPPStaticVarIF;
typedef shared_ptr<ROCPPStaticVarIF> ROCPPStaticVarIF_Ptr;

class VariableBool;
typedef VariableBool ROCPPStaticVarBool;
typedef shared_ptr<ROCPPStaticVarBool> ROCPPStaticVarBool_Ptr;

class VariableDouble;
typedef VariableDouble ROCPPStaticVarDouble;
typedef shared_ptr<ROCPPStaticVarDouble> ROCPPStaticVarDouble_Ptr;

class VariableInt;
typedef VariableInt ROCPPStaticVarInt;
typedef shared_ptr<ROCPPStaticVarInt> ROCPPStaticVarInt_Ptr;

class AdaptiveVariableIF;
typedef AdaptiveVariableIF ROCPPAdaptVarIF;
typedef shared_ptr<ROCPPAdaptVarIF> ROCPPAdaptVarIF_Ptr;

class AdaptVarBool;
typedef AdaptVarBool ROCPPAdaptVarBool;
typedef shared_ptr<ROCPPAdaptVarBool> ROCPPAdaptVarBool_Ptr;

class AdaptVarDouble;
typedef AdaptVarDouble ROCPPAdaptVarDouble;
typedef shared_ptr<ROCPPAdaptVarDouble> ROCPPAdaptVarDouble_Ptr;

class AdaptVarInt;
typedef AdaptVarInt ROCPPAdaptVarInt;
typedef shared_ptr<ROCPPAdaptVarInt> ROCPPAdaptVarInt_Ptr;

class dvContainer;
typedef shared_ptr<dvContainer> ROCPPdvContainer_Ptr;
typedef shared_ptr<const dvContainer> ROCPPconstdvContainer_Ptr;

typedef map<string, ROCPPVarIF_Ptr> dvMapType;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%% UNCERTAIN PARAMETER TYPEDEF %%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class UncertaintyIF;
typedef UncertaintyIF ROCPPUnc;
typedef shared_ptr<ROCPPUnc> ROCPPUnc_Ptr;

class uncContainer;
typedef shared_ptr<uncContainer> ROCPPuncContainer_Ptr;
typedef shared_ptr<const uncContainer> ROCPPconstuncContainer_Ptr;

typedef map<string, ROCPPUnc_Ptr> uncMapType;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% CONSTRAINT TERM TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class ConstraintTermIF;
typedef ConstraintTermIF ROCPPCstrTermIF;
typedef shared_ptr<ROCPPCstrTermIF> ROCPPCstrTermIF_Ptr;
typedef shared_ptr<const ROCPPCstrTermIF> ROCPPconstCstrTermIF_Ptr;

class ProductTerm;
typedef ProductTerm ROCPPProdTerm;
typedef shared_ptr<ROCPPProdTerm> ROCPPProdTerm_Ptr;
typedef shared_ptr<const ROCPPProdTerm> ROCPPconstProdTerm_Ptr;

class NormTerm;
typedef NormTerm ROCPPNorm;
typedef shared_ptr<NormTerm> ROCPPNormTerm_Ptr;

class LHSExpression;
typedef LHSExpression ROCPPExpr;
typedef shared_ptr<ROCPPExpr> ROCPPExpr_Ptr;
typedef shared_ptr<const ROCPPExpr> ROCPPconstExpr_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% CONSTRAINT TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class ConstraintIF;
typedef ConstraintIF ROCPPConstraintIF;
typedef shared_ptr<ROCPPConstraintIF> ROCPPConstraintIF_Ptr;

class IneqConstraint;
typedef IneqConstraint ROCPPIneqConstraint;
typedef shared_ptr<ROCPPIneqConstraint> ROCPPIneqConstraint_Ptr;

class EqConstraint;
typedef IneqConstraint ROCPPEqConstraint;
typedef shared_ptr<ROCPPEqConstraint> ROCPPEqConstraint_Ptr;

class ClassicConstraintIF;
typedef ClassicConstraintIF ROCPPClassicConstraintIF;
typedef shared_ptr<ClassicConstraintIF> ROCPPClassicConstraint_Ptr;

class SOSConstraint;
typedef SOSConstraint ROCPPSOSConstraint;
typedef shared_ptr<ROCPPSOSConstraint> ROCPPSOSConstraint_Ptr;

class IfThenConstraint;
typedef IfThenConstraint ROCPPIfThenConstraint;
typedef shared_ptr<ROCPPIfThenConstraint> ROCPPIfThenConstraint_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% OBJECTIVE TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class ObjectiveFunctionIF;
typedef ObjectiveFunctionIF ROCPPObjectiveIF;
typedef shared_ptr<ROCPPObjectiveIF> ROCPPObjectiveIF_Ptr;

class SimpleObjective;
typedef SimpleObjective ROCPPSimpleObjective;
typedef shared_ptr<ROCPPSimpleObjective> ROCPPSimpleObjective_Ptr;

class MaxObjective;
typedef MaxObjective ROCPPMaxObjective;
typedef shared_ptr<ROCPPMaxObjective> ROCPPMaxObjective_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% OPTIMIZATION MODEL TYPE DEFS %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class OptimizationModelIF;
typedef OptimizationModelIF ROCPPOptModelIF;
typedef shared_ptr<ROCPPOptModelIF> ROCPPOptModelIF_Ptr;

class MultiStageOptModelDDID;
typedef MultiStageOptModelDDID ROCPPOptModelDDID;
typedef shared_ptr<ROCPPOptModelDDID> ROCPPOptModelDDID_Ptr;

class DeterministicOptimizationModel;
typedef DeterministicOptimizationModel ROCPPDetOptModel;
typedef shared_ptr<ROCPPDetOptModel> ROCPPDetOptModel_Ptr;

class UncertainOptimizationModel;
typedef UncertainOptimizationModel ROCPPUncOptModel;
typedef shared_ptr<ROCPPUncOptModel> ROCPPUncOptModel_Ptr;
typedef shared_ptr<const ROCPPUncOptModel> ROCPPconstUncOptModel_Ptr;

class MultiStageOptModelExoID;
typedef MultiStageOptModelExoID ROCPPOptModelExoID;
typedef shared_ptr<ROCPPOptModelExoID> ROCPPOptModelExoID_Ptr;

class UncertainSingleStageOptimizationModel;
typedef UncertainSingleStageOptimizationModel ROCPPUncSSOptModel;
typedef shared_ptr<ROCPPUncSSOptModel> ROCPPUncSSOptModel_Ptr;

class UncertainMultiStageOptimizationModel;
typedef UncertainMultiStageOptimizationModel ROCPPUncMSOptModel;
typedef shared_ptr<ROCPPUncMSOptModel> ROCPPUncMSOptModel_Ptr;

class Bilinear_MISOCP;
typedef Bilinear_MISOCP ROCPPBilinMISOCP;
typedef shared_ptr<ROCPPBilinMISOCP> ROCPPBilinMISOCP_Ptr;

class MISOCP;
typedef MISOCP ROCPPMISOCP;
typedef shared_ptr<ROCPPMISOCP> ROCPPMISOCP_Ptr;

class CPLEXMISOCP;
typedef CPLEXMISOCP ROCPPCPLEXMISOCP;
typedef shared_ptr<ROCPPCPLEXMISOCP> ROCPPCPLEXMISOCP_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%% VARIABLE CONVERTER TYPE DEFS %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class VariableConverterIF;
typedef VariableConverterIF ROCPPVariableConverterIF;
typedef shared_ptr<ROCPPVariableConverterIF> ROCPPVariableConverterIF_Ptr;

class OneToOneVariableConverterIF;
typedef OneToOneVariableConverterIF ROCPPOneToOneVarConverterIF;
typedef shared_ptr<OneToOneVariableConverterIF> ROCPPOneToOneVarConverterIF_Ptr;

class OneToExprVariableConverterIF;
typedef OneToExprVariableConverterIF ROCPPOneToExprVarConverterIF;
typedef shared_ptr<ROCPPOneToExprVarConverterIF> ROCPPOneToExprVarConverterIF_Ptr;


class BilinearTermReformulatorIF;
typedef shared_ptr<BilinearTermReformulatorIF> ROCPPBilinearReform_Ptr;

class Bilinear_MItoMB_Converter;
typedef Bilinear_MItoMB_Converter ROCPPMItoMB;
typedef shared_ptr<Bilinear_MItoMB_Converter> ROCPPMItoMB_Ptr;

class UnaryConverter;
typedef shared_ptr<UnaryConverter> ROCPPUnaryMItoMB_Ptr;

class BinaryConverter;
typedef shared_ptr<BinaryConverter> ROCPPBinaryMItoMB_Ptr;

class RealVarDiscretizer;
typedef RealVarDiscretizer ROCPPRealVarDiscretizer;
typedef shared_ptr<RealVarDiscretizer> ROCPPRealVarDiscretizer_Ptr;

class UncertaintySetRealVarApproximator;
typedef UncertaintySetRealVarApproximator ROCPPUncSetRealVarApprox;
typedef shared_ptr<UncertaintySetRealVarApproximator> ROCPPUncSetRealVarApprox_Ptr;

class PredefO2OVariableConverter;
typedef PredefO2OVariableConverter ROCPPPredefO2OVarConverter;
typedef shared_ptr<PredefO2OVariableConverter> ROCPPPredefO2OVarConverter_Ptr;

class PredefO2EVariableConverter;
typedef PredefO2EVariableConverter ROCPPPredefO2EVarConverter;
typedef shared_ptr<PredefO2EVariableConverter> ROCPPPredefO2EVarConverter_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%% UNCERTAINTY CONVERTER TYPE DEFS %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class UncertaintyConverterIF;
typedef UncertaintyConverterIF ROCPPUncertaintyConverterIF;
typedef shared_ptr<ROCPPUncertaintyConverterIF> ROCPPUncertaintyConverterIF_Ptr;

class UncertaintyToVariableConverter;
typedef UncertaintyToVariableConverter ROCPPUncToVarConverter;
typedef shared_ptr<ROCPPUncToVarConverter> ROCPPUncToVarConverter_Ptr;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%% DECISION RULE TYPE DEFS %%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
class DecisionRuleIF;
typedef DecisionRuleIF ROCPPDRIF;
typedef shared_ptr<ROCPPDRIF> ROCPPDRIF_Ptr;

class ContinuousVarsDRIF;
typedef shared_ptr<ContinuousVarsDRIF> ROCPPContinuousVarsDR_Ptr;

class LinearDecisionRule;
typedef LinearDecisionRule ROCPPLinearDR;
typedef shared_ptr<ROCPPLinearDR> ROCPPLinearDR_Ptr;

class DiscreteVarsDRIF;
typedef shared_ptr<DiscreteVarsDRIF> ROCPPDiscreteVarsDR_Ptr;

class ConstantDecisionRule;
typedef ConstantDecisionRule ROCPPConstantDR;
typedef shared_ptr<ROCPPConstantDR> ROCPPConstantDR_Ptr;

class PiecewiseDecisionRule;
typedef PiecewiseDecisionRule ROCPPPWDR;
typedef shared_ptr<ROCPPPWDR> ROCPPPWDR_Ptr;

class Kadaptability;
typedef Kadaptability ROCPPKAdapt;
typedef shared_ptr<ROCPPKAdapt> ROCPPKAdapt_Ptr;

class KadaptabilityPartitionEncoderMS;
typedef shared_ptr<KadaptabilityPartitionEncoderMS> ROCPPKadaptEncoder_Ptr;

class PartitionConstructorIF;
typedef shared_ptr<PartitionConstructorIF> ROCPPParConstructor_Ptr;

class PartitionConverter;
typedef shared_ptr<PartitionConverter> ROCPPParConverter_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% REFORMULATION STRATGEY TYPE DEFS %%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class ReformulationStrategyIF;
typedef ReformulationStrategyIF ROCPPStrategy;
typedef shared_ptr<ROCPPStrategy> ROCPPStrategy_Ptr;

class ReformulationOrchestrator;
typedef ReformulationOrchestrator ROCPPOrchestrator;
typedef shared_ptr<ROCPPOrchestrator> ROCPPOrchestrator_Ptr;

class RobustifyEngine;
typedef RobustifyEngine ROCPPRobustifyEngine;
typedef shared_ptr<RobustifyEngine> ROCPPRobustifyEngine_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% SOLVER MODELLER TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class SolverModellerIF;
typedef SolverModellerIF ROCPPSolverInterface;
typedef shared_ptr<SolverModellerIF> ROCPPSolverInterface_Ptr;

class SolverParams;
typedef SolverParams ROCPPSolverParams;
typedef shared_ptr<ROCPPSolverParams> ROCPPSolverParams_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%% SCIP SOLVER TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class SCIPModeller;
typedef SCIPModeller ROCPPSCIP;
typedef shared_ptr<SCIPModeller> ROCPPSCIP_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%% GUROBI SOLVER TYPE DEFS %%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class GurobiModeller;
typedef GurobiModeller ROCPPGurobi;
typedef shared_ptr<GurobiModeller> ROCPPGurobi_Ptr;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%% OPT MODEL CONVERTER TYPE DEFS %%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class BTR_bigM;
typedef BTR_bigM ROCPPBTR_bigM;
typedef shared_ptr<BTR_bigM> ROCPPBTR_bigM_Ptr;

#endif /* HeaderIncludeFiles_hpp */
