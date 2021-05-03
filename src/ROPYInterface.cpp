//
//  RoPyInterface.cpp
//  RobustOptimizationPlatform
//
// This software is Copyright © 2020 Phebe Vayanos. All Rights Reserved.
// Software created by Phebe Vayanos, Qing Jin, and George Elissaios
//

#ifdef PYTHON_PACKAGE

#include <vector>
#include <map>
#include <iostream>
#include "ROCPP.h"

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

namespace py=pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, shared_ptr<T>);
PYBIND11_MODULE(ROPY, m) {
    m.doc() = "Robust Optimization in Python";

    //python for decision variable
    py::enum_<decVariableType>(m, "decVariableType").value("intDV", decVariableType::intDV).value("boolDV", decVariableType::boolDV).value("contDV", decVariableType::contDV);
    py::enum_<constraintTermType >(m, "constraintTermType").value("prodTerm", constraintTermType::prodTerm).value("normTerm", constraintTermType::normTerm);
    py::enum_<objType>(m, "objType").value("simpleObj", objType::simpleObj).value("maxObj", objType::maxObj);
    py::enum_<problemType>(m, "problemType").value("uncertainType", problemType::uncertainType).value("dduType", problemType::dduType).value("deterministicType", problemType::deterministicType);
    py::enum_<uncOptModelObjType>(m, "uncOptModelObjType").value("robust", uncOptModelObjType::robust).value("stochastic", uncOptModelObjType::stochastic);
    
    py::class_<ROCPPVarIF, ROCPPVarIF_Ptr>(m, "RoPyVarIF").def("getLB", &ROCPPVarIF::getLB).def("getUB", &ROCPPVarIF::getUB).def("setLB", &ROCPPVarIF::setLB).def("setUB", &ROCPPVarIF::setUB)
    .def("isBooleanVar", &ROCPPVarIF::isBooleanVar).def("isIntegerVar", &ROCPPVarIF::isIntegerVar).def("isRealVar", &ROCPPVarIF::isRealVar)
    .def("getType", &ROCPPVarIF::getType).def("getName", &ROCPPVarIF::getName).def("getTimeStage", &ROCPPVarIF::getTimeStage)
    .def("__add__", [](ROCPPVarIF_Ptr x, double y){return x + y;})
    .def("__sub__", [](ROCPPVarIF_Ptr x, double y){return x - y;})
    .def("__radd__", [](ROCPPVarIF_Ptr x, double y){return x + y;})
    .def("__rsub__", [](ROCPPVarIF_Ptr x, double y){return (-1.0)*x + y;})
    .def("__add__", [](ROCPPVarIF_Ptr x, ROCPPVarIF_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPVarIF_Ptr x, ROCPPVarIF_Ptr y){return x - y;})
    .def("__add__", [](ROCPPVarIF_Ptr x, ROCPPUnc_Ptr y){return x + y;})
    .def("__add__", [](ROCPPVarIF_Ptr x, ROCPPCstrTermIF_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPVarIF_Ptr x, ROCPPCstrTermIF_Ptr y){return x - y;})
    .def("__add__", [](ROCPPVarIF_Ptr x, ROCPPExpr_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPVarIF_Ptr x, ROCPPExpr_Ptr y){return x - y;})
    .def("__mul__", [](ROCPPVarIF_Ptr x, ROCPPVarIF_Ptr y){return x * y;})
    .def("__mul__", [](ROCPPVarIF_Ptr x, ROCPPUnc_Ptr y){return x * y;})
    .def("__mul__", [](ROCPPVarIF_Ptr x, ROCPPExpr_Ptr y){return x * y;})
    .def("__rmul__", [](ROCPPVarIF_Ptr x, float y){return y * x;})
    .def("__le__", [](ROCPPVarIF_Ptr x, double y){return x <= y;})
    .def("__ge__", [](ROCPPVarIF_Ptr x, double y){return x >= y;})
    .def("__eq__", [](ROCPPVarIF_Ptr x, double y){return x == y;})
    .def("__le__", [](ROCPPVarIF_Ptr x, ROCPPVarIF_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPVarIF_Ptr x, ROCPPVarIF_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPVarIF_Ptr x, ROCPPVarIF_Ptr y){return x == y;})
    .def("__le__", [](ROCPPVarIF_Ptr x, ROCPPUnc_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPVarIF_Ptr x, ROCPPUnc_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPVarIF_Ptr x, ROCPPUnc_Ptr y){return x == y;})
    .def("__le__", [](ROCPPVarIF_Ptr x, ROCPPCstrTermIF_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPVarIF_Ptr x, ROCPPCstrTermIF_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPVarIF_Ptr x, ROCPPCstrTermIF_Ptr y){return x == y;})
    .def("__le__", [](ROCPPVarIF_Ptr x, ROCPPExpr_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPVarIF_Ptr x, ROCPPExpr_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPVarIF_Ptr x, ROCPPExpr_Ptr y){return x == y;});

    py::class_<ROCPPStaticVarInt, ROCPPStaticVarInt_Ptr, ROCPPVarIF>(m, "RoPyStaticVarInt").def(py::init<string, double, double>(),"name"_a, "lb"_a = -INFINITY, "ub"_a = INFINITY);
    py::class_<ROCPPStaticVarReal, ROCPPStaticVarReal_Ptr, ROCPPVarIF>(m, "RoPyStaticVarDouble").def(py::init<string, double, double>(),"name"_a, "lb"_a = -INFINITY, "ub"_a = INFINITY);
    py::class_<ROCPPStaticVarBool, ROCPPStaticVarBool_Ptr, ROCPPVarIF>(m, "RoPyStaticVarBool").def(py::init<string, double, double>(),"name"_a, "lb"_a = 0.0, "ub"_a = 1.0);
    py::class_<ROCPPAdaptVarBool, ROCPPAdaptVarBool_Ptr, ROCPPVarIF>(m, "RoPyAdaptVarBool").def(py::init<string, uint, double, double>(),"name"_a, "timsStage"_a, "lb"_a = 0.0, "ub"_a = 1.0);
    py::class_<ROCPPAdaptVarInt, ROCPPAdaptVarInt_Ptr, ROCPPVarIF>(m, "RoPyAdaptVarInt").def(py::init<string, uint, double, double>(),"name"_a, "timsStage"_a, "lb"_a = -INFINITY, "ub"_a = INFINITY);
    py::class_<ROCPPAdaptVarReal, ROCPPAdaptVarReal_Ptr, ROCPPVarIF>(m, "RoPyAdaptVarDouble").def(py::init<string, uint, double, double>(),"name"_a, "timsStage"_a, "lb"_a = -INFINITY, "ub"_a = INFINITY);

    //python for uncertain parameter
    py::class_<ROCPPUnc, ROCPPUnc_Ptr>(m, "RoPyUnc").def(py::init<string, uint, bool>(), "name"_a, "timeStage"_a = 1, "isObservable"_a = true)
    .def("getName", &ROCPPUnc::getName).def("getTimeStage", &ROCPPUnc::getTimeStage)
    .def("isObservable", &ROCPPUnc::isObservable)
    .def("__add__", [](ROCPPUnc_Ptr x, double y){return x + y;})
    .def("__sub__", [](ROCPPUnc_Ptr x, double y){return x - y;})
    .def("__radd__", [](ROCPPUnc_Ptr x, double y){return x + y;})
    .def("__rsub__", [](ROCPPUnc_Ptr x, double y){return -1.0*x + y;})
    .def("__add__", [](ROCPPUnc_Ptr x, ROCPPVarIF_Ptr y){return y + x;})
    .def("__sub__", [](ROCPPUnc_Ptr x, ROCPPVarIF_Ptr y){return x - y;})
    .def("__add__", [](ROCPPUnc_Ptr x, ROCPPUnc_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPUnc_Ptr x, ROCPPUnc_Ptr y){return x - y;})
    .def("__add__", [](ROCPPUnc_Ptr x, ROCPPCstrTermIF_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPUnc_Ptr x, ROCPPCstrTermIF_Ptr y){return x - y;})
    .def("__add__", [](ROCPPUnc_Ptr x, ROCPPExpr_Ptr y){return y + x;})
    .def("__sub__", [](ROCPPUnc_Ptr x, ROCPPExpr_Ptr y){return x - y;})
    .def("__rmul__", [](ROCPPUnc_Ptr x, float y){return y * x;})
    .def("__mul__", [](ROCPPUnc_Ptr x, ROCPPVarIF_Ptr y){return x * y;})
    .def("__mul__", [](ROCPPUnc_Ptr x, ROCPPExpr_Ptr y){return x * y;})
    .def("__le__", [](ROCPPUnc_Ptr x, double y){return x <= y;})
    .def("__ge__", [](ROCPPUnc_Ptr x, double y){return x >= y;})
    .def("__eq__", [](ROCPPUnc_Ptr x, double y){return x == y;})
    .def("__le__", [](ROCPPUnc_Ptr x, ROCPPVarIF_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPUnc_Ptr x, ROCPPVarIF_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPUnc_Ptr x, ROCPPVarIF_Ptr y){return x == y;})
    .def("__le__", [](ROCPPUnc_Ptr x, ROCPPUnc_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPUnc_Ptr x, ROCPPUnc_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPUnc_Ptr x, ROCPPUnc_Ptr y){return x == y;})
    .def("__le__", [](ROCPPUnc_Ptr x, ROCPPCstrTermIF_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPUnc_Ptr x, ROCPPCstrTermIF_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPUnc_Ptr x, ROCPPCstrTermIF_Ptr y){return x == y;})
    .def("__le__", [](ROCPPUnc_Ptr x, ROCPPExpr_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPUnc_Ptr x, ROCPPExpr_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPUnc_Ptr x, ROCPPExpr_Ptr y){return x == y;});
    
    //python for constraint terms
    py::class_<ROCPPCstrTermIF, ROCPPCstrTermIF_Ptr>(m, "RoPyCstrTermIF").def("isProductTerm", &ROCPPCstrTermIF::isProductTerm).def("isNonlinearProdTerm", &ROCPPCstrTermIF::isNonlinearProdTerm)
    .def("isNormTerm", &ROCPPCstrTermIF::isNormTerm).def("isConstant", &ROCPPCstrTermIF::isConstant).def("isDeterministic", &ROCPPCstrTermIF::isDeterministic).def("isLinear", &ROCPPCstrTermIF::isDeterministic)
    .def("isLinear", &ROCPPCstrTermIF::isLinear).def("isQuadratic", &ROCPPCstrTermIF::isQuadratic).def("getNumContVars", &ROCPPCstrTermIF::getNumContVars).def("getNumIntVars", &ROCPPCstrTermIF::getNumIntVars)
    .def("getNumBoolVars", &ROCPPCstrTermIF::getNumBoolVars).def("getNumAdaptiveContVar", &ROCPPCstrTermIF::getNumAdaptiveContVars).def("getAdaptiveVars", &ROCPPCstrTermIF::getNumAdaptiveVars)
    .def("getNumUncertainties", &ROCPPCstrTermIF::getNumUncertainties)
    .def("__add__", [](ROCPPCstrTermIF_Ptr x, double y){return x + y;})
    .def("__sub__", [](ROCPPCstrTermIF_Ptr x, double y){return x - y;})
    .def("__radd__", [](ROCPPCstrTermIF_Ptr x, double y){return x + y;})
    //.def("__rsub__", [](ROCPPCstrTermIF_Ptr x, double y){return -1.0*x + y;})
    .def("__add__", [](ROCPPCstrTermIF_Ptr x, ROCPPUnc_Ptr y){return y + x;})
    .def("__sub__", [](ROCPPCstrTermIF_Ptr x, ROCPPUnc_Ptr y){return x - y;})
    .def("__add__", [](ROCPPCstrTermIF_Ptr x, ROCPPVarIF_Ptr y){return y + x;})
    .def("__sub__", [](ROCPPCstrTermIF_Ptr x, ROCPPVarIF_Ptr y){return x - y;})
    .def("__add__", [](ROCPPCstrTermIF_Ptr x, ROCPPCstrTermIF_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPCstrTermIF_Ptr x, ROCPPCstrTermIF_Ptr y){return x - y;})
    .def("__add__", [](ROCPPCstrTermIF_Ptr x, ROCPPExpr_Ptr y){return y + x;})
    .def("__sub__", [](ROCPPCstrTermIF_Ptr x, ROCPPExpr_Ptr y){return x - y;})
    //.def("__rmul__", [](ROCPPCstrTermIF_Ptr x, float y){return y * x;})
    .def("__le__", [](ROCPPCstrTermIF_Ptr x, double y){return x <= y;})
    .def("__ge__", [](ROCPPCstrTermIF_Ptr x, double y){return x >= y;})
    .def("__eq__", [](ROCPPCstrTermIF_Ptr x, double y){return x == y;})
    .def("__le__", [](ROCPPCstrTermIF_Ptr x, ROCPPVarIF_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPCstrTermIF_Ptr x, ROCPPVarIF_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPCstrTermIF_Ptr x, ROCPPVarIF_Ptr y){return x == y;})
    .def("__le__", [](ROCPPCstrTermIF_Ptr x, ROCPPUnc_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPCstrTermIF_Ptr x, ROCPPUnc_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPCstrTermIF_Ptr x, ROCPPUnc_Ptr y){return x == y;})
    .def("__le__", [](ROCPPCstrTermIF_Ptr x, ROCPPExpr_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPCstrTermIF_Ptr x, ROCPPExpr_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPCstrTermIF_Ptr x, ROCPPExpr_Ptr y){return x == y;});

    py::class_<ROCPPNorm, ROCPPNormTerm_Ptr, ROCPPCstrTermIF>(m, "RoPyNorm").def(py::init<vector<ROCPPExpr_Ptr>>());
    
    //python for left hand side expression
    py::class_<ROCPPExpr, ROCPPExpr_Ptr>(m, "RoPyExpr").def(py::init<>())
    .def("add", (void (ROCPPExpr::*)(double)) &ROCPPExpr::add, "add double to the expression")
    .def("add", (void (ROCPPExpr::*)(double, ROCPPVarIF_Ptr)) &ROCPPExpr::add, "add decision variable to the expression")
    .def("add", (void (ROCPPExpr::*)(double, ROCPPVarIF_Ptr, ROCPPVarIF_Ptr)) &ROCPPExpr::add, "add product of decision variables to the expression")
    .def("add", (void (ROCPPExpr::*)(ROCPPconstCstrTermIF_Ptr)) &ROCPPExpr::add, "add constraint term to the expression")
    .def("add", (void (ROCPPExpr::*)(ROCPPconstExpr_Ptr)) &ROCPPExpr::add, "add expression to the expression")
    .def("add", (void (ROCPPExpr::*)(double, ROCPPconstExpr_Ptr)) &ROCPPExpr::add, "add expression term to the expression")
    .def("add", (void (ROCPPExpr::*)(double, ROCPPconstExpr_Ptr, ROCPPVarIF_Ptr)) &ROCPPExpr::add, "add product of expression and variable to the expression")
    .def("isDeterministic", &ROCPPExpr::isDeterministic).def("hasNonlinearities", &ROCPPExpr::hasNonlinearities).def("getNumVars", &ROCPPExpr::getNumVars)
    .def("__add__", [](ROCPPExpr_Ptr x, double y){return x + y;})
    .def("__sub__", [](ROCPPExpr_Ptr x, double y){return x - y;})
    .def("__radd__", [](ROCPPExpr_Ptr x, double y){return x + y;})
    .def("__rsub__", [](ROCPPExpr_Ptr x, double y){return (-1.0)*x + y;})
    .def("__add__", [](ROCPPExpr_Ptr x, ROCPPUnc_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPExpr_Ptr x, ROCPPUnc_Ptr y){return x - y;})
    .def("__add__", [](ROCPPExpr_Ptr x, ROCPPVarIF_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPExpr_Ptr x, ROCPPVarIF_Ptr y){return x - y;})
    .def("__add__", [](ROCPPExpr_Ptr x, ROCPPCstrTermIF_Ptr y){return x + y;})
    //.def("__sub__", [](ROCPPExpr_Ptr x, ROCPPCstrTermIF_Ptr y){return x - y;})
    .def("__add__", [](ROCPPExpr_Ptr x, ROCPPExpr_Ptr y){return x + y;})
    .def("__sub__", [](ROCPPExpr_Ptr x, ROCPPExpr_Ptr y){return x - y;})
    .def("__mul__", [](ROCPPExpr_Ptr x, double y){return y * x;})
    .def("__rmul__", [](ROCPPExpr_Ptr x, double y){return y * x;})
    .def("__mul__", [](ROCPPExpr_Ptr x, ROCPPVarIF_Ptr y){return y * x;})
    .def("__mul__", [](ROCPPExpr_Ptr x, ROCPPUnc_Ptr y){return x * y;})
    //.def("__mul__", [](ROCPPExpr_Ptr x, ROCPPExpr_Ptr y){return x * y;})
    .def("__le__", [](ROCPPExpr_Ptr x, double y){return x <= y;})
    .def("__ge__", [](ROCPPExpr_Ptr x, double y){return x >= y;})
    .def("__eq__", [](ROCPPExpr_Ptr x, double y){return x == y;})
    .def("__le__", [](ROCPPExpr_Ptr x, ROCPPVarIF_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPExpr_Ptr x, ROCPPVarIF_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPExpr_Ptr x, ROCPPVarIF_Ptr y){return x == y;})
    .def("__le__", [](ROCPPExpr_Ptr x, ROCPPUnc_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPExpr_Ptr x, ROCPPUnc_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPExpr_Ptr x, ROCPPUnc_Ptr y){return x == y;})
    .def("__le__", [](ROCPPExpr_Ptr x, ROCPPCstrTermIF_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPExpr_Ptr x, ROCPPCstrTermIF_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPExpr_Ptr x, ROCPPCstrTermIF_Ptr y){return x == y;})
    .def("__le__", [](ROCPPExpr_Ptr x, ROCPPExpr_Ptr y){return x <= y;})
    .def("__ge__", [](ROCPPExpr_Ptr x, ROCPPExpr_Ptr y){return x >= y;})
    .def("__eq__", [](ROCPPExpr_Ptr x, ROCPPExpr_Ptr y){return x == y;})
    .def("getConst", &ROCPPExpr::getSumConstantTerms).def("getNumTerms", &ROCPPExpr::getNumTerms);
    
    //python for constraint
    py::class_<ROCPPConstraintIF, ROCPPConstraintIF_Ptr>(m, "RoPyConstraintIF").def("isClassicConstraint", &ROCPPConstraintIF::isClassicConstraint).def("SetRhs", &ROCPPConstraintIF::set_rhs);

    py::class_<ROCPPObjectiveIF, ROCPPObjectiveIF_Ptr>(m, "RoPyObjectiveIF").def("getObjType", &ROCPPObjectiveIF::getObjType).def("isDeterministic", &ROCPPObjectiveIF::isDeterministic);
    py::class_<ROCPPSimpleObjective, ROCPPSimpleObjective_Ptr, ROCPPObjectiveIF>(m, "RoPySimpleObjective");
    py::class_<ROCPPMaxObjective, ROCPPMaxObjective_Ptr, ROCPPObjectiveIF>(m, "RoPyMaxObjective");
    
    //python for optimization model
    py::class_<ROCPPOptModelIF, ROCPPOptModelIF_Ptr>(m, "RoPyOptModelIF")
    .def("writeToFile", &ROCPPOptModelIF::WriteToFile)
    .def("addConstraint", &ROCPPOptModelIF::add_constraint, "pConstraint"_a, "blockNme"_a = "main")
    .def("addConstraints", &ROCPPOptModelIF::add_constraints, "first"_a, "last"_a, "blockNme"_a = "main")
    .def("addSOCConstraint", &ROCPPOptModelIF::add_soc_constraint, "coneHead"_a, "otherVar"_a, "blockNme"_a = "main")
    .def("addConstraintUncSet", &ROCPPOptModelIF::add_constraint_uncset, "pUncCstr"_a, "blockNme"_a = "main")
    .def("setObjective", (void (ROCPPOptModelIF::*)(ROCPPExpr_Ptr)) &ROCPPOptModelIF::set_objective)
    .def("setObjective", (void (ROCPPOptModelIF::*)(vector<ROCPPExpr_Ptr>)) &ROCPPOptModelIF::set_objective)
    .def("getMeasVar", &ROCPPOptModelIF::getMeasVar)
    .def("getVar", &ROCPPOptModelIF::getVar)
    .def("getUnc", &ROCPPOptModelIF::getUnc)
    .def("addDDU", &ROCPPOptModelIF::add_ddu);
    
    py::class_<ROCPPDetOptModel, ROCPPDetOptModel_Ptr, ROCPPOptModelIF>(m, "RoPyDetOptModel").def(py::init<>());
    py::class_<MISOCP, shared_ptr<MISOCP>, ROCPPDetOptModel>(m, "RoPyMISOCP").def(py::init<>());
    py::class_<ROCPPUncOptModel, ROCPPUncOptModel_Ptr, ROCPPOptModelIF>(m, "RoPyUncOptModel")
    .def(py::init<uint, uncOptModelObjType>(), "numTimeStages"_a = 1, "objType"_a = robust);
    py::class_<ROCPPUncSSOptModel, ROCPPUncSSOptModel_Ptr, ROCPPUncOptModel>(m, "RoPyUncSSOptModel")
    .def(py::init<uncOptModelObjType>(), "objType"_a = robust);
    py::class_<ROCPPUncMSOptModel, ROCPPUncMSOptModel_Ptr, ROCPPUncOptModel>(m, "RoPyUncMSOptModel")
    .def(py::init<uint, uncOptModelObjType>(), "numTimeStages"_a = 1, "objType"_a = robust);
    py::class_<ROCPPOptModelExoID, ROCPPOptModelExoID_Ptr, UncertainMultiStageOptimizationModel>(m, "RoPyOptModelExoID")
    .def(py::init<uint, uncOptModelObjType>(), "numTimeStages"_a = 1, "objType"_a = robust);
    py::class_<ROCPPOptModelDDID, ROCPPOptModelDDID_Ptr, UncertainMultiStageOptimizationModel>(m, "RoPyOptModelDDID")
    .def(py::init<uint, uncOptModelObjType>(), "numTimeStages"_a = 1, "objType"_a = robust)
    .def("pairUncertainties", &ROCPPOptModelDDID::pair_uncertainties);
    
    // python for reformulation strategy
    py::class_<ROCPPStrategy, ROCPPStrategy_Ptr>(m, "RoPyStrategy")
    .def("Reformulate", (ROCPPOptModelIF_Ptr (ROCPPStrategy::*)(ROCPPOptModelIF_Ptr) ) &ROCPPStrategy::Reformulate);
    
    py::class_<ROCPPRobustifyEngine, ROCPPRobustifyEngine_Ptr, ROCPPStrategy>(m, "RoPyRobustifyEngine")
    .def(py::init<uint, string, string>(), "dualVarsCounter"_a = 0, "dualNme_suff"_a = "", "dualNme"_a = "dual")
    .def("robustify", &ROCPPRobustifyEngine::robustify);
    
    py::class_<BilinearTermReformulatorIF, ROCPPBilinearReform_Ptr, ROCPPStrategy>(m, "RoPyBilinearReform");
    py::class_<BTR_bigM, ROCPPBTR_bigM_Ptr, BilinearTermReformulatorIF>(m, "RoPyBTR_bigM")
    .def(py::init<string, string, uint, double>(), "aux_var_name"_a = "bl", "aux_var_sffx"_a = "", "auxVarCnt"_a = 0, "M"_a = 100);
    
    py::class_<Bilinear_MItoMB_Converter, ROCPPMItoMB_Ptr, ROCPPStrategy>(m, "RoPyMItoMB");
    py::class_<UnaryConverter, ROCPPUnaryMItoMB_Ptr, Bilinear_MItoMB_Converter>(m, "RoPyUnaryMItoMB")
    .def(py::init<>());
    py::class_<BinaryConverter, ROCPPBinaryMItoMB_Ptr, Bilinear_MItoMB_Converter>(m, "RoPyBinaryMItoMB")
    .def(py::init<>());
    
    py::class_<UncertaintySetRealVarApproximator, ROCPPUncSetRealVarApprox_Ptr, ROCPPStrategy>(m, "RoPyUncSetRealVarApprox")
    .def(py::init<uint>());
        
    py::class_<DecisionRuleIF, ROCPPDRIF_Ptr>(m, "RoPyDecisionRuleIF");
    
    py::class_<LinearDecisionRule, ROCPPLinearDR_Ptr, ROCPPStrategy>(m, "RoPyLinearDR")
    .def(py::init<uint, double>(), "memory"_a = 1000, "bigM"_a = 100.)
    .def("printOut", (void (LinearDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPVarIF_Ptr) ) &LinearDecisionRule::printOut)
    .def("printOut", (void (LinearDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPUnc_Ptr) ) &LinearDecisionRule::printOut);
    
    py::class_<ConstantDecisionRule, ROCPPConstantDR_Ptr, ROCPPStrategy>(m, "RoPyConstantDR")
    .def(py::init<uint>(), "memory"_a = 1000)
    .def("printOut", (void (ConstantDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPVarIF_Ptr) ) &ConstantDecisionRule::printOut)
    .def("printOut", (void (ConstantDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPUnc_Ptr) ) &ConstantDecisionRule::printOut);
    
    py::class_<KadaptabilityDecisionRule, ROCPPKAdapt_Ptr, DecisionRuleIF, ROCPPStrategy>(m, "RoPyKadapt")
    .def(py::init<const map<uint,uint>, double, double, string>(), "numPartitionsMap"_a, "bigM"_a = 100.0, "epsilon"_a = 0.0001, "folder"_a=" ")
    .def("printOut", (void (KadaptabilityDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPVarIF_Ptr, string partition)) &KadaptabilityDecisionRule::printOut)
    .def("printOut", (void (KadaptabilityDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPUnc_Ptr, string partition)) &KadaptabilityDecisionRule::printOut)
    .def("printOut", (void (KadaptabilityDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPVarIF_Ptr) ) &KadaptabilityDecisionRule::printOut)
    .def("printOut", (void (KadaptabilityDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPUnc_Ptr) ) &KadaptabilityDecisionRule::printOut);
    
    py::class_<PiecewiseDecisionRule, ROCPPPWDR_Ptr, DecisionRuleIF, ROCPPStrategy>(m, "RoPyPWDR")
    .def(py::init<const map<string,uint>&, double, bool, string>(), "numPartitionsMap"_a = map<string, uint>(),"bigM"_a = 100.0, "epsilon"_a = 0.0001, "folder"_a=" ")
    .def("printOut", (void (PiecewiseDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPVarIF_Ptr, map<ROCPPUnc_Ptr, uint> partitionIn) ) &PiecewiseDecisionRule::printOut)
    .def("printOut", (void (PiecewiseDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPUnc_Ptr, map<ROCPPUnc_Ptr, uint> partitionIn) ) &PiecewiseDecisionRule::printOut)
    .def("printOut", (void (PiecewiseDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPVarIF_Ptr) ) &PiecewiseDecisionRule::printOut)
    .def("printOut", (void (PiecewiseDecisionRule::*)(const ROCPPOptModelIF_Ptr, const map<string, double> &, ROCPPUnc_Ptr) ) &PiecewiseDecisionRule::printOut);
    
    // python for orchestrator
    py::class_<ReformulationOrchestrator, ROCPPOrchestrator_Ptr>(m, "RoPyOrchestrator")
    .def(py::init<>())
    .def("Reformulate", (ROCPPOptModelIF_Ptr (ReformulationOrchestrator::*)(ROCPPOptModelIF_Ptr, ROCPPStrategy_Ptr) const) &ReformulationOrchestrator::Reformulate)
    .def("Reformulate", (ROCPPOptModelIF_Ptr (ReformulationOrchestrator::*)(ROCPPOptModelIF_Ptr, vector<ROCPPStrategy_Ptr>) const) &ReformulationOrchestrator::Reformulate);
    

    // python for solver
    py::class_<SolverParams, ROCPPSolverParams_Ptr>(m, "RoPySolverParams")
    .def(py::init<bool, bool, pair<bool,double>, pair<bool,double>, pair<bool,double>, pair<bool,double>, pair<bool,double>, pair<bool,double>, double>(), "useLazyNACs"_a=false, "verbose"_a = true, "epIntLimit"_a = make_pair(false,0.), "timeLimit"_a = make_pair(false,0.), "epGapLimit"_a = make_pair(false,1.e-4),  "epAGapLimit"_a = make_pair(false,1.e-10),
                  "epOptLimit"_a = make_pair(false,1.e-6),  "epRHSLimit"_a = make_pair(false,1.e-9),  "SOSeps"_a = 1.e-20);

    py::class_<SolverModellerIF, ROCPPSolverInterface_Ptr>(m, "RoPySolverIF")
    .def("getOptValue", &SolverModellerIF::getOptValue)
    .def("getSolution", (map<string, double> (SolverModellerIF::*)() const) &SolverModellerIF::getSolution)
    .def("getSolution", (double (SolverModellerIF::*)(string) const) &SolverModellerIF::getSolution)
    .def("solve", &SolverModellerIF::solve, "pModelIn"_a, "writeSlnToFile"_a = false, "fileName"_a = "", "writeSlnToConsole"_a = true, "WSVars"_a = (map<string,double>()), "priorities"_a = (map<string,int>()), "deleteModel"_a = false);
    
#ifdef USE_GUROBI
    py::class_<GurobiModeller, ROCPPGurobi_Ptr, SolverModellerIF>(m, "RoPySolver")
    .def(py::init<const SolverParams &, bool>(), "pSParams"_a, "useLazyNACs"_a=false);
#elif defined(USE_SCIP)
    py::class_<SCIPModeller, ROCPPSCIP_Ptr, SolverModellerIF>(m, "RoPySolver")
    .def(py::init<const SolverParams &, bool>(), "pSParams"_a, "useLazyNACs"_a=false);
#endif
    }

#endif
