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

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
PYBIND11_MODULE(ROPY, m) {
    m.doc() = "Robust Optimization in Python";

    //python for decision variable
    py::enum_<decVariableType>(m, "decVariableType").value("intDV", decVariableType::intDV).value("boolDV", decVariableType::boolDV).value("contDV", decVariableType::contDV); 
    py::enum_<constraintTermType >(m, "constraintTermType").value("prodTerm", constraintTermType::prodTerm).value("normTerm", constraintTermType::normTerm);
    py::enum_<objType>(m, "objType").value("simpleObj", objType::simpleObj).value("maxObj", objType::maxObj);
    py::enum_<problemType>(m, "problemType").value("uncertainType", problemType::uncertainType).value("dduType", problemType::dduType).value("deterministicType", problemType::deterministicType);
    py::enum_<uncOptModelObjType>(m, "uncOptModelObjType").value("robust", uncOptModelObjType::robust).value("stochastic", uncOptModelObjType::stochastic);
    
    py::class_<DecisionVariableIF, std::shared_ptr<DecisionVariableIF>>(m, "RoPyVarIF").def("getLB", &DecisionVariableIF::getLB).def("getUB", &DecisionVariableIF::getUB).def("setLB", &DecisionVariableIF::setLB).def("setUB", &DecisionVariableIF::setUB)
    .def("isBooleanVar", &DecisionVariableIF::isBooleanVar).def("isIntegerVar", &DecisionVariableIF::isIntegerVar).def("isRealVar", &DecisionVariableIF::isRealVar)
    .def("getType", &DecisionVariableIF::getType).def("getName", &DecisionVariableIF::getName).def("getTimeStage", &DecisionVariableIF::getTimeStage)
    .def("__add__", [](std::shared_ptr<DecisionVariableIF> x, double y){return x + y;})
    .def("__sub__", [](std::shared_ptr<DecisionVariableIF> x, double y){return x - y;})
    .def("__radd__", [](std::shared_ptr<DecisionVariableIF> x, double y){return x + y;})
    .def("__rsub__", [](std::shared_ptr<DecisionVariableIF> x, double y){return (-1.0)*x + y;})
    .def("__add__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<DecisionVariableIF> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<DecisionVariableIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<UncertaintyIF> y){return x + y;})
    .def("__add__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<ConstraintTermIF> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<ConstraintTermIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<LHSExpression> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<LHSExpression> y){return x - y;})
    .def("__mul__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<DecisionVariableIF> y){return x * y;})
    .def("__mul__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<UncertaintyIF> y){return x * y;})
    .def("__mul__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<LHSExpression> y){return x * y;})
    .def("__rmul__", [](std::shared_ptr<DecisionVariableIF> x, float y){return y * x;})
    .def("__le__", [](std::shared_ptr<DecisionVariableIF> x, double y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<DecisionVariableIF> x, double y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<DecisionVariableIF> x, double y){return x == y;})
    .def("__le__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<DecisionVariableIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<DecisionVariableIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<DecisionVariableIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<UncertaintyIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<UncertaintyIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<UncertaintyIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<ConstraintTermIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<ConstraintTermIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<ConstraintTermIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<LHSExpression> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<LHSExpression> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<DecisionVariableIF> x, std::shared_ptr<LHSExpression> y){return x == y;});

    py::class_<VariableInt, std::shared_ptr<VariableInt>, DecisionVariableIF>(m, "RoPyVarInt").def(py::init<string, double, double>(),"name"_a, "lb"_a = -INFINITY, "ub"_a = INFINITY);
    py::class_<VariableDouble, std::shared_ptr<VariableDouble>, DecisionVariableIF>(m, "RoPyVarDouble").def(py::init<string, double, double>(),"name"_a, "lb"_a = -INFINITY, "ub"_a = INFINITY);
    py::class_<VariableBool, std::shared_ptr<VariableBool>, DecisionVariableIF>(m, "RoPyVarBool").def(py::init<string, double, double>(),"name"_a, "lb"_a = 0.0, "ub"_a = 1.0);
    py::class_<AdaptVarBool, std::shared_ptr<AdaptVarBool>, DecisionVariableIF>(m, "RoPyAdaptVarBool").def(py::init<string, uint, double, double>(),"name"_a, "timsStage"_a, "lb"_a = 0.0, "ub"_a = 1.0);
    py::class_<AdaptVarInt, std::shared_ptr<AdaptVarInt>, DecisionVariableIF>(m, "RoPyAdaptVarInt").def(py::init<string, uint, double, double>(),"name"_a, "timsStage"_a, "lb"_a = -INFINITY, "ub"_a = INFINITY);
    py::class_<AdaptVarDouble, std::shared_ptr<AdaptVarDouble>, DecisionVariableIF>(m, "RoPyAdaptVarDouble").def(py::init<string, uint, double, double>(),"name"_a, "timsStage"_a, "lb"_a = -INFINITY, "ub"_a = INFINITY);

    py::class_<UncertaintyIF, std::shared_ptr<UncertaintyIF>>(m, "RoPyUnc").def(py::init<string, uint, bool>())
    .def("getName", &UncertaintyIF::getName).def("getTimeStage", &UncertaintyIF::getTimeStage)
    .def("isObservable", &UncertaintyIF::isObservable)
    .def("__add__", [](std::shared_ptr<UncertaintyIF> x, double y){return x + y;})
    .def("__sub__", [](std::shared_ptr<UncertaintyIF> x, double y){return x - y;})
    .def("__radd__", [](std::shared_ptr<UncertaintyIF> x, double y){return x + y;})
    .def("__rsub__", [](std::shared_ptr<UncertaintyIF> x, double y){return -1.0*x + y;})
    .def("__add__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<DecisionVariableIF> y){return y + x;})
    .def("__sub__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<DecisionVariableIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<UncertaintyIF> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<UncertaintyIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<ConstraintTermIF> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<ConstraintTermIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<LHSExpression> y){return y + x;})
    .def("__sub__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<LHSExpression> y){return x - y;})
    .def("__rmul__", [](std::shared_ptr<UncertaintyIF> x, float y){return y * x;})
    .def("__mul__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<DecisionVariableIF> y){return x * y;})
    .def("__mul__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<LHSExpression> y){return x * y;})
    .def("__le__", [](std::shared_ptr<UncertaintyIF> x, double y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<UncertaintyIF> x, double y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<UncertaintyIF> x, double y){return x == y;})
    .def("__le__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<DecisionVariableIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<DecisionVariableIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<DecisionVariableIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<UncertaintyIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<UncertaintyIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<UncertaintyIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<ConstraintTermIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<ConstraintTermIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<ConstraintTermIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<LHSExpression> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<LHSExpression> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<UncertaintyIF> x, std::shared_ptr<LHSExpression> y){return x == y;});

    py::class_<ConstraintTermIF, std::shared_ptr<ConstraintTermIF>>(m, "ConstraintTermIF").def("isProductTerm", &ConstraintTermIF::isProductTerm).def("isNonlinearProdTerm", &ConstraintTermIF::isNonlinearProdTerm)
    .def("isNormTerm", &ConstraintTermIF::isNormTerm).def("isConstant", &ConstraintTermIF::isConstant).def("isDeterministic", &ConstraintTermIF::isDeterministic).def("isLinear", &ConstraintTermIF::isDeterministic)
    .def("isLinear", &ConstraintTermIF::isLinear).def("isQuadratic", &ConstraintTermIF::isQuadratic).def("getNumContVars", &ConstraintTermIF::getNumContVars).def("getNumIntVars", &ConstraintTermIF::getNumIntVars)
    .def("getNumBoolVars", &ConstraintTermIF::getNumBoolVars).def("getNumAdaptiveContVar", &ConstraintTermIF::getNumAdaptiveContVars).def("getAdaptiveVars", &ConstraintTermIF::getNumAdaptiveVars)
    .def("getNumUncertainties", &ConstraintTermIF::getNumUncertainties)
    .def("__add__", [](std::shared_ptr<ConstraintTermIF> x, double y){return x + y;})
    .def("__sub__", [](std::shared_ptr<ConstraintTermIF> x, double y){return x - y;})
    .def("__radd__", [](std::shared_ptr<ConstraintTermIF> x, double y){return x + y;})
    //.def("__rsub__", [](std::shared_ptr<ConstraintTermIF> x, double y){return -1.0*x + y;})
    .def("__add__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<UncertaintyIF> y){return y + x;})
    .def("__sub__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<UncertaintyIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<DecisionVariableIF> y){return y + x;})
    .def("__sub__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<DecisionVariableIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<ConstraintTermIF> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<ConstraintTermIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<LHSExpression> y){return y + x;})
    .def("__sub__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<LHSExpression> y){return x - y;})
    //.def("__rmul__", [](std::shared_ptr<ConstraintTermIF> x, float y){return y * x;})
    .def("__le__", [](std::shared_ptr<ConstraintTermIF> x, double y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<ConstraintTermIF> x, double y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<ConstraintTermIF> x, double y){return x == y;})
    .def("__le__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<DecisionVariableIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<DecisionVariableIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<DecisionVariableIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<UncertaintyIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<UncertaintyIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<UncertaintyIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<LHSExpression> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<LHSExpression> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<ConstraintTermIF> x, std::shared_ptr<LHSExpression> y){return x == y;});

    py::class_<NormTerm, std::shared_ptr<NormTerm>, ConstraintTermIF>(m, "NormTerm").def(py::init<vector<std::shared_ptr<LHSExpression>>>());

    py::class_<LHSExpression, std::shared_ptr<LHSExpression>>(m, "RoPyExpr").def(py::init<>())
    .def("add", (void (LHSExpression::*)(double)) &LHSExpression::add, "add double to the expression")
    .def("add", (void (LHSExpression::*)(double, std::shared_ptr<DecisionVariableIF>)) &LHSExpression::add, "add decision variable to the expression")
    .def("add", (void (LHSExpression::*)(double, std::shared_ptr<DecisionVariableIF>, std::shared_ptr<DecisionVariableIF>)) &LHSExpression::add, "add product of decision variables to the expression")
    .def("add", (void (LHSExpression::*)(std::shared_ptr<const ConstraintTermIF>)) &LHSExpression::add, "add constraint term to the expression")
    .def("add", (void (LHSExpression::*)(std::shared_ptr<const LHSExpression>)) &LHSExpression::add, "add expression to the expression")
    .def("add", (void (LHSExpression::*)(double, std::shared_ptr<const LHSExpression>)) &LHSExpression::add, "add expression term to the expression")
    .def("add", (void (LHSExpression::*)(double, std::shared_ptr<const LHSExpression>, std::shared_ptr<DecisionVariableIF>)) &LHSExpression::add, "add product of expression and variable to the expression")
    .def("isDeterministic", &LHSExpression::isDeterministic).def("hasNonlinearities", &LHSExpression::hasNonlinearities).def("getNumVars", &LHSExpression::getNumVars)
    .def("__add__", [](std::shared_ptr<LHSExpression> x, double y){return x + y;})
    .def("__sub__", [](std::shared_ptr<LHSExpression> x, double y){return x - y;})
    .def("__radd__", [](std::shared_ptr<LHSExpression> x, double y){return x + y;})
    .def("__rsub__", [](std::shared_ptr<LHSExpression> x, double y){return (-1.0)*x + y;})
    .def("__add__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<UncertaintyIF> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<UncertaintyIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<DecisionVariableIF> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<DecisionVariableIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<ConstraintTermIF> y){return x + y;})
    //.def("__sub__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<ConstraintTermIF> y){return x - y;})
    .def("__add__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<LHSExpression> y){return x + y;})
    .def("__sub__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<LHSExpression> y){return x - y;})
    .def("__mul__", [](std::shared_ptr<LHSExpression> x, double y){return y * x;})
    .def("__rmul__", [](std::shared_ptr<LHSExpression> x, double y){return y * x;})
    .def("__mul__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<DecisionVariableIF> y){return y * x;})
    .def("__mul__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<UncertaintyIF> y){return x * y;})
    //.def("__mul__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<LHSExpression> y){return x * y;})
    .def("__le__", [](std::shared_ptr<LHSExpression> x, double y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<LHSExpression> x, double y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<LHSExpression> x, double y){return x == y;})
    .def("__le__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<DecisionVariableIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<DecisionVariableIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<DecisionVariableIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<UncertaintyIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<UncertaintyIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<UncertaintyIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<ConstraintTermIF> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<ConstraintTermIF> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<ConstraintTermIF> y){return x == y;})
    .def("__le__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<LHSExpression> y){return x <= y;})
    .def("__ge__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<LHSExpression> y){return x >= y;})
    .def("__eq__", [](std::shared_ptr<LHSExpression> x, std::shared_ptr<LHSExpression> y){return x == y;})
    .def("getConst", &LHSExpression::getSumConstantTerms).def("getNumTerms", &LHSExpression::getNumTerms);

    py::class_<ConstraintIF, std::shared_ptr<ConstraintIF>>(m, "RoPyConstraint").def("isClassicConstraint", &ConstraintIF::isClassicConstraint).def("SetRhs", &ConstraintIF::set_rhs);

    py::class_<ObjectiveFunctionIF, std::shared_ptr<ObjectiveFunctionIF>>(m, "ObjectiveFunctionIF").def("getObjType", &ObjectiveFunctionIF::getObjType).def("isDeterministic", &ObjectiveFunctionIF::isDeterministic);
    py::class_<SimpleObjective, std::shared_ptr<SimpleObjective>, ObjectiveFunctionIF>(m, "SimpleObjective");
    py::class_<MaxObjective, std::shared_ptr<MaxObjective>, ObjectiveFunctionIF>(m, "MaxObjective");
    
    py::class_<OptimizationModelIF, std::shared_ptr<OptimizationModelIF>>(m, "RoPyOptModelIF")
    .def("writeToFile", &OptimizationModelIF::WriteToFile)
    .def("addConstraint", &OptimizationModelIF::add_constraint)
    .def("addConstraints", &OptimizationModelIF::add_constraints)
    .def("addSOCConstraint", &OptimizationModelIF::add_soc_constraint)
    .def("addConstraintUncSet", &OptimizationModelIF::add_constraint_uncset)
    .def("setObjective", (void (OptimizationModelIF::*)(std::shared_ptr<LHSExpression>)) &OptimizationModelIF::set_objective)
    .def("setObjective", (void (OptimizationModelIF::*)(vector<std::shared_ptr<LHSExpression>>)) &OptimizationModelIF::set_objective)
    .def("getMeasVar", &OptimizationModelIF::getMeasVar)
    .def("getVar", &OptimizationModelIF::getVar)
    .def("getUnc", &OptimizationModelIF::getUnc)
    .def("addDDU", &OptimizationModelIF::add_ddu);
    
    py::class_<DeterministicOptimizationModel, std::shared_ptr<DeterministicOptimizationModel>, OptimizationModelIF>(m, "RoPyDetOptModel").def(py::init<>());
    py::class_<MISOCP, std::shared_ptr<MISOCP>, DeterministicOptimizationModel>(m, "RoPyMISOCP").def(py::init<>());
    py::class_<UncertainOptimizationModel, std::shared_ptr<UncertainOptimizationModel>, OptimizationModelIF>(m, "RoPyUncOptModel")
    .def(py::init<uint, uncOptModelObjType>(), "numTimeStage"_a = 1, "objType"_a = robust);
    py::class_<UncertainSingleStageOptimizationModel, std::shared_ptr<UncertainSingleStageOptimizationModel>, UncertainOptimizationModel>(m, "RoPyUncSSOptModel");
    py::class_<DDUOptimizationModel, std::shared_ptr<DDUOptimizationModel>, UncertainOptimizationModel>(m, "RoPyDDUOptModel")
    .def(py::init<uint, uncOptModelObjType>(), "numTimeStage"_a = 1, "objType"_a = robust);

    py::class_<SolverParams, std::shared_ptr<SolverParams>>(m, "RoPySolverParams")
    .def(py::init<bool, bool, pair<bool,double>, pair<bool,double>, pair<bool,double>, pair<bool,double>, pair<bool,double>, pair<bool,double>, double>(), "useLazyNACs"_a=false, "verbose"_a = true, "epIntLimit"_a = make_pair(false,0.), "timeLimit"_a = make_pair(false,0.), "epGapLimit"_a = make_pair(false,1.e-4),  "epAGapLimit"_a = make_pair(false,1.e-10),
                  "epOptLimit"_a = make_pair(false,1.e-6),  "epRHSLimit"_a = make_pair(false,1.e-9),  "SOSeps"_a = 1.e-20);

    py::class_<SolverModellerIF, std::shared_ptr<SolverModellerIF>>(m, "RoPySolver")
    .def("getOptValue", &SolverModellerIF::getOptValue)
    .def("getSolution", (map<string, double> (SolverModellerIF::*)() const) &SolverModellerIF::getSolution)
    .def("getSolution", (double (SolverModellerIF::*)(string) const) &SolverModellerIF::getSolution)
    .def("solve", &SolverModellerIF::solve, "pModelIn"_a, "writeSlnToFile"_a = false, "fileName"_a = "", "writeSlnToConsole"_a = true, "WSVars"_a = (map<string,double>()), "priorities"_a = (map<string,int>()), "deleteModel"_a = false);
    
#ifdef USE_GUROBI
    py::class_<GurobiModeller, std::shared_ptr<GurobiModeller>, SolverModellerIF>(m, "RoPyGurobi")
    .def(py::init<const SolverParams &, bool>(), "pSParams"_a, "useLazyNACs"_a=false);
#elif defined(USE_SCIP)
    py::class_<SCIPModeller, std::shared_ptr<SCIPModeller>, SolverModellerIF>(m, "RoPySCIP")
    .def(py::init<const SolverParams &, bool>(), "pSParams"_a, "useLazyNACs"_a=false);
#endif
    
    py::class_<RobustifyEngine, std::shared_ptr<RobustifyEngine>>(m, "RoPyRobEng")
    .def(py::init<uint, string, string>(), "dualVarsCounter"_a = 0, "dualNme_suff"_a = "", "dualNme"_a = "dual")
    .def("robustify", &RobustifyEngine::robustify);
    
    py::class_<KadaptabilityPartitionEncoderMS, std::shared_ptr<KadaptabilityPartitionEncoderMS>>(m, "RoPyKadaptEncoder")
    .def(py::init<const map<uint,uint> &>(), "KMap"_a);

    py::class_<DDUApproximatorIF, std::shared_ptr<DDUApproximatorIF>>(m, "RoPyAproximator")
    .def("approximate", (std::shared_ptr<MISOCP> (DDUApproximatorIF::*)(std::shared_ptr<OptimizationModelIF>) ) &DDUApproximatorIF::approximate)
    .def("printParameters", &DDUApproximatorIF::printParametersToScreen)
    .def("getParameters", &DDUApproximatorIF::getSolutionApproachParameters)
    .def("printOut", (void (DDUApproximatorIF::*)(const std::shared_ptr<OptimizationModelIF>, const map<string, double> &, std::shared_ptr<DecisionVariableIF>) ) &DDUApproximatorIF::printOut)
    .def("printOut", (void (DDUApproximatorIF::*)(const std::shared_ptr<OptimizationModelIF>, const map<string, double> &, std::shared_ptr<UncertaintyIF>) ) &DDUApproximatorIF::printOut);
    
    py::class_<KadaptabilityApproximatorMS, std::shared_ptr<KadaptabilityApproximatorMS>, DDUApproximatorIF>(m, "RoPyKadaptApprox")
    .def(py::init<std::shared_ptr<OptimizationModelIF>, uint, double, double, string>(), "pIn"_a, "K"_a, "bigM"_a = 100.0, "epsilon"_a = 0.0001, "folder"_a=" ")
    .def(py::init<std::shared_ptr<KadaptabilityPartitionEncoderMS>, double, double, string>(), "MSK"_a, "bigM"_a = 100.0, "epsilon"_a = 0.0001, "folder"_a=" ")
    .def("fixHeuristicSolutions", &KadaptabilityApproximatorMS::fixSecondStageVariablesToWarmStart)
    .def("getHeuristicSolutions",
        &KadaptabilityApproximatorMS::getWsSolutions)
    .def("printOut", (void (KadaptabilityApproximatorMS::*)(const std::shared_ptr<OptimizationModelIF>, const map<string, double> &, std::shared_ptr<DecisionVariableIF>, string partition)) &KadaptabilityApproximatorMS::printOut)
    .def("printOut", (void (KadaptabilityApproximatorMS::*)(const std::shared_ptr<OptimizationModelIF>, const map<string, double> &, std::shared_ptr<UncertaintyIF>, string partition)) &KadaptabilityApproximatorMS::printOut);
    
    py::class_<PiecewiseApproximator, std::shared_ptr<PiecewiseApproximator>, DDUApproximatorIF>(m, "RoPyPiecewiseApprox")
    .def(py::init<std::shared_ptr<OptimizationModelIF>, string, uint, double, bool, string>(), "pIn"_a, "numPartitionsStr"_a, "numBits"_a = 5, "bigM"_a = 100., "useExplicitNACs"_a = false, "folder"_a=" ")
    .def(py::init<std::shared_ptr<OptimizationModelIF>, const map<string,uint> &, uint, double, bool, string>(), "pIn"_a, "numPartitionsMap"_a = map<string, uint>(), "numBits"_a = 5,"bigM"_a = 100.0, "epsilon"_a = 0.0001, "folder"_a=" ")
    .def("printOut", (void (PiecewiseApproximator::*)(const std::shared_ptr<OptimizationModelIF>, const map<string, double> &, std::shared_ptr<DecisionVariableIF>, map<std::shared_ptr<UncertaintyIF>, uint> partitionIn) ) &PiecewiseApproximator::printOut)
    .def("printOut", (void (PiecewiseApproximator::*)(const std::shared_ptr<OptimizationModelIF>, const map<string, double> &, std::shared_ptr<UncertaintyIF>, map<std::shared_ptr<UncertaintyIF>, uint> partitionIn) ) &PiecewiseApproximator::printOut);
    
    py::class_<LDRCDRApproximator, std::shared_ptr<LDRCDRApproximator>, DDUApproximatorIF>(m, "RoPyLCDRApprox")
    .def(py::init<std::shared_ptr<OptimizationModelIF>, uint, uint, double, string>(), "pIn"_a, "memory"_a = 1000, "numBits"_a = 5, "bigM"_a = 100., "folder"_a=" ");
    }

#endif
