# ROCPP
## Introduction
ROC++ is a C++ based platform for modeling, automatically reformulating, and solving robust optimization problems. 

ROC++ can address both single- and multi-stage problems involving exogenous and/or endogenous uncertain parameters and real- and/or binary-valued adaptive variables. 

Please visit our [website](https://sites.google.com/usc.edu/robust-opt-cpp/home) for more detailed information.

## How to use
### Dependencies
ROC++ requires at least one of the following MILP solvers:
* [GUROBI](https://www.gurobi.com/)
* [SCIP](https://www.scipopt.org/)
    * Note that SCIP needs at leaset one LP solver, the supported LP solver interfaces are listed [here](https://www.scipopt.org/doc/html/LPI.php).
    * If you want to solve a nonlinear MIP porblem, you need to have at least one of [NLP solver](https://www.scipopt.org/doc/html/NLPISOLVERS.php) supported by SCIP.
    * The compilation instructions of SCIP are [here](https://www.scipopt.org/doc/html/CMAKE.php) using CMake and [here](https://www.scipopt.org/doc/html/MAKE.php) using Makefiles.

### Compile
#### ROCPP project
After installation of the solvers, we can build the `ROCPP` project using CMake by the following steps:
```bash
mkdir build
cd build
# use the following line to choose SCIP as the solver
cmake -DEXAMPLE=EXAMPLE_NAME -DSOLVER=SCIP ..
# or use the follwoing line to choose Gurobi as the solver and also specify the path to Gurobi
cmake -DEXAMPLE=EXAMPLE_NAME -DGUROBI_DIR=/path/to/gurobi ..
make
```

CMake will build files in a separated directory. Users need to create this directory, usually called `build` or whatever other names. Then run `cmake -DEXAMPLE=EXAMPLE_NAME -DGUROBI_DIR=/path/to/gurobi ..` or `cmake -DEXAMPLE=EXAMPLE_NAME -DSOLVER=SCIP ..` to configure the build. The option `EXAMPLE` indicates the example to run. `SOLVER` set the solver to use. If you choose GUROBI as the solver, you need to specify the path to gurobi to `GUROBI_DIR`. The following table lists detailed information on CMake options.

Next, run `make` to compile the source code. A generated executable can be found in `bin`. In macOS, you need to give the system permission to run the executable. A library `ROCPP.a` will also be created and put in the folder `lib`.

#### ROPY library
We use [pybind11](https://pybind11.readthedocs.io/en/stable/index.html) to create Python bindings of the C++ code. 

We add pybind11 as a submodule of this repo. 
If you clone this repo, an empty `pybind11` directory will be created. Run the following commands to initialize the local configuration files and fetch the data. 
```bash
git submodule init
git submodule update
```
If you directly download the .zip, then there will be an empty pybind11 directory. You need to download it [here](https://github.com/pybind/pybind11) and replace the empty one.

Now we are able to build the `ROPY` library by the following steps. Note that we don't need to create a new build folder if it already existed.
```bash
mkdir build
cd build
# use the following line to choose SCIP as the solver
cmake -DBUILD_PYTHON=ON -DSOLVER=SCIP ..
# or use the follwoing line to choose Gurobi as the solver and also specify the path to Gurobi
cmake -DBUILD_PYTHON=ON -DGUROBI_DIR=/path/to/gurobi ..
make
```

By setting the option `BUILD_PYTHON` to `ON`, we choose to build a library for python.  A library `ROPY.python-version.so` will be created and put in the folder `lib`. To use the library, we need to put it in the same directory of python files and add a line `from ROPY import *` at the top of the file. See more examples in the `scripts/examples_py` folder.

|CMake Options|type    |Avaliable values|Description|
|:-----------:|:------:|:--------------:|:---------------------------------------------------------------------------------------------------|
|EXAMPLE      |string|BB, RSFC, PB    |Name of the example to run in the folder `scripts/examples_cpp`, can also be the names of users created .cpp files|
|SOLVER       |string|GUROBI(default), SCIP    |Name of the solvers that are supported by ROCPP, SCIP by default|
|GUROBI_DIR   |path  |-               |Path to the folder which contains the Gurobi libraries and include folder|
|BUILD_PYTHON |bool  |ON, OFF(default)|If `ON`, then a `.so` library for python will be created, else an executable of the specific example will be generated|
  
## Replicating
To get the same results, run the RSFC, PB, and BB examples following the instructions above.

## Getting start
We briefly introduce how to build optimization models and apply the reformulation methods here. For more detailed class structure of our platform, please see the [doxygen file](https://robust-opt-cpp.github.io/ROCPPDocumentation/).

### Model Input
**Optimization Model**

The script should start with building an optimization model. We list several most frequently used model types here.
- UncSSOptModel: stands for uncertain single stage optimization model, where no adaptive decision variable exists.
```C++
ROCPPOptModelIF_Ptr Model(new ROCPPUncSSOptModel(uncOptModelObjType objType = rboust));
```
```python
Model = RoPyOptModel(uncOptModelObjType.objType = rboust) 
```
- OptModelExoID: stands for optimization model with exogenous information discovery.
```C++
ROCPPOptModelIF_Ptr Model(new ROCPPOptModelExoID(int timeStage, uncOptModelObjType objType = rboust));
```
```python
Model = RoPyOptModelExoID(int timeStage, objType = uncOptModelObjType.rboust) 
```
- OptModelDDID: stands for optimization model with decision dependent information discovery.
```C++
ROCPPOptModelIF_Ptr Model(new ROCPPOptModelDDID(int timeStage, uncOptModelObjType objType = rboust));
```
```python
Model = RoPyOptModelDDID(int timeStage, objType = uncOptModelObjType.rboust) 
```
**Note**: Our platform has limited support of stochastic optimization problem, for which the objType should be assigned 'stochastic' or 'uncOptModelObjType.stochastic'.

**Decision Varaible**

The next components of an optimization model is decision variable. Our platform contains two types of vairables static and adptive.
- StaticVarBool/Int/Double
```C++
ROCPPVarIF_Ptr staticVarReal(new ROCPPStaticVarBool(string varName, double lb = -INFINITY, double ub = INFINITY);
```
```python
staticVarDouble = RoPyStaticVarDouble(string varName, float lb = -INFINITY, float ub = INIFINITY)
```
- AdaptVarBool/Int/Double
```C++
ROCPPVarIF_Ptr AdaptVarReal(new ROCPPStaticVarBool(string varName, int timeStage, double lb = -INFINITY, double ub = INFINITY);
```
```python
staticVarDouble = RoPyStaticVarDouble(string varName, int timeStage, float lb = -INFINITY, float ub = INFINITY)
```
The binary and integer types follow the similar way of definition, note that the default bounds for binary variable is (0, 1).

**Uncertain Parameter**

There are two types for uncertain parameters, the exogenous one whose discovery is independent on the decision, and the endogenous one with decision dependent information discovery. We specify the endogenous one by using extra function adding into the model.
- Create Uncertain Parameter
```C++
ROCPPUnc_Ptr unc(new ROCPPUnc(string uncName, int timeStage = 1));
```
```python
unc = RoPyUnc(string uncName, int timeStage = 1)
```
- Specify the endogenous DDID one to the model
```C++
Model->add_ddu(ROCPPUnc_Ptr unc, int firstObservableTimeStage, int lastObservableTimeStage, map<int, double> observationCostEachStage);
```
```python
Model->add_ddu(RoPyUnc unc, int firstObservableTimeStage, int lastObservableTimeStage, dict{int: double} observationCostEachStage);
```
The 'timeStage' argument represents the time stage that the uncertain parameter is observed. For the DDID one, since the time stage depends on decision variable, we restrict 'timeStage = 1' when construct the parameter.

**Build Uncertainty Set, Add Constraints and Objective Function**

Our platform provides a similar way of operator ovearload to construct an expression and constraint. For concrete exmaples please refer the files in scripts folder. We only list the functions needed here.
```C++
Model->add_constraint_uncset(ROCPPConstraintIF_Ptr cstr);
Model->add_constraint(ROCPPConstraintIF_Ptr cstr);
Model->set_objective(ROCPPExpr_Ptr obj);
```
```Python
Model.add_constraint_uncset(RoPyConstraintIF cstr)
Model.add_constraint(RoPyConstraintIF cstr)
Model.set_objective(RoPyExpr obj)
```

### Reformulate a Model

After build a model, we are at the place to refourmulate the model to a problem that can be solved by off shelf solver. The suported reformulation Strategies are shown in the figure below.![](figures/strategy.png)

The following describes the function of each reformulation strategy and how to construct it.

**Robustify Engine**

It turns a problem with uncertainty into a deterministic form.
```C++
ROCPPStrategy_Ptr pRE(new ROCPPRobustifyEngine ());
```
```Python
pRE = RoPyRobustifyEngine()
```
**Bilinear Term Reformulator**

It linearizes a problem by replacing all the bilinear terms in the problem.
```C++
ROCPPStrategy_Ptr pBTR(new ROCPPBTR_bigM ());
```
```Python
pBTR = RoPyBTR_bigM()
```
**Decision Rule**
It contains all approximation schemes that we dicussed in the paper.
- Linear and Constant Decision Rule
```C++
ROCPPStrategy_Ptr pLDR(new ROCPPLinearDR());
ROCPPStrategy_Ptr pCDR(new ROCPPConstantDR());
```
```Python
pLDR = RoPyLinearDR()
pCDR = RoPyConstantDR()
```
- Piecewise Linear and Constant Decision Rule
```C++
ROCPPStrategy_Ptr pPWApprox(new ROCPPPWDR(map<string,uint> mapUncNametoNumBreakPoints));
```
```Python
pPWApprox = RoPyPWDR(dict{string: int} mapUncNametoNumBreakPoints))
```
- KAdaptability
```C++
ROCPPStrategy_Ptr pKadaptStrategy (new ROCPPKadapt(map<uint, uint> mapTimeStagetoNumPolicies));
```
```Python
pKadaptStrategy = RoPyKadapt(dict{int: int} mapTimeStagetoNumPolicies)
```
**Note**: For more detailed arguments parameter setting please see the doxygen file.

**Reformulation Orchestrator**

Our platform controls the reformulation process by an orchestrator. It takes in a model and a vector of strategies, each strategy will sequentially do its own job. The following is an example shows how to construct an orchestrator and finally reformulate the problem.
```C++
ROCPPOrchestrator_Ptr pOrch(new ROCPPOrchestrator());

vector <ROCPPStrategy_Ptr > strategyVec {pKadaptStrategy, pRE, pBTR};
ROCPPOptModelIF_Ptr reformModel = pOrch->Reformulate(Model, strategyVec);
```
```Python
pOrch = RoPyOrchestrator()

strategyVec = [pKadaptStrategy, pRE, pBTR]
reformModel = pOrch.Reformulate(Model, strategyVec)
```

### Solver Interface
For now we support the solver Gurobi and SCIP. The following lines show how to call a solver and solve the model.
```C++
ROCPPSolver_Ptr pSolver(new ROCPPGurobi(SolverParams()));
//ROCPPSolver_Ptr pSolver(new ROCPPSCIP(SolverParams()));
pSolver->solve(refModel);
```
```Python
pSolver = RoPySolver(RoPySolverParams())
pSolver.solve(refModel)
```
**Note**: For more detailed SolverParams setting, please see the doxygen file.\
**Note**: The solver used in Python is pre-selected in the compilatio step.

## Please cite us
We hope that you find ROC++ useful in your work. **If you do use it, please cite us as**:

P Vayanos, Q Jin, and G Elissaios. [ROC++: Robust Optimization in C++](http://www.optimization-online.org/DB_FILE/2020/06/7835.pdf). Underreview at Informs Journal of Computing, 2020.

**If you use the ROCPPKadapt approximator, please also cite the paper**:

Vayanos P, Georghiou A, Yu H (2019) [Robust optimization with decision-dependent information discovery](http://www.optimization-online.org/DB_HTML/2019/09/7375.html). Under review at Management Science.

**If you use the ROCPPPiecewise approximator, please also cite the paper**:

Vayanos  P,  Kuhn  D,  Rustem  B  (2011)  [Decision  rules  for  information  discovery  in  multi-stage  stochastic programming](https://ieeexplore.ieee.org/document/6161382). Proceedings of the 50th IEEE Conference on Decision and Control , 7368–7373. 
