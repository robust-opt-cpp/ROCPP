# CHECK CMAKE VERSION
cmake_minimum_required(VERSION 3.3)
# set(CMAKE_C_COMPILER clang)
# set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_STANDARD 11)

# CHOOSE TO BUILD PYTHON PROJECT
option(BUILD_PYTHON "Build the package for python" OFF)

if(BUILD_PYTHON)
    project(ROPY)
else()
    project(ROCPP)
endif() 

# CHOOSE SOLVER
SET(SOLVER "GUROBI" CACHE STRING "The solver to use: SCIP / GUROBI(default)")
string(TOLOWER ${SOLVER} SOLVER)

# FIND SOLVER
if(SOLVER STREQUAL "gurobi")

    message(STATUS "Using Gurobi")

    # SET GUROBI DIRECTORY
    SET(GUROBI_DIR "" CACHE PATH "The path of Gurobi is:")
    message(STATUS "Gurobi path: ${GUROBI_DIR}")

    # SET CMAKE FILE PATH FOR FINDING GUROBI
    SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

    # FIND GUROBI AND INCLUDE HEADER FILES
    find_package(GUROBI REQUIRED)
    include_directories(${GUROBI_INCLUDE_DIRS})

elseif(SOLVER STREQUAL "scip")

    message(STATUS "Using SCIP")

    # FIND SCIP AND INCLUDE HEADER FILES
    find_package(SCIP REQUIRED)
    include_directories(${SCIP_INCLUDE_DIRS})

endif()

# SET SUBDIRECTORY OR EXAMPLE
if(BUILD_PYTHON)
	add_subdirectory(pybind11)
else()
    SET(EXAMPLE "" CACHE STRING "The example to execute")
endif()

# ADD SUBDIRECTORY FOR SOURCE FILE
add_subdirectory(src bin)