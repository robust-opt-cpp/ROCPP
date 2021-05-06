# INCLUDE HEADER FILES
include_directories(${PROJECT_SOURCE_DIR}/src/include)

# FIND SOURCE FILES
aux_source_directory(. ALL_SRC)

# BUILD LIBRARY
if(BUILD_PYTHON)
    add_library(${CMAKE_PROJECT_NAME} MODULE ${ALL_SRC})
else()
    add_library(${CMAKE_PROJECT_NAME} ${ALL_SRC})
endif()


#LINK SOLVER TO THE PROJECT
if(SOLVER STREQUAL "gurobi")
    target_link_libraries(${CMAKE_PROJECT_NAME} optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
    target_link_libraries(${CMAKE_PROJECT_NAME} ${GUROBI_LIBRARY})
    add_compile_definitions(USE_GUROBI)

elseif(SOLVER STREQUAL "scip")
    target_link_libraries(${CMAKE_PROJECT_NAME} ${SCIP_LIBRARIES})
    add_compile_definitions(USE_SCIP)
endif()

if(BUILD_PYTHON)

    # COMPILE THE INTERFACE CODE
    add_compile_definitions(PYTHON_PACKAGE)

    # LINK THE MODULE TO THE PROJECT
    target_link_libraries(ROPY pybind11::module)

    # SET MODULE'S POSITION AND NAME
    set_target_properties(ROPY PROPERTIES
        PREFIX "${PYTHON_MODULE_PREFIX}" 
        SUFFIX "${PYTHON_MODULE_EXTENSION}"
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
else()
    # ADD THE EXAMPLE TO TEST
    add_executable(${EXAMPLE} ${PROJECT_SOURCE_DIR}/scripts/examples_cpp/${EXAMPLE}.cpp)

    # SET COMPILER DEFINITION FOR CHOOSING SOLVER
    if(SOLVER STREQUAL "gurobi")
        target_compile_definitions(${EXAMPLE} PUBLIC USE_GUROBI)
    elseif(SOLVER STREQUAL "scip")
        target_compile_definitions(${EXAMPLE} PUBLIC USE_SCIP)
    endif()

    # LINK LIBRARIES TO THE EXECUTABLE FILE
    target_link_libraries(${EXAMPLE} PUBLIC ROCPP)
    
    # SET LIBRARY'S POSITION AND NAME
    set_target_properties(ROCPP PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

endif()
