##
##     This file is part of qpOASES.
##
##     qpOASES -- An Implementation of the Online Active Set Strategy.
##     Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
##     Christian Kirches et al. All rights reserved.
##
##     qpOASES is free software; you can redistribute it and/or
##     modify it under the terms of the GNU Lesser General Public
##     License as published by the Free Software Foundation; either
##     version 2.1 of the License, or (at your option) any later version.
##
##     qpOASES is distributed in the hope that it will be useful,
##     but WITHOUT ANY WARRANTY; without even the implied warranty of
##     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##     See the GNU Lesser General Public License for more details.
##
##     You should have received a copy of the GNU Lesser General Public
##     License along with qpOASES; if not, write to the Free Software
##     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##     Filename:  CMakeLists.txt
##     Author:    Hans Joachim Ferreau (thanks to Milan Vukov)
##     Version:   3.2
##     Date:      2007-2017
##

cmake_minimum_required(VERSION 3.5)

PROJECT(qpOASES CXX)
SET(PACKAGE_NAME "qpOASES")
SET(PACKAGE_VERSION "3.2.2")
SET(PACKAGE_SO_VERSION "3.2")
SET(PACKAGE_DESCRIPTION "An implementation of the online active set strategy")
SET(PACKAGE_AUTHOR "Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.")
SET(PACKAGE_MAINTAINER "Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.")
SET(PACKAGE_URL "https://projects.coin-or.org/qpOASES")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)

IF(NOT CMAKE_INSTALL_LIBDIR)
    SET(CMAKE_INSTALL_LIBDIR lib)
ENDIF(NOT CMAKE_INSTALL_LIBDIR)

IF(NOT CMAKE_INSTALL_BINDIR)
    SET(CMAKE_INSTALL_BINDIR ${CMAKE_INSTALL_LIBDIR})
ENDIF(NOT CMAKE_INSTALL_BINDIR)

IF(NOT CMAKE_INSTALL_INCLUDEDIR)
    SET(CMAKE_INSTALL_INCLUDEDIR include)
ENDIF(NOT CMAKE_INSTALL_INCLUDEDIR)


IF( NOT CMAKE_VERBOSE_MAKEFILE )
    SET( CMAKE_VERBOSE_MAKEFILE OFF )
ENDIF( NOT CMAKE_VERBOSE_MAKEFILE )

IF( NOT CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE
        )
ENDIF( NOT CMAKE_BUILD_TYPE )

option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" OFF)
option(QPOASES_BUILD_EXAMPLES "Build examples." ON)
option(QPOASES_BUILD_NONLINEAR_TESTS "Build TurboSCP nonlinear tests." ON)
option(QPOASES_BUILD_EXTERNAL_BENCHMARKS "Build OSQP, MOSEK, and HPIPM comparisons." OFF)
option(QPOASES_AVOID_LA_NAMING_CONFLICTS "If ON, avoid to re-defined symbols that conflict with Blas/Lapack provided functions." OFF)

IF(BUILD_SHARED_LIBS AND WIN32)
    MESSAGE(FATAL_ERROR "Compiling qpOASES as a shared library in Windows is not supported.")
ENDIF()

############################################################
#################### compiler flags ########################
############################################################
# Find and enable OpenMP for parallel execution
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found - enabling parallel ADMM")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    message(WARNING "OpenMP not found - ADMM will run sequentially")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
IF(QPOASES_AVOID_LA_NAMING_CONFLICTS)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__AVOID_LA_NAMING_CONFLICTS__")
ENDIF()

IF ( UNIX )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -O3 -finline-functions")
ELSEIF( WINDOWS )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -nologo -EHsc -DWIN32")
ENDIF()

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D__DEBUG__")

############################################################
######################## rpath #############################
############################################################
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/casadi")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:${CMAKE_INSTALL_PREFIX}/lib/casadi")
endif("${isSystemDir}" STREQUAL "-1")



############################################################
#################### build and install #####################
############################################################
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/local/include)
# compile qpOASES libraries
FILE(GLOB SRC src/*.cpp)

# library
ADD_LIBRARY(qpOASES ${SRC})
INSTALL(TARGETS qpOASES
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
SET_TARGET_PROPERTIES(
    qpOASES
    PROPERTIES
    SOVERSION ${PACKAGE_SO_VERSION}
    )

# headers
INSTALL(FILES include/qpOASES.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

INSTALL(DIRECTORY include/qpOASES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*.ipp"
  PATTERN ".svn" EXCLUDE)

############################################################
######################### Package Config ###################
############################################################

include(CMakePackageConfigHelpers)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
configure_package_config_file(qpOASESConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/qpOASESConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/qpOASES
  PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/qpOASESConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/qpOASESConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/qpOASESConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/qpOASES
    COMPONENT qpOASES)

############################################################
######################### examples #########################
############################################################

if (QPOASES_BUILD_EXAMPLES)
    # compile qpOASES examples
    SET(EXAMPLE_NAMES
        example1
        example1a
        example1b
        example2
        example3
        example3b
        example4
        example5
        exampleLP
        qrecipe
        qrecipeSchur
    )

    FOREACH(ELEMENT ${EXAMPLE_NAMES})
        ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
        TARGET_LINK_LIBRARIES(${ELEMENT} qpOASES)
    ENDFOREACH(ELEMENT ${EXAMPLE_NAMES})
endif(QPOASES_BUILD_EXAMPLES)

############################################################
######################### tests ###########################
############################################################

if(QPOASES_BUILD_NONLINEAR_TESTS)
    enable_testing()
    ADD_EXECUTABLE(test_nonlinear_model tests/test_nonlinear_model.cpp)
    TARGET_LINK_LIBRARIES(test_nonlinear_model qpOASES)
    add_test(NAME nonlinear_model_jacobians COMMAND test_nonlinear_model)
    ADD_EXECUTABLE(test_stage_varying_riccati tests/test_stage_varying_riccati.cpp)
    TARGET_LINK_LIBRARIES(test_stage_varying_riccati qpOASES)
    add_test(NAME stage_varying_riccati COMMAND test_stage_varying_riccati)
    ADD_EXECUTABLE(test_nonlinear_two_agent tests/test_nonlinear_two_agent.cpp)
    TARGET_LINK_LIBRARIES(test_nonlinear_two_agent qpOASES)
    add_test(NAME nonlinear_two_agent COMMAND test_nonlinear_two_agent)
    ADD_EXECUTABLE(test_heterogeneous_models tests/test_heterogeneous_models.cpp)
    TARGET_LINK_LIBRARIES(test_heterogeneous_models qpOASES)
    add_test(NAME heterogeneous_model_jacobians COMMAND test_heterogeneous_models)
    ADD_EXECUTABLE(
        test_heterogeneous_three_agent
        tests/test_heterogeneous_three_agent.cpp
    )
    TARGET_LINK_LIBRARIES(test_heterogeneous_three_agent qpOASES)
    add_test(
        NAME heterogeneous_three_agent
        COMMAND test_heterogeneous_three_agent
    )
    ADD_EXECUTABLE(
        test_nonlinear_static_obstacles
        tests/test_nonlinear_static_obstacles.cpp
    )
    TARGET_LINK_LIBRARIES(test_nonlinear_static_obstacles qpOASES)
    add_test(NAME nonlinear_static_obstacles COMMAND test_nonlinear_static_obstacles)
    ADD_CUSTOM_TARGET(nonlinear_heterogeneous_benchmark
        COMMAND $<TARGET_FILE:test_heterogeneous_three_agent>
            --csv ${CMAKE_BINARY_DIR}/heterogeneous_benchmark.csv
        DEPENDS test_heterogeneous_three_agent)
endif()

if(QPOASES_BUILD_EXTERNAL_BENCHMARKS)
find_package(osqp REQUIRED)

# Find MOSEK
set(MOSEK_ROOT_DIR "/mnt/c/Users/cheny/Documents/GitHub/mosek/11.0/tools/platform/linux64x86")
set(MOSEK_INCLUDE_DIR "${MOSEK_ROOT_DIR}/h")
set(MOSEK_LIBRARY_DIR "${MOSEK_ROOT_DIR}/bin")

# Check if MOSEK directories exist
if(EXISTS "${MOSEK_INCLUDE_DIR}" AND EXISTS "${MOSEK_LIBRARY_DIR}")
    message(STATUS "MOSEK found at ${MOSEK_ROOT_DIR}")
    set(MOSEK_FOUND TRUE)
    include_directories(${MOSEK_INCLUDE_DIR})
    link_directories(${MOSEK_LIBRARY_DIR})
else()
    message(WARNING "MOSEK not found at ${MOSEK_ROOT_DIR}")
    set(MOSEK_FOUND FALSE)
endif()

link_directories(/usr/local/lib)

# Find BLASFEO (required by HPIPM)
set(BLASFEO_ROOT_DIR "/opt/blasfeo")
set(BLASFEO_INCLUDE_DIR "${BLASFEO_ROOT_DIR}/include")
set(BLASFEO_LIBRARY_DIR "${BLASFEO_ROOT_DIR}/lib")

# Check if BLASFEO directories exist
if(EXISTS "${BLASFEO_INCLUDE_DIR}" AND EXISTS "${BLASFEO_LIBRARY_DIR}")
    message(STATUS "BLASFEO found at ${BLASFEO_ROOT_DIR}")
    set(BLASFEO_FOUND TRUE)
    include_directories(${BLASFEO_INCLUDE_DIR})
    link_directories(${BLASFEO_LIBRARY_DIR})
else()
    message(WARNING "BLASFEO not found at ${BLASFEO_ROOT_DIR}")
    set(BLASFEO_FOUND FALSE)
endif()

# Find HPIPM
set(HPIPM_ROOT_DIR "/opt/hpipm")
set(HPIPM_INCLUDE_DIR "${HPIPM_ROOT_DIR}/include")
set(HPIPM_LIBRARY_DIR "${HPIPM_ROOT_DIR}/lib")

# Check if HPIPM directories exist
if(EXISTS "${HPIPM_INCLUDE_DIR}" AND EXISTS "${HPIPM_LIBRARY_DIR}")
    message(STATUS "HPIPM found at ${HPIPM_ROOT_DIR}")
    set(HPIPM_FOUND TRUE)
    include_directories(${HPIPM_INCLUDE_DIR})
    link_directories(${HPIPM_LIBRARY_DIR})
else()
    message(WARNING "HPIPM not found at ${HPIPM_ROOT_DIR}")
    set(HPIPM_FOUND FALSE)
endif()

# OSQP tests
ADD_EXECUTABLE(test_2agent_osqp tests/test_2agent_osqp.cpp)
TARGET_LINK_LIBRARIES(test_2agent_osqp qpOASES osqp)

ADD_EXECUTABLE(test_4agent_osqp tests/test_4agent_osqp.cpp)
TARGET_LINK_LIBRARIES(test_4agent_osqp qpOASES osqp)

ADD_EXECUTABLE(test_6agent_osqp tests/test_6agent_osqp.cpp)
TARGET_LINK_LIBRARIES(test_6agent_osqp qpOASES osqp)

ADD_EXECUTABLE(test_8agent_osqp tests/test_8agent_osqp.cpp)
TARGET_LINK_LIBRARIES(test_8agent_osqp qpOASES osqp)

ADD_EXECUTABLE(test_10agent_osqp tests/test_10agent_osqp.cpp)
TARGET_LINK_LIBRARIES(test_10agent_osqp qpOASES osqp)

ADD_EXECUTABLE(test_14agent_osqp tests/test_14agent_osqp.cpp)
TARGET_LINK_LIBRARIES(test_14agent_osqp qpOASES osqp)

# MOSEK tests (only build if MOSEK is found)
if(MOSEK_FOUND)
    ADD_EXECUTABLE(test_2agent_mosek tests/test_2agent_mosek.cpp)
    TARGET_LINK_LIBRARIES(test_2agent_mosek qpOASES mosek64)

    ADD_EXECUTABLE(test_4agent_mosek tests/test_4agent_mosek.cpp)
    TARGET_LINK_LIBRARIES(test_4agent_mosek qpOASES mosek64)

    ADD_EXECUTABLE(test_6agent_mosek tests/test_6agent_mosek.cpp)
    TARGET_LINK_LIBRARIES(test_6agent_mosek qpOASES mosek64)

    ADD_EXECUTABLE(test_8agent_mosek tests/test_8agent_mosek.cpp)
    TARGET_LINK_LIBRARIES(test_8agent_mosek qpOASES mosek64)

    ADD_EXECUTABLE(test_10agent_mosek tests/test_10agent_mosek.cpp)
    TARGET_LINK_LIBRARIES(test_10agent_mosek qpOASES mosek64)

    ADD_EXECUTABLE(test_14agent_mosek tests/test_14agent_mosek.cpp)
    TARGET_LINK_LIBRARIES(test_14agent_mosek qpOASES mosek64)

    message(STATUS "MOSEK tests will be built")
else()
    message(STATUS "MOSEK tests will be skipped (MOSEK not found)")
endif()

# HPIPM tests (only build if HPIPM is found)
if(HPIPM_FOUND)
    ADD_EXECUTABLE(test_2agent_hpipm tests/test_2agent_hpipm.cpp)
    TARGET_LINK_LIBRARIES(test_2agent_hpipm qpOASES hpipm blasfeo)

    ADD_EXECUTABLE(test_4agent_hpipm tests/test_4agent_hpipm.cpp)
    TARGET_LINK_LIBRARIES(test_4agent_hpipm qpOASES hpipm blasfeo)

    ADD_EXECUTABLE(test_6agent_hpipm tests/test_6agent_hpipm.cpp)
    TARGET_LINK_LIBRARIES(test_6agent_hpipm qpOASES hpipm blasfeo)

    message(STATUS "HPIPM tests will be built")
else()
    message(STATUS "HPIPM tests will be skipped (HPIPM not found)")
endif()
endif()

##
##   end of file
##
