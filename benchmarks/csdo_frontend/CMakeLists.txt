cmake_minimum_required(VERSION 3.16)
project(CsdoRootWarmstartExporter LANGUAGES CXX)

set(CSDO_SOURCE_DIR "" CACHE PATH "Path to the external CSDO source tree")
if(NOT EXISTS "${CSDO_SOURCE_DIR}/pbs/PBS.h")
    message(FATAL_ERROR "Set CSDO_SOURCE_DIR to the CSDO source tree")
endif()

find_package(Boost 1.65 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(
    csdo_root_warmstart_exporter
    root_warmstart_exporter.cpp
    "${CSDO_SOURCE_DIR}/common/motion_planning.cc"
    "${CSDO_SOURCE_DIR}/hybrid_a_star/Instance.cc"
    "${CSDO_SOURCE_DIR}/hybrid_a_star/hybrid_astar_interface.cc"
    "${CSDO_SOURCE_DIR}/pbs/Conflict.cc"
    "${CSDO_SOURCE_DIR}/pbs/PBS.cc"
    "${CSDO_SOURCE_DIR}/pbs/PBSNode.cc"
    "${CSDO_SOURCE_DIR}/sqp/corridor.cc"
    "${CSDO_SOURCE_DIR}/sqp/inter_agent_cons.cc"
    "${CSDO_SOURCE_DIR}/util/file_utils.cc"
)
target_compile_features(csdo_root_warmstart_exporter PRIVATE cxx_std_14)
target_include_directories(
    csdo_root_warmstart_exporter
    PRIVATE
        "${CSDO_SOURCE_DIR}"
        ${Boost_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(
    csdo_root_warmstart_exporter
    PRIVATE
        Eigen3::Eigen
        yaml-cpp
        ${OMPL_LIBRARIES}
)
