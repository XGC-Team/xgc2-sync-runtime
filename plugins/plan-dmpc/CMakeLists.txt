cmake_minimum_required(VERSION 3.16)
project(plan_dmpc LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Standard install locations. Callers pass the prefixes that match the core
# and the acados it was built against. This file does not name a work tree.
set(PLAN_DMPC_CORE_PREFIX "/usr/local" CACHE PATH "Prefix containing formation_generator_dmpc_core")
set(PLAN_DMPC_ACADOS_PREFIX "/usr/local" CACHE PATH "Prefix containing the acados libraries for that core")
# The installed core package reads ACADOS_ROOT. It is the same prefix.
set(ACADOS_ROOT "${PLAN_DMPC_ACADOS_PREFIX}")
list(APPEND CMAKE_PREFIX_PATH "${PLAN_DMPC_CORE_PREFIX}")
find_package(formation_generator_dmpc_core CONFIG REQUIRED)

get_filename_component(PLAN_DMPC_RUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_library(plan_dmpc SHARED plan_dmpc.cpp plan_dmpc_plugin.cpp)
target_include_directories(plan_dmpc PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${PLAN_DMPC_RUNTIME_ROOT}/abi/include"
  "${PLAN_DMPC_RUNTIME_ROOT}/plugins")
target_link_libraries(plan_dmpc PUBLIC
  formation_generator::formation_generator_dmpc_core
  formation_generator::formation_generator_dmpc_config)
set_target_properties(plan_dmpc PROPERTIES
  OUTPUT_NAME "plan_dmpc"
  PREFIX "lib"
  BUILD_RPATH "${PLAN_DMPC_CORE_PREFIX}/lib;${PLAN_DMPC_ACADOS_PREFIX}/lib")

add_executable(test_plan_dmpc test_plan_dmpc.cpp)
target_compile_definitions(test_plan_dmpc PRIVATE
  "PLAN_DMPC_TESTDATA=\"${CMAKE_CURRENT_SOURCE_DIR}/testdata\"")
target_link_libraries(test_plan_dmpc PRIVATE plan_dmpc yaml-cpp dl)
target_link_options(test_plan_dmpc PRIVATE
  "-Wl,-rpath,${PLAN_DMPC_ACADOS_PREFIX}/lib"
  "-Wl,-rpath,${PLAN_DMPC_CORE_PREFIX}/lib")
