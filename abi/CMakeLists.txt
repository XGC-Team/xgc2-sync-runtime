cmake_minimum_required(VERSION 3.16)
project(XgcRuntimeSDK VERSION 0.1.0 LANGUAGES NONE)

# Header-only SDK. This entry point can also be added explicitly from an owner
# project with add_subdirectory(${XGC_RUNTIME_SDK_SOURCE_ROOT}/abi ...).
# It neither builds the Host nor configures any domain plugin or transport.
include(CMakePackageConfigHelpers)
set(CMAKE_INSTALL_INCLUDEDIR "include" CACHE PATH "Header installation directory")
set(CMAKE_INSTALL_DATADIR "share" CACHE PATH "Architecture-independent data directory")
set(sdk_package_dir "${CMAKE_INSTALL_DATADIR}/cmake/XgcRuntimeSDK")

add_library(xgc_runtime_sdk INTERFACE)
add_library(XgcRuntime::SDK ALIAS xgc_runtime_sdk)
set_target_properties(xgc_runtime_sdk PROPERTIES EXPORT_NAME SDK)
target_include_directories(xgc_runtime_sdk INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../plugins/common>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/xgc-runtime>
)

# These are the canonical files; do not create a second copy in this source tree.
install(FILES
  include/xgc_rt.h
  include/xgc_clock_source.h
  include/xgc_schemas_v1.h
  include/xgc_dmpc_planner_v1.h
  ../plugins/common/flat_config.hpp
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/xgc-runtime"
)
install(TARGETS xgc_runtime_sdk EXPORT XgcRuntimeSDKTargets)
install(EXPORT XgcRuntimeSDKTargets
  NAMESPACE XgcRuntime::
  DESTINATION "${sdk_package_dir}"
)
configure_package_config_file(
  cmake/XgcRuntimeSDKConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/XgcRuntimeSDKConfig.cmake"
  INSTALL_DESTINATION "${sdk_package_dir}"
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/XgcRuntimeSDKConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
  ARCH_INDEPENDENT
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/XgcRuntimeSDKConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/XgcRuntimeSDKConfigVersion.cmake"
  DESTINATION "${sdk_package_dir}"
)
