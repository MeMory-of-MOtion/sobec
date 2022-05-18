cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_NAMESPACE memory-of-motion)
set(PROJECT_NAME sobec)
set(PROJECT_DESCRIPTION "Sandbox for optimal control explicitely for bipeds")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
IF(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  IF(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    MESSAGE(FATAL_ERROR "\nPlease run the following command first:\ngit submodule update --init\n")
  ELSE()
    include(FetchContent)
    FetchContent_Declare("jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  ENDIF()
ENDIF()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

SET(CXX_DISABLE_WERROR True)
ADD_COMPILE_OPTIONS(-Wno-error)

# Project dependencies
find_package(Boost REQUIRED COMPONENTS unit_test_framework)
#ADD_PROJECT_DEPENDENCY(pinocchio 2.4.7 REQUIRED PKG_CONFIG_REQUIRES "pinocchio >= 2.4.7")
ADD_PROJECT_DEPENDENCY(crocoddyl REQUIRED)
#ADD_PROJECT_DEPENDENCY(crocoddyl 1.8.0 REQUIRED PKG_CONFIG_REQUIRES "crocoddyl >= 1.8.0")
#ADD_PROJECT_DEPENDENCY(eigenpy 2.6.4 REQUIRED PKG_CONFIG_REQUIRES "eigenpy >= 2.6.4")
#ADD_PROJECT_DEPENDENCY(example-robot-data 3.7.0 REQUIRED PKG_CONFIG_REQUIRES "example-robot-data >= 3.7.0")

if(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
  SEARCH_FOR_BOOST_PYTHON()
endif()

# Main Library
set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/python.hpp
  include/${PROJECT_NAME}/fwd.hpp

  include/${PROJECT_NAME}/com-velocity.hpp
  include/${PROJECT_NAME}/com-velocity.hxx
  include/${PROJECT_NAME}/quadratic-ref.hpp
)

set(${PROJECT_NAME}_SOURCES
  src/sobec.cpp
  )

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
#target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} PUBLIC crocoddyl::crocoddyl)
#target_link_libraries(${PROJECT_NAME} eigenpy::eigenpy)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

# Main Executable
add_executable(${PROJECT_NAMESPACE}-${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAMESPACE}-${PROJECT_NAME} ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "\$ORIGIN/../lib")

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()

# Unit tests
add_subdirectory(tests)

# Installation
install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
install(TARGETS ${PROJECT_NAMESPACE}-${PROJECT_NAME} RUNTIME DESTINATION bin)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
