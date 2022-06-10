cmake_minimum_required(VERSION 3.10)

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

ADD_COMPILE_OPTIONS(-Wno-error)

# Project dependencies
ADD_PROJECT_DEPENDENCY(crocoddyl REQUIRED)

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
  include/${PROJECT_NAME}/residual-com-velocity.hpp
  include/${PROJECT_NAME}/residual-cop.hpp
  include/${PROJECT_NAME}/residual-vel-collision.hpp
  include/${PROJECT_NAME}/residual-fly-high.hpp
  include/${PROJECT_NAME}/activation-quad-ref.hpp
  include/${PROJECT_NAME}/lowpassfilter/statelpf.hpp
  include/${PROJECT_NAME}/lowpassfilter/lpf.hpp
  include/${PROJECT_NAME}/contact/contact3d.hpp
  include/${PROJECT_NAME}/contact/contact1d.hpp
  include/${PROJECT_NAME}/contact/multiple-contacts.hpp
  include/${PROJECT_NAME}/contact/contact-fwddyn.hpp
  include/${PROJECT_NAME}/contact/contact-force.hpp
  include/${PROJECT_NAME}/residual-com-velocity.hxx
  include/${PROJECT_NAME}/residual-cop.hxx
  include/${PROJECT_NAME}/residual-vel-collision.hxx
  include/${PROJECT_NAME}/lowpassfilter/statelpf.hxx
  include/${PROJECT_NAME}/lowpassfilter/lpf.hxx
  include/${PROJECT_NAME}/contact/contact3d.hxx
  include/${PROJECT_NAME}/contact/contact1d.hxx
  include/${PROJECT_NAME}/contact/multiple-contacts.hxx
  include/${PROJECT_NAME}/contact/contact-fwddyn.hxx
  include/${PROJECT_NAME}/contact/contact-force.hxx
  include/${PROJECT_NAME}/residual-fly-high.hxx
 )

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC crocoddyl::crocoddyl)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()

# Unit tests
if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
  add_subdirectory(tests)
endif()

# Installation
install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
