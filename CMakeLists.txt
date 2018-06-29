cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME example-adder)
set(PROJECT_DESCRIPTION "This is an example project, to show how to use Gepetto's tools")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

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

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
