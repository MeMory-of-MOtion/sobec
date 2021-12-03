add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

ADD_UNIT_TEST(add add.cpp)
target_link_libraries(add ${PROJECT_NAME})

ADD_UNIT_TEST(boost_add boost_add.cpp)
target_link_libraries(boost_add ${PROJECT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_test(NAME exe-fail COMMAND ${CMAKE_BINARY_DIR}/${PROJECT_NAMESPACE}-${PROJECT_NAME} 3 4)

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()
