FILE(GLOB_RECURSE ${PROJECT_NAME}_FACTORY_TEST
  ${CMAKE_CURRENT_SOURCE_DIR}/factory/*.cpp
  )

ADD_LIBRARY(${PROJECT_NAME}_unittest SHARED ${${PROJECT_NAME}_FACTORY_TEST})
SET_TARGET_PROPERTIES(${PROJECT_NAME}_unittest PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_unittest ${PROJECT_NAME} example-robot-data::example-robot-data)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_unittest ${PROJECT_NAME} crocoddyl::crocoddyl)

add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

ADD_UNIT_TEST(add add.cpp)
target_link_libraries(add ${PROJECT_NAME})

ADD_UNIT_TEST(boost_add boost_add.cpp)
target_link_libraries(boost_add ${PROJECT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

ADD_UNIT_TEST(boost_pin boost_pin)
target_link_libraries(boost_pin ${PROJECT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

ADD_UNIT_TEST(boost_croco boost_croco)
target_link_libraries(boost_croco ${PROJECT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

ADD_UNIT_TEST(test_costs test_costs)
target_link_libraries(test_costs ${PROJECT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_link_libraries(test_costs ${PROJECT_NAME} ${PROJECT_NAME}_unittest)

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()
