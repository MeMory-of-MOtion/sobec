set(${PROJECT_NAME}_FACTORY_TEST
  factory/activation.hpp
  factory/cost.hpp
  factory/pinocchio_model.hpp
  factory/random_generator.hpp
  factory/state.hpp
  factory/statelpf.hpp
  factory/lpf.hpp
  factory/diff-action.hpp
  factory/actuation.hpp
  factory/activation.cpp
  factory/cost.cpp
  factory/pinocchio_model.cpp
  factory/state.cpp
  factory/statelpf.cpp
  factory/lpf.cpp
  factory/diff-action.cpp
  factory/actuation.cpp
  )

add_library(${PROJECT_NAME}_unittest SHARED ${${PROJECT_NAME}_FACTORY_TEST})
target_link_libraries(${PROJECT_NAME}_unittest PUBLIC ${PROJECT_NAME} crocoddyl::crocoddyl)

ADD_UNIT_TEST(test_costs test_costs.cpp)
target_link_libraries(test_costs PUBLIC ${PROJECT_NAME}_unittest)

ADD_UNIT_TEST(test_states test_states.cpp)
target_link_libraries(test_states PUBLIC ${PROJECT_NAME}_unittest)

ADD_UNIT_TEST(test_stateslpf test_stateslpf.cpp)
target_link_libraries(test_stateslpf PUBLIC ${PROJECT_NAME}_unittest)

ADD_UNIT_TEST(test_lpf test_lpf.cpp)
target_link_libraries(test_lpf PUBLIC ${PROJECT_NAME}_unittest)

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()
