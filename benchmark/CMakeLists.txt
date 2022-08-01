SET(${PROJECT_NAME}_BENCHMARK
  bench-mpc-walk
  )


FOREACH(BENCHMARK_NAME ${${PROJECT_NAME}_BENCHMARK})
  ADD_EXECUTABLE(${BENCHMARK_NAME} ${BENCHMARK_NAME}.cpp)
  target_link_libraries(${BENCHMARK_NAME} PUBLIC ${PROJECT_NAME} crocoddyl::crocoddyl)
ENDFOREACH(BENCHMARK_NAME ${${PROJECT_NAME}_BENCHMARK})

if(BUILD_PY2CPP_INTERFACE)
  target_compile_definitions(bench-mpc-walk PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  target_link_libraries(bench-mpc-walk PUBLIC ${PROJECT_NAME}_py2cpp)
endif()
