# ADD_PYTHON_UNIT_TEST("py-walk-with-traj" "tests/python/test_walk_with_traj.py"
# "python")
add_python_unit_test("py-vel-collision" "tests/python/test_vel_collision.py"
                     "python")
add_python_unit_test("py-cop" "tests/python/test_cop.py" "python")
add_python_unit_test("py-fly-high" "tests/python/test_fly_high.py" "python")
add_python_unit_test("py-test-collision" "tests/python/test_feet_collision.py"
                     "python")
add_python_unit_test("py-test-mpc-walk" "tests/python/test_mpc_walk.py"
                     "python")
add_python_unit_test("py-walk" "tests/python/test_walk.py" "python")
add_python_unit_test("py-mpc-walk-complex"
                     "tests/python/test_mpc_walk__complex.py" "python")
add_python_unit_test("py-ocpcpp" "tests/python/test_ocpcpp.py" "python")
add_python_unit_test("py-yaml" "tests/python/test_yaml.py" "python")
add_python_unit_test("py-ref-force" "tests/python/test_ref_force.py" "python")

string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWER)
# Deactivate temporarily this test ... maybe it is not very interesting. if
# (CMAKE_BUILD_TYPE_LOWER STREQUAL "release") ADD_PYTHON_UNIT_TEST("py-mpccpp"
# "tests/python/test_mpccpp.py" "python") endif()
