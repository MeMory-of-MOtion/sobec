set(${PY_NAME}_SOURCES
  main.cpp
  residual-com-velocity.cpp
  residual-vel-collision.cpp
  residual-cop.cpp
  residual-feet-collision.cpp
  residual-fly-high.cpp
  activation-quad-ref.cpp
  lpf.cpp
  contact3d.cpp
  contact1d.cpp
  multiple-contacts.cpp
  contact-fwddyn.cpp
  contact-force.cpp
  mpc-walk.cpp
  )

add_library(${PY_NAME}_pywrap SHARED ${${PY_NAME}_SOURCES})
target_link_libraries(${PY_NAME}_pywrap PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
target_include_directories(${PY_NAME}_pywrap SYSTEM PRIVATE ${PYTHON_INCLUDE_DIR})
set_target_properties(${PY_NAME}_pywrap PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../../.."
  LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
  PREFIX ""
  SUFFIX ${PYTHON_EXT_SUFFIX})
TARGET_LINK_BOOST_PYTHON(${PY_NAME}_pywrap PRIVATE)
install(TARGETS ${PY_NAME}_pywrap DESTINATION ${${PY_NAME}_INSTALL_DIR})

set(${PY_NAME}_PYTHON
  __init__.py
  )

foreach(python ${${PY_NAME}_PYTHON})
  PYTHON_INSTALL_ON_SITE(${PY_NAME} ${python})
endforeach()

set(${PY_WALK_NAME}_PYTHON
  __init__.py
  ocp.py
  weight_share.py
  robot_wrapper.py
  config_mpc.py
  )

foreach(python ${${PY_WALK_NAME}_PYTHON})
  PYTHON_INSTALL_ON_SITE(${PY_NAME}/walk ${python})
endforeach()
