set(${PY_NAME}_SOURCES
  main.cpp
  std-containers.cpp
  flexibility_compensation.cpp
  crocomplements/residual-com-velocity.cpp
  crocomplements/residual-vel-collision.cpp
  crocomplements/residual-cop.cpp
  crocomplements/residual-center-of-friction.cpp
  crocomplements/residual-feet-collision.cpp
  crocomplements/residual-fly-high.cpp
  crocomplements/activation-quad-ref.cpp
  crocomplements/lowpassfilter/action.cpp
  crocomplements/lowpassfilter/state.cpp
  crocomplements/contact/contact3d.cpp
  crocomplements/contact/contact1d.cpp
  crocomplements/contact/multiple-contacts.cpp
  crocomplements/contact/contact-fwddyn.cpp
  crocomplements/contact/contact-force.cpp
  crocomplements/softcontact/dam3d.cpp
  crocomplements/softcontact/dam3d-augmented.cpp
  crocomplements/softcontact/iam3d-augmented.cpp
  crocomplements/softcontact/state.cpp
  walk-with-traj/designer.cpp
  walk-with-traj/horizon_manager.cpp
  walk-with-traj/model_factory.cpp
  walk-with-traj/wbc.cpp
  walk-without-think/ocp.cpp
  walk-without-think/mpc.cpp
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
  pinbullet.py
  viewer_multiple.py
  talos_collections.py
  repr_ocp.py
  logs.py
  # talos_low.py
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
  miscdisp.py
  params.py
  yaml_params.py
  plotter.py
  save_traj.py
  )

foreach(python ${${PY_WALK_NAME}_PYTHON})
  PYTHON_INSTALL_ON_SITE(${PY_NAME}/walk_without_think ${python})
endforeach()
