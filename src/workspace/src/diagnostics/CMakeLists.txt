cmake_minimum_required(VERSION 3.0.2)
project(diagnostics)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/diagnostics_websocket.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_install_python(PROGRAMS
    tests/test_diagnostics_websocket.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  )

  catkin_add_nosetests(tests/test_diagnostics_unit.py)
  add_rostest(tests/diagnostics_websocket.test)
endif()
