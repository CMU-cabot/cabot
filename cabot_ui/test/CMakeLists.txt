find_package(ament_cmake_pytest REQUIRED)
set(_pytest_tests
  test_event.py
  test_geojson.py
  test_geoutil.py
  test_i18n.py
  test_navigation.py
  test_status.py
  # Add other test files here
)

foreach(_test_path ${_pytest_tests})
  get_filename_component(_test_name ${_test_path} NAME_WE)
  ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index
  )
endforeach()
