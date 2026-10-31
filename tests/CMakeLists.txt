find_package(Catch2 3 REQUIRED)

add_executable(unit_tests test_linecrossing.cpp)
target_include_directories(unit_tests PRIVATE "${CMAKE_SOURCE_DIR}")
target_compile_definitions(unit_tests PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_link_libraries(unit_tests PRIVATE Catch2::Catch2WithMain ${OSMIUM_LIBRARIES} ${Boost_LIBRARIES} SQLite::SQLite3)

if(ENABLE_COVERAGE)
  target_compile_options(unit_tests PRIVATE --coverage -O0)
  target_link_options(unit_tests PRIVATE --coverage)
endif()

if(ENABLE_MULL)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "ENABLE_MULL needs clang, e.g. CXX=clang++-19")
  endif()
  # plugin version must match the clang major version
  string(REGEX MATCH "^[0-9]+" CLANG_MAJOR "${CMAKE_CXX_COMPILER_VERSION}")
  set(MULL_PLUGIN "/usr/lib/mull-ir-frontend-${CLANG_MAJOR}" CACHE FILEPATH "mull clang plugin")
  target_compile_options(unit_tests PRIVATE -O0 -g -grecord-command-line -fpass-plugin=${MULL_PLUGIN})
endif()

add_test(NAME unit_tests COMMAND unit_tests)

set(XML "${CMAKE_CURRENT_SOURCE_DIR}/data/crossings.osm.xml")
add_test(NAME cli_geojson COMMAND linecrossing -i ${XML} -o ${CMAKE_CURRENT_BINARY_DIR}/cli.geojson)
add_test(NAME cli_filters COMMAND linecrossing -i ${XML} -o ${CMAKE_CURRENT_BINARY_DIR}/cli_f.geojson -f highway -x boundary)
add_test(NAME cli_bad_angle COMMAND linecrossing -i ${XML} -a 95)
add_test(NAME cli_bad_parallel COMMAND linecrossing -i ${XML} -a 10 -p 10)
add_test(NAME cli_bad_format COMMAND linecrossing -i ${XML} -o ${CMAKE_CURRENT_BINARY_DIR}/cli.txt)
add_test(NAME cli_no_input COMMAND linecrossing)
set_tests_properties(cli_bad_angle cli_bad_parallel cli_bad_format cli_no_input PROPERTIES WILL_FAIL TRUE)

if(ENABLE_COVERAGE)
  find_program(GCOVR gcovr)
  if(GCOVR)
    add_custom_target(coverage
      COMMAND ${CMAKE_COMMAND} -E env ${CMAKE_CTEST_COMMAND} --output-on-failure
      COMMAND ${GCOVR} -r ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} --filter ".*/linecrossing\\.[ch]pp" --txt --html-details ${CMAKE_BINARY_DIR}/coverage.html
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  endif()
endif()
