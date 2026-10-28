# Locate the nlohmann-json header-only library.
#
#  NLOHMANN_JSON_FOUND
#  NLOHMANN_JSON_INCLUDE_DIRS
#
# The library is looked for as a checkout in the project directory or next
# to it first, then in the system installation.

find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
    HINTS
        ${CMAKE_SOURCE_DIR}/nlohmann-json
        ${CMAKE_SOURCE_DIR}/../nlohmann-json
        ${CMAKE_SOURCE_DIR}/json
        ${CMAKE_SOURCE_DIR}/../json
    PATH_SUFFIXES include single_include)

set(NLOHMANN_JSON_INCLUDE_DIRS "${NLOHMANN_JSON_INCLUDE_DIR}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NlohmannJSON DEFAULT_MSG NLOHMANN_JSON_INCLUDE_DIR)

mark_as_advanced(NLOHMANN_JSON_INCLUDE_DIR)
