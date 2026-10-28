cmake_minimum_required (VERSION 3.18.4)
project(linecrossing)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Osmium REQUIRED COMPONENTS io gdal)

find_package(Boost CONFIG 1.70.0 REQUIRED COMPONENTS program_options )
find_package(nlohmann_json 3.11 REQUIRED)
find_package(SQLite3 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(linecrossing linecrossing.cpp)
include_directories("${OSMIUM_INCLUDE_DIRS}")
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(linecrossing ${OSMIUM_LIBRARIES} ${Boost_LIBRARIES} nlohmann_json::nlohmann_json SQLite::SQLite3)

