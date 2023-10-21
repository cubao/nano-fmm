cmake_minimum_required(VERSION 3.5...3.18)
project(nano_fmm)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
endif()
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
endif()
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
endif()

# set(CMAKE_BUILD_TYPE "Debug")

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "" FORCE)
  message(STATUS "Set build type to default: ${CMAKE_BUILD_TYPE}")
else()
  message(STATUS "Your build type: ${CMAKE_BUILD_TYPE}")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -ggdb")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -ggdb")
  add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

include_directories(3rdparty src)

option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
  file(GLOB SRCS examples/*.cpp)
  foreach(src ${SRCS})
    string(REGEX REPLACE "(^.*/|.cpp$)" "" exe ${src})
    add_executable(${exe} ${src})
    install(TARGETS ${exe} RUNTIME DESTINATION bin)
  endforeach(src)
endif()

set(PYBIND11_CPP_STANDARD -std=c++17)
add_subdirectory(pybind11)
file(GLOB_RECURSE SRCS src/*.cpp)
pybind11_add_module(_nano_fmm ${SRCS})

target_compile_definitions(_nano_fmm
                           PRIVATE VERSION_INFO=${NANO_FMM_VERSION_INFO})
