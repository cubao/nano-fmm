cmake_minimum_required(VERSION 3.4...3.18)
project(nano_fmm)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
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

include_directories(3rdparty src)

set(PYBIND11_CPP_STANDARD -std=c++17)
add_subdirectory(pybind11)
file(GLOB_RECURSE SRCS src/*.cpp)
pybind11_add_module(_nano_fmm ${SRCS})

target_compile_definitions(_nano_fmm
                           PRIVATE VERSION_INFO=${NANO_FMM_VERSION_INFO})
