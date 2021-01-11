cmake_minimum_required(VERSION 3.10)

set(NAME KinectFusion)
project(${NAME})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

## OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})
message(STATUS "OpenCV = ${OpenCV_VERSION}")

## Includes / Libs
include_directories(include)
link_directories(lib)

## Project Source/Header files
file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB_RECURSE HEADERS include/*.h)

add_executable(${NAME} ${SOURCES} ${HEADERS})