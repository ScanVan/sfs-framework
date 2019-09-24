cmake_minimum_required(VERSION 2.8.0)
project( cmakeTemplate  CXX)
set(CMAKE_CXX_FLAGS "-std=c++11") 

find_package( OpenCV 4 REQUIRED )
find_package( Eigen3 REQUIRED )

set(ROOT ./)
include_directories(${ROOT}/src)
include_directories(${ROOT}/third)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

file(GLOB cmakeTemplate_SRC
    "src/*.?pp"
)

add_executable( cmakeTemplate ${cmakeTemplate_SRC} )
target_link_libraries( cmakeTemplate ${OpenCV_LIBS}) 

add_custom_target(run
    COMMAND bin/cmakeTemplate
    DEPENDS cmakeTemplate
)

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")

