cmake_minimum_required(VERSION 2.8.0)
project( sfs-framework  CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread")

find_package( OpenCV 4 REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( yaml-cpp REQUIRED )

set(ROOT ./)
include_directories(${ROOT}/src)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${YAML_CPP_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(COMMON_FILES       
	"src/framework-frontend.cpp"       
	"src/framework-structure.cpp"  
	"src/framework-viewpoint.cpp"
	"src/framework-database.cpp"  
	"src/framework-sparsefeature.cpp"  
	"src/framework-transform.cpp"  
	"src/framework-viewpointsource.cpp"
	"src/framework-feature.cpp"   
	"src/framework-stillcompute.cpp"   
	"src/framework-utiles.cpp"
)

# Odometry
file(GLOB sfs-framework_SRC
    ${COMMON_FILES}
	"src/framework.cpp"       
)

# Densification
file(GLOB sfs-framework_dense_SRC
    ${COMMON_FILES}
	"src/framework-dense.cpp"       
)

add_executable( sfs-framework ${sfs-framework_SRC} )
target_link_libraries( sfs-framework yaml-cpp ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} stdc++fs)


add_executable( sfs-framework_dense ${sfs-framework_dense_SRC} )
target_link_libraries( sfs-framework_dense yaml-cpp ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} stdc++fs)
set_target_properties( sfs-framework_dense PROPERTIES COMPILE_FLAGS "-DMIAOU" )

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")

