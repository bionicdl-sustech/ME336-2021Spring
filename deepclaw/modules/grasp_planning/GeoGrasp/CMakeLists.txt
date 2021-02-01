cmake_minimum_required(VERSION 3.0)
project(geograsp)

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-I/usr/include/python3.6m -lpython3.6m")

set(CMAKE_PREFIX_PATH "/usr/lib/python3.6")
set(PYTHON_EXECUTABLE /usr/bin/python3.6)
set(PYTHON_LIBRARY /usr/lib/x86_64-linux-gnu/libpython3.6m.so)
#set(PYTHON_INCLUDE_DIRS "/usr/include/python3.6m")


#find_package(VTK 7.1 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11)

include_directories( ${PCL_INCLUDE_DIRS})
include_directories( ${Eigen3_INCLUDE_DIRS})
#include(${VTK_USE_FILE})
#include_directories('/usr/include/python3.6m')

link_libraries(${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

pybind11_add_module(GeoGrasp cloud_processor.cpp GeoGrasp.cpp GeoGrasp.h)
#add_executable(GeoGraspX cloud_processor.cpp GeoGrasp.cpp)



