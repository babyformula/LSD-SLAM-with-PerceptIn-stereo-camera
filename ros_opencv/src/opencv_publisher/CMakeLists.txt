cmake_minimum_required(VERSION 2.8.3)
project(opencv_test)

set(CMAKE_C_FLAGS "-std=c99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -std=c++11")
set(CMAKE_BUILD_TYPE Release)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

## Find catkin macros and libraries

find_package(catkin  REQUIRED COMPONENTS cv_bridge image_transport  roscpp sensor_msgs std_msgs)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(PkgConfig)

## Declare a catkin package
catkin_package()

## Include directories,
## Add target and link libraries

include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
pkg_check_modules(LIBUSB libusb-1.0)

set(EXT_LIBS
    ${OpenCV_LIBS}
    ${LIBUSB_LIBRARIES}
    -lpthread
    -lblas
    -llapack
    )

# Set library and headers.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/cv_bridge)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_executable(opencv_test_node src/opencv_test_node.cpp)
target_link_libraries(opencv_test_node -lPerceptInPIRVS ${EXT_LIBS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(opencv_test_node opencv_test_gencpp)
