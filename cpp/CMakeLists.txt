CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type selected, default to Debug")
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type (default Debug)" FORCE)
endif()
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
PROJECT(cpp)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

set(Boost_USE_STATIC_LIBS        ON) # only find static libs
find_package(Boost COMPONENTS filesystem regex)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)



include_directories(${Boost_INCLUDE_DIRS}
    /opt/ros/kinetic/include/opencv-3.3.1-dev
    ${EIGEN3_INCLUDE_DIR}
    include
    /usr/include/python2.7
)

add_executable(runner src/main.cpp src/icp.cpp src/cv_utils.cpp src/plot.cpp)
target_link_libraries(runner ${Boost_LIBRARIES} ${OpenCV_LIBS} python2.7)
