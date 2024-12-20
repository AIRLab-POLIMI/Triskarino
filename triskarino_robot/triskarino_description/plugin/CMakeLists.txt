cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(omniwheel_control)

find_package(roscpp REQUIRED)
find_package(tf REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${tf_INCLUDE_DIRS})
include_directories(${nav_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${geoemtry_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


# Build our plugin
add_library(omniwheel_control SHARED main.cc)
target_link_libraries(omniwheel_control ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${tf_LIBRARIES} )
target_compile_features(omniwheel_control PRIVATE cxx_std_17)

if (${gazebo_VERSION_MAJOR} LESS 6)
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(omniwheel_control ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(omniwheel_control ${GAZEBO_LIBRARIES})
endif()