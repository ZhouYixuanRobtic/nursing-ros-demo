cmake_minimum_required(VERSION 2.8.3)
project(nursing_driver)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs roscpp std_msgs
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(include include/nursing_driver ${catkin_INCLUDE_DIRS})

add_library(SocketCommunicator src/SocketCommunicator.cpp)
target_link_libraries(SocketCommunicator ${catkin_LIBRARIES})

add_library(NursingDriver src/NursingDriver.cpp)
target_link_libraries(NursingDriver ${catkin_LIBRARIES} SocketCommunicator)

add_executable(nursing_driver src/DriverNode.cpp )
target_link_libraries(nursing_driver ${catkin_LIBRARIES} SocketCommunicator NursingDriver)

add_executable(server src/server.cpp)
target_link_libraries(server ${catkin_LIBRARIES} SocketCommunicator)

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES} SocketCommunicator)

add_executable(nursing_gazebo_driver src/nursing_gazebo_driver.cpp)
target_link_libraries(nursing_gazebo_driver ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )


