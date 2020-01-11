cmake_minimum_required(VERSION 2.8.3)
project(libcreate)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roslint
)

find_package(Boost REQUIRED system thread)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include tests
  LIBRARIES create
)

## Specify additional locations of header files
include_directories(
  include tests ${catkin_INCLUDE_DIRS}
)

## Declare cpp library
add_library(create
  src/create.cpp
  src/data.cpp
  src/packet.cpp
  src/serial.cpp
  src/serial_query.cpp
  src/serial_stream.cpp
  src/types.cpp
)

if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET create PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET create PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()

if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(create "${CMAKE_THREAD_LIBS_INIT}")
endif()

target_link_libraries(create
  ${Boost_LIBRARIES}
)

## Declare example executables
set(EXAMPLES
  bumper_example
  create_demo
  odom_example
)
foreach(EXAMPLE ${EXAMPLES})
  add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
endforeach()

## Specify libraries to link a library or executable target against
target_link_libraries(create_demo
  ${Boost_LIBRARIES}
  create
)
target_link_libraries(bumper_example
  ${Boost_LIBRARIES}
  create
)
target_link_libraries(odom_example
  ${Boost_LIBRARIES}
  create
)

## Install

install(TARGETS create
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/create/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS bumper_example create_demo odom_example
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

###########
# Testing #
###########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
  
  add_executable(test_create tests/test_create.cpp)
  add_executable(test_data tests/test_data.cpp)
  add_executable(test_packet tests/test_packet.cpp)
  add_executable(test_robot_model tests/test_robot_model.cpp)
  add_executable(test_serial_query tests/test_serial_query.cpp)
  add_executable(test_serial_stream tests/test_serial_stream.cpp)
  
  target_link_libraries(test_create ${Boost_LIBRARIES} gtest ${catkin_LIBRARIES} create)
  target_link_libraries(test_data ${Boost_LIBRARIES} gtest ${catkin_LIBRARIES} create)
  target_link_libraries(test_packet ${Boost_LIBRARIES} gtest ${catkin_LIBRARIES} create)
  target_link_libraries(test_robot_model ${Boost_LIBRARIES} gtest ${catkin_LIBRARIES} create)
  target_link_libraries(test_serial_query ${Boost_LIBRARIES} gtest ${catkin_LIBRARIES} create)
  target_link_libraries(test_serial_stream ${Boost_LIBRARIES} gtest ${catkin_LIBRARIES} create)

  add_rostest(tests/${PROJECT_NAME}.test)
endif()

set(ROSLINT_CPP_OPTS "--filter=-build/c++11, -runtime/references, -runtime/arrays")
roslint_cpp()
