cmake_minimum_required(VERSION 2.8.3)
project(libcreate)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED system thread)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES create
)

## Specify additional locations of header files
include_directories(
  include
)

## Declare cpp library
add_library(create
  src/create.cpp
  src/serial.cpp
  src/serial_stream.cpp
  src/serial_query.cpp
  src/data.cpp
  src/packet.cpp
  src/types.cpp
  src/encoder.cpp
  src/odometry.cpp
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
endforeach(EXAMPLE)

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
