cmake_minimum_required(VERSION 2.8.3)
project(libcreate)

add_compile_options(-std=c++11 -Wall -Wextra -Werror -Wpedantic -DCMAKE_BUILD_TYPE=Release)

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
  add_executable(${PROJECT_NAME}_${EXAMPLE} examples/${EXAMPLE}.cpp)
  ## Specify libraries to link a library or executable target against
  target_link_libraries(${PROJECT_NAME}_${EXAMPLE}
    ${Boost_LIBRARIES}
    create
  )
  install(TARGETS ${PROJECT_NAME}_${EXAMPLE}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endforeach()

## Install

install(TARGETS create
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# http://wiki.ros.org/catkin/CMakeLists.txt#Installing_header_files
install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

###########
# Testing #
###########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})

  set(TESTS
    create
    data
    packet
    robot_model
    serial_query
    serial_stream
  )

  foreach(TEST ${TESTS})
    add_executable(test_${PROJECT_NAME}_${TEST} tests/test_${TEST}.cpp)
    target_link_libraries(test_${PROJECT_NAME}_${TEST}
      ${Boost_LIBRARIES}
      gtest
      ${catkin_LIBRARIES}
      create
    )
  endforeach()

  add_rostest(tests/${PROJECT_NAME}.test)
endif()

set(ROSLINT_CPP_OPTS "--filter=-build/c++11, -runtime/references, -runtime/arrays")
roslint_cpp()
