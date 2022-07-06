cmake_minimum_required(VERSION 3.4)
project(vkr_sim)

## Compile as C++11
set(CMAKE_CXX_STANDARD 11)

add_subdirectory(submodules/kdl_parser/kdl_parser)

# Debug option
ADD_DEFINITIONS(-DDEBUG)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)

message("message ${orocos_kdl_INCLUDE_DIRS}")

########### CppUnit and tests ###########
find_package(CppUnit)
IF(CppUnit_FOUND)
	message(STATUS "SUCCESSFUL: CPP_UNIT found")
	add_subdirectory(src/testing)
ELSE(CppUnit_FOUND)
#	MESSAGE(STATUS "WARNING: CPP_UNIT not found")
ENDIF (CppUnit_FOUND)

add_subdirectory(submodules/youbot_driver)

include_directories(
    include/
    ${Eigen3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    submodules/kdl_parser/kdl_parser/include
    submodules/youbot_driver/include
)

# Library with kinematic
add_library(robot_model SHARED
    src/robot_model.cpp
)
target_link_libraries(robot_model Eigen3::Eigen ${orocos_kdl_LIBRARIES} kdl_parser)

# Library with controllers
add_library(robot_controllers SHARED
    src/pid.cpp
    src/servo.cpp
)

target_link_libraries(robot_controllers robot_model)


add_library(robots
  src/youbot_arm.cpp
)

target_link_libraries(robots YouBotDriver)


#Add executables

add_executable(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic robot_model)

add_executable(test_controllers test/test_controllers.cpp)
target_link_libraries(test_controllers robot_controllers)

add_executable(test_servo test/test_servo.cpp)
target_link_libraries(test_servo robot_controllers)

add_executable(test_robot test/test_robot.cpp)
target_link_libraries(test_robot robot_controllers robots)