cmake_minimum_required(VERSION 2.8.3)
project(roboy_controller)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp controller_interface hardware_interface pluginlib
        roboy_communication_middleware
		ecl_geometry
		aruco_detect
		sensor_msgs
		cv_bridge
		std_msgs
		image_transport
		common_utilities
		controller_manager
		roboy_simulation)
catkin_package(CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities roboy_simulation INCLUDE_DIRS include)

# roboticslibrary
find_package(rl REQUIRED)
include_directories(${RL_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
message(STATUS ${GAZEBO_INCLUDE_DIRS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#==============================================================================
# Libraries
#==============================================================================
add_library(PositionController src/PositionController.cpp)
add_dependencies(PositionController
	roboy_communication_middleware_generate_messages_cpp
)
target_link_libraries(PositionController ${catkin_LIBRARIES})

add_library(VelocityController src/VelocityController.cpp)
add_dependencies(VelocityController
	roboy_communication_middleware_generate_messages_cpp
)
target_link_libraries(VelocityController ${catkin_LIBRARIES})

add_library(ForceController src/ForceController.cpp)
add_dependencies(ForceController
	roboy_communication_middleware_generate_messages_cpp
)
target_link_libraries(ForceController ${catkin_LIBRARIES})

add_library(MuscleActivityController src/MuscleActivityController.cpp)
add_dependencies(MuscleActivityController
	roboy_communication_middleware_generate_messages_cpp
)
target_link_libraries(MuscleActivityController ${catkin_LIBRARIES})

add_library(JointAngleController src/JointAngleController.cpp)
add_dependencies(JointAngleController
		roboy_communication_middleware_generate_messages_cpp
		)
target_link_libraries(JointAngleController ${catkin_LIBRARIES} ${librealsense_LIBRARIES} )

add_library(MuscController src/MuscController.cpp)
add_dependencies(MuscController
		roboy_communication_middleware_generate_messages_cpp
		)
target_link_libraries(MuscController ${catkin_LIBRARIES})

add_executable(roboy src/roboy.cpp include/roboy_controller/roboy.hpp)
add_dependencies(roboy roboy_communication_middleware_generate_messages_cpp )
target_link_libraries(roboy ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(HitDetection  src/HitDetectionPlugin.cpp)
add_dependencies(roboy roboy_communication_middleware_generate_messages_cpp )
target_link_libraries(HitDetection ${GAZEBO_libraries}  ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

add_library(CupTablePlugin  src/CupTablePlugin.cpp)
add_dependencies(CupTablePlugin roboy_communication_middleware_generate_messages_cpp )
target_link_libraries(CupTablePlugin ${GAZEBO_libraries}  ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        PositionController
        VelocityController
        ForceController
        MuscleActivityController
        MuscController
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        PositionController_plugin.xml
        VelocityController_plugin.xml
        ForceController_plugin.xml
        MuscleActivityController_plugin.xml
        MuscController_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY
        config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )
