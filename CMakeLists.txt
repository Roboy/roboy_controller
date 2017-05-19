cmake_minimum_required(VERSION 2.8.3)
project(roboy_controller)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp controller_interface hardware_interface pluginlib
        roboy_communication_middleware)
catkin_package(CATKIN_DEPENDS message_runtime roboy_communication_middleware)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#==============================================================================
# Libraries
#==============================================================================
add_library(PositionController src/PositionController.cpp)
add_dependencies(PositionController roboy_communication_generate_messages_cpp)
target_link_libraries(PositionController ${catkin_LIBRARIES})

add_library(VelocityController src/VelocityController.cpp)
add_dependencies(VelocityController roboy_communication_generate_messages_cpp)
target_link_libraries(VelocityController ${catkin_LIBRARIES})

add_library(ForceController src/ForceController.cpp)
add_dependencies(ForceController roboy_communication_generate_messages_cpp)
target_link_libraries(ForceController ${catkin_LIBRARIES})

add_library(MuscleActivityController src/MuscleActivityController.cpp)
add_dependencies(MuscleActivityController roboy_communication_generate_messages_cpp)
target_link_libraries(MuscleActivityController ${catkin_LIBRARIES})
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        PositionController
        VelocityController
        ForceController
        MuscleActivityController
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        PositionController_plugin.xml
        VelocityController_plugin.xml
        ForceController_plugin.xml
        MuscleActivityController_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY
        config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )
