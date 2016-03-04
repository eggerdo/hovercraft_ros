cmake_minimum_required(VERSION 2.8.3)
project(hovercraft_node)

find_package(catkin REQUIRED geometry_msgs
			                 message_generation
			                 std_msgs)

catkin_python_setup()

add_message_files(
    DIRECTORY msg
    FILES
        HovercraftCommand.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(DEPENDS std_msgs
					   message_runtime
					   geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS nodes/hovercraft_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
