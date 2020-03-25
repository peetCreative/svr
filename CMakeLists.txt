cmake_minimum_required( VERSION 3.5 )


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
link_libraries(stdc++fs)
project( SVR )

option( BUILD_ROS "compile ROS Nodelet" false )
option( BUILD_EXECUTABLE "compile Executable" true )

if (BUILD_ROS)
	find_package(catkin REQUIRED COMPONENTS
		cv_bridge
		roscpp
		sensor_msgs)
endif()

find_package( OpenCV REQUIRED )

set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}" )

include( CMake/Bootstrap.cmake )
include( CMake/Dependencies/OGRE.cmake )

setupOgre( OGRE_SOURCE, OGRE_BINARIES, OGRE_LIBRARIES, FALSE )

# Setup our application
set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}" )

macro( add_dir_recursive dir retVal)
	file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()
macro( add_dir dir retVal)
	file( GLOB ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

include_directories( "./include" )


add_dir_recursive( ./src/OgreCommon SOURCES_OGRE)
add_dir_recursive( ./include/OgreCommon HEADERS_OGRE)
add_dir( ./src SOURCES_BASE)
add_dir( ./include HEADERS_BASE)
if ( BUILD_ROS )
	add_dir_recursive( ./src/ros SOURCES_ROS)
	add_dir_recursive( ./include/ros HEADERS_ROS)
endif()

#TODO: make the
# set(OpenVR_DIR /usr/local/include/openvr/)
find_package( OpenVR REQUIRED )
if (NOT OpenVR_FOUND)
	message("no openvr")
	set(OpenVR_INCLUDE_DIRS /opt/include/openvr/)
	set(OpenVR_LIBRARIES /opt/lib/libOpenVR.so)
endif()


include_directories(
	${OpenVR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

if ( BUILD_ROS )
	catkin_package()
	include_directories(
		${HEADERS_OGRE} ${HEADERS_ROS} ${HEADERS_BASE}
		${catkin_INCLUDE_DIRS})
	add_library( ${PROJECT_NAME}
		${SOURCES_OGRE} ${HEADERS_ROS} ${SOURCES_BASE}
		${OGRE_LIBRARIES} ${OpenVR_LIBRARIES} ${OpenCV_LIBS})
endif()

if ( BUILD_EXECUTABLE )
	add_executable( ${PROJECT_NAME}
		${SOURCES_BASE} ${SOURCES_OGRE} ${SOURCE_ROS}
		${HEADERS_BASE} ${HEADERS_OGRE} ${HEADERS_ROS}
		${RESOURCES} )
	target_link_libraries( ${PROJECT_NAME}
		${OGRE_LIBRARIES} ${OpenVR_LIBRARIES} ${OpenCV_LIBS} )
endif()
