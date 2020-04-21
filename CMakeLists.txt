cmake_minimum_required( VERSION 3.5 )


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
link_libraries(stdc++fs)
project( surgical_vr )

include(CMakeDependentOption)
option( BUILD_ROS "compile ROS Nodelet" ON )
option(BUILD_EXECUTABLE "Build an executable" OFF)
# CMAKE_DEPENDENT_OPTION(BUILD_EXECUTABLE "Build an executable" OFF
#                        "NOT BUILD_ROS" ON)
find_package( OpenCV REQUIRED )
# find_package( libconfig++ REQUIRED)

if (BUILD_ROS)
    MESSAGE("build ROS")
    find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        sensor_msgs
        message_filters
        pluginlib)
    catkin_package()
    set( CMAKE_CURRENT_BIN_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}" )
elseif (BUILD_EXECUTABLE)
    MESSAGE("build executable")
    set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}" )
    set( CMAKE_CURRENT_BIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin" )
endif()
MESSAGE( "${CATKIN_DEVEL_PREFIX}")
MESSAGE( "${CATKIN_ENV}")
MESSAGE( "${CATKIN_PACKAGE_LIB_DESTINATION}")
MESSAGE( "${EXECUTABLE_OUTPUT_DIRECTORY}")
MESSAGE( "${CMAKE_CURRENT_BIN_DIR}")
add_definitions(-DRESOURCE_PATH="${CMAKE_CURRENT_BIN_DIR}/Data/")
add_definitions(-DPLUGINS_FOLDER="${CMAKE_CURRENT_BIN_DIR}/${CMAKE_BUILD_TYPE}/")

include( CMake/Bootstrap.cmake )
include( CMake/Dependencies/OGRE.cmake )

setupOgre( OGRE_SOURCE, OGRE_BINARIES, OGRE_LIBRARIES, FALSE  )

macro( add_dir_recursive dir retVal)
	file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()
macro( add_dir dir retVal)
	file( GLOB ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

include_directories( "./include" )

add_dir_recursive( src/OgreCommon SOURCES_OGRE)
add_dir_recursive( include/OgreCommon HEADERS_OGRE)
add_dir( src SOURCES_BASE)
add_dir( include HEADERS_BASE)

#TODO: make the
# set(OpenVR_DIR /usr/local/include/openvr/)
find_package( OpenVR REQUIRED )
if (NOT OpenVR_FOUND)
	message("no openvr")
	set(OpenVR_INCLUDE_DIRS /opt/include/openvr/)
	set(OpenVR_LIBRARIES /opt/lib/libOpenVR.so)
endif()


include_directories(${OpenVR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

if ( BUILD_ROS )
    include_directories("include/OgreCommon")
    add_definitions(-DUSE_ROS)
    include_directories("${catkin_INCLUDE_DIRS}")
    add_executable( ROSSVRNode
        ${SOURCES_OGRE} ${SOURCES_ROS} ${SOURCES_BASE})
    target_link_libraries( ROSSVRNode
        ${catkin_LIBRARIES} ${OGRE_LIBRARIES}
        ${OpenVR_LIBRARIES} ${OpenCV_LIBS} config++)
endif()

if ( BUILD_EXECUTABLE )
    add_executable( ${PROJECT_NAME}_bin
        ${SOURCES_OGRE} ${SOURCES_BASE}
        ${HEADERS_BASE} ${HEADERS_OGRE}
        ${RESOURCES} )
    target_link_libraries( ${PROJECT_NAME}_bin
        ${OGRE_LIBRARIES} ${OpenVR_LIBRARIES} ${OpenCV_LIBS} config++ )
endif()
