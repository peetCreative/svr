cmake_minimum_required( VERSION 3.0 )


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project( SVR )



set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}" )

include( CMake/Bootstrap.cmake )
include( CMake/Dependencies/OGRE.cmake )

setupOgre( OGRE_SOURCE, OGRE_BINARIES, OGRE_LIBRARIES, FALSE )

# Setup our application
set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}" )

macro( add_recursive dir retVal )
	file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

find_package( OpenCV REQUIRED )

include_directories( "./include" )

add_recursive( ./src SOURCES )
add_recursive( ./include HEADERS )

#TODO: make the
# set(OpenVR_DIR /usr/local/include/openvr/)
find_package( OpenVR REQUIRED )
if (NOT OpenVR_FOUND)
	message("no openvr")
    set(OpenVR_INCLUDE_DIRS /opt/include/openvr/)
    set(OpenVR_LIBRARIES /opt/lib/libOpenVR.so)
endif()
	
include_directories( ${OpenVR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} ${SOURCES} ${HEADERS} ${RESOURCES} )
target_link_libraries( ${PROJECT_NAME} ${OGRE_LIBRARIES} ${OpenVR_LIBRARIES} ${OpenCV_LIBS} )
