# #This scripts will add all the cpp and h files under src and include folders, and
# #assumes that your Ogre source code is in Dependencies/Ogre and that:
# # In Windows you built Ogre into Dependencies/Ogre/build
# # In Linux you built Release into Dependencies/Ogre/build/Release
# # In Linux you built Debug into Dependencies/Ogre/build/Debug
# #
# # If your source code is not at "Dependencies/Ogre"; you can use "mklink /D" to create
# # a symbolic link to where the source code is located on Windows.
# # On Linux, you can use "ln -s"
# 
# #set( CMAKE_TOOLCHAIN_FILE CMake/iOS.cmake )
# 
# cmake_minimum_required( VERSION 3.0 )
# 
# 
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# project( SurgicalVR )
# 
# 
# 
# set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}" )
# 
# include( CMake/Bootstrap.cmake )
# include( CMake/Dependencies/OGRE.cmake )
# 
# setupOgre( OGRE_SOURCE, OGRE_BINARIES, OGRE_LIBRARIES, FALSE )
# 
# # Setup our application
# set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}" )
# if( MSVC )
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
# 	if( NOT PLATFORM_X64 )
# 		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")
# 	endif()
# 	add_definitions( -DUNICODE -D_UNICODE )
# endif()
# 
# if( APPLE )
# 	macro( add_recursive dir retVal )
# 		file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c ${dir}/*.mm ${dir}/*.m )
# 	endmacro()
# else()
# 	macro( add_recursive dir retVal )
# 		file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
# 	endmacro()
# endif()
# 
# find_package( OpenCV REQUIRED )
# 
# include_directories( "./include" )
# 
# add_recursive( ./src SOURCES )
# add_recursive( ./include HEADERS )
# 
# if( APPLE )
# 	file( GLOB_RECURSE RESOURCES ./src/*.storyboard )
# 	set( RESOURCES ${RESOURCES} ./Data/Resources.cfg ./bin/Data )
# endif()
# 
# set(OpenVR_FOUND TRUE)
# set(OpenVR_INCLUDE_DIRS /usr/local/include/openvr/)
# set(OpenVR_LIBRARIES /usr/local/lib/libopenvr_api.so)
# 
# include_directories( ${OpenVR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# 
# add_executable( ${PROJECT_NAME} WIN32 MACOSX_BUNDLE ${SOURCES} ${HEADERS} ${RESOURCES} )
# target_link_libraries( ${PROJECT_NAME} ${OGRE_LIBRARIES} ${OpenVR_LIBRARIES} ${OpenCV_LIBS} )
# 
# if( APPLE )
# 	set_target_properties( ${PROJECT_NAME} PROPERTIES XCODE_ATTRIBUTE_ENABLE_BITCODE "NO" )
# 	set_target_properties( ${PROJECT_NAME} PROPERTIES RESOURCE "${RESOURCES}" )
# 	set_target_properties( ${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/src/Info.plist )
# 	#set_target_properties( ${PROJECT_NAME} PROPERTIES MACOSX_BUNDLE_ICON_FILE SampleBrowser_OSX.icns)
# 	set( CMAKE_EXE_LINKER_FLAGS "-framework Foundation -framework CoreGraphics -framework QuartzCore -framework UIKit -framework Metal -framework MetalKit -framework ModelIO" )
# endif()
#This scripts will add all the cpp and h files under src and include folders, and
#assumes that your Ogre source code is in Dependencies/Ogre and that:
# In Windows you built Ogre into Dependencies/Ogre/build
# In Linux you built Release into Dependencies/Ogre/build/Release
# In Linux you built Debug into Dependencies/Ogre/build/Debug
#
# If your source code is not at "Dependencies/Ogre"; you can use "mklink /D" to create
# a symbolic link to where the source code is located on Windows.
# On Linux, you can use "ln -s"

#set( CMAKE_TOOLCHAIN_FILE CMake/iOS.cmake )

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
# find_package( OpenVR REQUIRED )
# if (NOT OpenVR_FOUND)
# 	message("no openvr")
set(OpenVR_INCLUDE_DIRS /usr/local/include/openvr/)
set(OpenVR_LIBRARIES /usr/local/lib/libopenvr_api.so)
# endif()
	
include_directories( ${OpenVR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} ${SOURCES} ${HEADERS} ${RESOURCES} )
target_link_libraries( ${PROJECT_NAME} ${OGRE_LIBRARIES} ${OpenVR_LIBRARIES} ${OpenCV_LIBS} )
