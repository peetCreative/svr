
set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG" )

if( CMAKE_BUILD_TYPE STREQUAL "" )
        set( CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "One of: Debug Release RelWithDebInfo MinSizeRel." FORCE )
endif()

set( CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG=1 -D_DEBUG=1 -g")
set( CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -Os -DNDEBUG")
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")
set( CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -g" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-overloaded-virtual" )

if( CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
    set( CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -fno-limit-debug-info" )
endif()

# Determine if we are compiling for a 32bit or 64bit system
include(CheckTypeSize)
CHECK_TYPE_SIZE("void*" ARCH_PTR_SIZE BUILTIN_TYPES_ONLY)
if (ARCH_PTR_SIZE EQUAL 8)
    set(PLATFORM_X64 TRUE)
else ()
    set(PLATFORM_X64 FALSE)
endif ()

macro( add_recursive dir retVal )
    file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()