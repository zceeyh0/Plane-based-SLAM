cmake_minimum_required( VERSION 2.8 )
project( VO )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src )

