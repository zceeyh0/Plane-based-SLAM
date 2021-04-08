INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

AUX_SOURCE_DIRECTORY(./ DIR_SRCS)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( PCL REQUIRED )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # used in Ubuntu 16.04

find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )

set( DBoW3_INCLUDE_DIRS "~/Downloads/VO/DBow3/src" )
set( DBoW3_LIBS "~/Downloads/VO/DBow3/build/src/libDBoW3.a" )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)

LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

add_executable( main main.cpp slamFunc.cpp icp.cpp pnp.cpp )
target_link_libraries( main
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension 
    ${CSPARSE_LIBRARY}
    ${DBoW3_LIBS} )
    
add_executable( vocab vocab.cpp )
target_link_libraries( vocab
    ${OpenCV_LIBS} 
    ${DBoW3_LIBS} )

