cmake_minimum_required( VERSION 2.8 )
project( vo )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

find_package( OpenCV REQUIRED )

include_directories( 
    "include/"
    ${OpenCV_INCLUDE_DIRS}
)

add_executable( vo src/vo.cpp  )
target_link_libraries( vo ${OpenCV_LIBS} )
