cmake_minimum_required(VERSION 2.8)
project( odometry )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
add_library(${PROJECT_NAME} SHARED
        visual_odometry.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(test1 test.cpp)
target_link_libraries(test1 ${PROJECT_NAME} ${OpenCV_LIBS})
                                                             
