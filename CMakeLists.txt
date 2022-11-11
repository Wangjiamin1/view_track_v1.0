cmake_minimum_required(VERSION 2.8)
#add_definitions(-std=c++11)
project(main)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

FILE(GLOB_RECURSE sourcefiles "src/*.cpp")

add_executable(main ${sourcefiles})

target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(main ${PROJECT_SOURCE_DIR}/lib/libViewLink.so)
target_link_libraries(main pthread)
