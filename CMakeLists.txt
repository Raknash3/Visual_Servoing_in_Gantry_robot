project(final)
cmake_minimum_required(VERSION 3.0)
find_package(VISP REQUIRED)
include_directories(${VISP_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_executable(main1 main.cpp)
target_link_libraries(main1 ${VISP_LIBRARIES})


add_executable(main2 main-3.cpp)
target_link_libraries(main2 ${VISP_LIBRARIES})

add_executable(main1-without-IOU main1-without_IOU.cpp)
target_link_libraries(main1-without-IOU ${VISP_LIBRARIES})





