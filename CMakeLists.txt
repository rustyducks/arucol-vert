cmake_minimum_required(VERSION 3.0.0)
project(arucol-vert VERSION 0.1.0)

find_package(OpenCV REQUIRED)

include(CTest)
enable_testing()

include_directories(include)
add_executable(arucol_vert_calib src/main_calib.cpp)
target_link_libraries(arucol_vert_calib ${OpenCV_LIBS})

add_executable(arucol_vert src/main.cpp src/arucolVert.cpp src/cameraParams.cpp)
target_link_libraries(arucol_vert ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
