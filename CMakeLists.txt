# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
project(locomotion_control_lib)
cmake_minimum_required(VERSION 2.6)
find_package(Rock)

add_definitions(-DWall)
add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
add_definitions(-DBASE_LOG_INFO)

rock_init(locomotion_control 0.1)
rock_standard_layout()
