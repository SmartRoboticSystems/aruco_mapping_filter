cmake_minimum_required(VERSION 2.8.3)
project(aruco_mapping_filter)

find_package(catkin REQUIRED COMPONENTS 
             roscpp
             std_msgs 
             message_generation
             image_transport
             cv_bridge
             )

find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)

SET(SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp
            ${PROJECT_SOURCE_DIR}/src/filter.cpp)   
   
SET(HEADERS ${PROJECT_SOURCE_DIR}/include/filter.h)

catkin_package()

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})
