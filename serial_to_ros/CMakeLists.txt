cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        hardware_tools_msgs)

find_package(Boost REQUIRED COMPONENTS program_options)

project(serial_to_ros)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS})

#build the tracking node
file(GLOB_RECURSE S2R_SRC 
                  src/*.cpp) 

add_executable(${PROJECT_NAME} ${S2R_SRC})
                            
                    
                                 
#link libriaries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
                       
#add dependencies to generated messages
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
