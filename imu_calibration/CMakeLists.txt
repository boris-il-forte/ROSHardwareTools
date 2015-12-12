cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs)

find_package(Boost REQUIRED COMPONENTS program_options)

project(imu_calibration)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS})

#build the tracking node
file(GLOB_RECURSE IC_SRC 
                  src/*.cpp) 

add_executable(${PROJECT_NAME} ${IC_SRC})
                            
                    
                                 
#link libriaries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
                       
#add dependencies to generated messages
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
