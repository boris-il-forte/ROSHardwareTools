cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs)

project(signal_smoother)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS})

#build the node
file(GLOB_RECURSE SM_SRC 
                  src/*.cpp) 

add_executable(${PROJECT_NAME} ${SM_SRC})
                            
#Install executables
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)                    
                                 
#link libriaries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
                       
#add dependencies to generated messages
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
