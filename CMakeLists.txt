# cmake_minimum_required(VERSION 3.5)
# project(hello)

# # # Default to C99
# # if(NOT CMAKE_C_STANDARD)
# #   set(CMAKE_C_STANDARD 99)
# # endif()

# # # Default to C++14
# # if(NOT CMAKE_CXX_STANDARD)
# #   set(CMAKE_CXX_STANDARD 14)
# # endif()

# # if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# #   add_compile_options(-Wall -Wextra -Wpedantic)
# # endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED )


# # add_executable(my_node src/my_node.cpp)
# # target_include_directories(my_node PUBLIC
# #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# #   $<INSTALL_INTERFACE:include>)

# include_directories(
#   include
# )

# include_directories(${CERES_INCLUDE_DIRS})

# # set(library_name hello)

# set(dependencies 
#   rclcpp
#   tf2
#   geometry_msgs
#   nav_msgs
#   Ceres
# )

# # add_library(${library_name} 
# # )

# # target_include_directories(${library_name} PUBLIC ${Eigen3_INCLUDE_DIRS})

# # ament_target_dependencies(${library_name}
# #   ${dependencies}
# #   Eigen3
# # )

# # target_link_libraries(${library_name}
# #   ${CERES_LIBRARIES}
# #   )

# #node test
# # add_executable(turtlebot_controller src/pid_one_turtle.cpp)
# add_executable(mpc_controller src/mpc_controller.cpp)
# target_link_libraries(mpc_controller 
#   ${CERES_LIBRARIES}
#   )
# target_include_directories(mpc_controller PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# # Link dependencies
# ament_target_dependencies(mpc_controller ${dependencies})

# # Install executable
# install(TARGETS mpc_controller
#   DESTINATION lib/${PROJECT_NAME}
# )

# # add_executable(mpc_turtle_p2p_velo src/mpc_ceres_ros.cpp)
# # # target_link_libraries(mpc_turtle_p2p_velo
# # #     ${library_name}
# # #     )
# # target_include_directories(mpc_turtle_p2p_velo PUBLIC 
# #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# #   $<INSTALL_INTERFACE:include>)

# #   target_link_libraries(mpc_turtle_p2p_velo
# #     ${CERES_LIBRARIES}
# # )

# # # Link dependencies
# # ament_target_dependencies(mpc_turtle_p2p_velo ${dependencies})

# # # Install executable
# # install(TARGETS mpc_turtle_p2p_velo
# #   DESTINATION lib/${PROJECT_NAME}
# # )


# #end node test


# # Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# # install(TARGETS ${library_name}
# #   ARCHIVE DESTINATION lib
# #   LIBRARY DESTINATION lib
# #   RUNTIME DESTINATION bin
# # )

# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )



# # Export package information
# # ament_export_dependencies(
# #   rclcpp
# #   geometry_msgs
# #   nav_msgs)  # Add this line

# # install(TARGETS my_node
# #   DESTINATION lib/${PROJECT_NAME})

# # if(BUILD_TESTING)
# #   find_package(ament_lint_auto REQUIRED)
# #   # the following line skips the linter which checks for copyrights
# #   # uncomment the line when a copyright and license is not present in all source files
# #   #set(ament_cmake_copyright_FOUND TRUE)
# #   # the following line skips cpplint (only works in a git repo)
# #   # uncomment the line when this package is not in a git repo
# #   #set(ament_cmake_cpplint_FOUND TRUE)
# #   ament_lint_auto_find_test_dependencies()
# # endif()
# ament_export_include_directories(include)
# ament_export_dependencies(${dependencies})
# ament_package()

#---------------------------------------------------------------------------------------

# cmake_minimum_required(VERSION 3.5)
# project(hello)

# find_package(ament_cmake REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
# find_package(tf2 REQUIRED)

# # nav2_package()

# include_directories(
#   include
# )

# # set(library_name rostek_followline)

# set(dependencies 
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   Ceres
#   tf2
# )

# # add_library(${library_name} SHARED
# #   src/mpc_ceres.cpp
# #   src/mpc_ceres_ros.cpp
# #   src/utils/ostacle_detect.cpp
# #   src/utils/polygon_func.cpp
# #   src/utils/rotate_control.cpp
# #   src/utils/calculate_function.cpp
# # )
# target_include_directories( PUBLIC ${Eigen3_INCLUDE_DIRS})
# ament_target_dependencies(
#   ${dependencies}
#   Eigen3
# )
# target_link_libraries(
#   ${CERES_LIBRARIES}
 
#   )
# #node test
# add_executable(mpc_controller src/mpc_controller.cpp)
# # target_link_libraries(mpc_controller
# #  ${CERES_LIBRARIES}
# #     )
# ament_target_dependencies(mpc_controller ${dependencies} Ceres)
# install(TARGETS
# mpc_controller
#   DESTINATION lib/${PROJECT_NAME})
# #end node test

# #callback param
# # add_executable(param_clbk gtest/param_clbk.cpp)
# # target_link_libraries(param_clbk
# #     ${library_name}
# #     )
# # ament_target_dependencies(param_clbk ${dependencies})
# # install(TARGETS
# # param_clbk
# #   DESTINATION lib/${PROJECT_NAME})
# # #end

# # install(TARGETS ${library_name}
# #   ARCHIVE DESTINATION lib
# #   LIBRARY DESTINATION lib
# #   RUNTIME DESTINATION bin
# # )

# install(DIRECTORY include/
#   DESTINATION include/
# )

# # install(FILES plugin.xml
# #   DESTINATION share/${PROJECT_NAME}
# # )

# # target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# # pluginlib_export_plugin_description_file(nav2_core plugin.xml)

# if(BUILD_TESTING)
#   # find_package(ament_cmake_gtest REQUIRED)
#   # set(fitting_test fitting_fnc_test)
#   # ament_add_gtest(${fitting_test} gtest/utils/polygon_func_test.cc)
#   # target_include_directories(${fitting_test} PUBLIC
#   #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   #   $<INSTALL_INTERFACE:include>
#   # )
#   # ament_target_dependencies(${fitting_test}
#   #   nav_msgs geometry_msgs std_msgs rclcpp
#   # )
#   # target_link_libraries(${fitting_test} name_of_local_library)
# endif()

# ament_export_include_directories(include)
# # ament_export_libraries(${library_name})
# ament_export_dependencies(${dependencies})
# ament_package()

#--------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(hello)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(tf2 REQUIRED)

include_directories(
  include
)

set(dependencies 
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  Ceres
  tf2
)

# add_executable(mpc_controller src/mpc_controller.cpp)
# target_include_directories(mpc_controller PUBLIC ${Eigen3_INCLUDE_DIRS})
# ament_target_dependencies(mpc_controller ${dependencies})
# target_link_libraries(mpc_controller ${CERES_LIBRARIES})

add_executable(mpc_obstacle_avoid src/mpc_obstacle_avoid.cpp)
target_include_directories(mpc_obstacle_avoid PUBLIC ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(mpc_obstacle_avoid ${dependencies})
target_link_libraries(mpc_obstacle_avoid ${CERES_LIBRARIES})

install(TARGETS mpc_obstacle_avoid
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
