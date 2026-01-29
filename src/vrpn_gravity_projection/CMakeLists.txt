cmake_minimum_required(VERSION 3.0.2)
project(vrpn_gravity_projection)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# 确保 Eigen3 被正确查找
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
  DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} # 必须包含 Eigen 路径
)

## --- 编译核心集成节点 (50Hz版) ---
add_executable(reset_four_element_node src/reset_four_element.cpp)
target_link_libraries(reset_four_element_node
  ${catkin_LIBRARIES}
)
# 将输出名称设置为 reset_four_element，方便在 launch 中调用
set_target_properties(reset_four_element_node PROPERTIES OUTPUT_NAME reset_four_element)

## --- 编译其他辅助节点 (保留原有配置) ---
add_executable(vrpn_gravity_projection_node src/vrpn_gravity_projection.cpp)
target_link_libraries(vrpn_gravity_projection_node ${catkin_LIBRARIES})
set_target_properties(vrpn_gravity_projection_node PROPERTIES OUTPUT_NAME vrpn_gravity_projection)

add_executable(twist_transform_node src/twist_transform.cpp)
target_link_libraries(twist_transform_node ${catkin_LIBRARIES})
set_target_properties(twist_transform_node PROPERTIES OUTPUT_NAME twist_transform)

## --- 安装规则 ---
install(TARGETS 
  reset_four_element_node 
  vrpn_gravity_projection_node 
  twist_transform_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)