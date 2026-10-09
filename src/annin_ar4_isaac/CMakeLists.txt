cmake_minimum_required(VERSION 3.5)
project(annin_ar4_isaac)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rosgraph_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(annin_ar4_isaac_hw SHARED
  src/isaac_tcp_system.cpp
)
target_compile_features(annin_ar4_isaac_hw PUBLIC cxx_std_17)
target_include_directories(annin_ar4_isaac_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(annin_ar4_isaac_hw ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

install(TARGETS annin_ar4_isaac_hw
  EXPORT export_annin_ar4_isaac_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch urdf isaac_sim
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/export_isaac_urdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_annin_ar4_isaac_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
