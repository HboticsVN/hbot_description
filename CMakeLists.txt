cmake_minimum_required(VERSION 3.8)
project(hbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Generate urdf/hbot.urdf (real robot) + urdf/hbot_sim.urdf (Gazebo) from
# hbot.urdf.xacro before urdf/ is installed.
install(CODE "set(HBOT_URDF_DIR \"${CMAKE_CURRENT_SOURCE_DIR}/urdf\")")
install(SCRIPT cmake/generate_urdf.cmake)

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
