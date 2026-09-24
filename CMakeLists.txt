cmake_minimum_required(VERSION 3.8)
project(hbot_description)

find_package(ament_cmake REQUIRED)

# The description is urdf/hbot.urdf.xacro (+ config/hbot_geometry.yaml).
# Launch files expand it at run time (launch/description.launch.py). For
# consumers that load a plain file, the two variants are also generated here,
# into the BUILD directory, and installed to share/hbot_description/urdf/:
#   hbot.urdf      real robot            (xacro hbot.urdf.xacro)
#   hbot_sim.urdf  Gazebo model          (xacro hbot.urdf.xacro use_sim:=true)
#   hbot_sim.sdf   SDF export of the Gazebo model, only where `gz` exists
# Nothing is written into the source tree.
find_program(XACRO xacro REQUIRED)
set(HBOT_XACRO ${CMAKE_CURRENT_SOURCE_DIR}/urdf/hbot.urdf.xacro)
file(GLOB HBOT_XACRO_DEPS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro
  ${CMAKE_CURRENT_SOURCE_DIR}/config/*.yaml)
set(HBOT_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/urdf)
file(MAKE_DIRECTORY ${HBOT_GEN_DIR})

add_custom_command(
  OUTPUT ${HBOT_GEN_DIR}/hbot.urdf
  COMMAND ${XACRO} ${HBOT_XACRO} -o ${HBOT_GEN_DIR}/hbot.urdf
  DEPENDS ${HBOT_XACRO_DEPS}
  COMMENT "xacro -> hbot.urdf (real robot)")
add_custom_command(
  OUTPUT ${HBOT_GEN_DIR}/hbot_sim.urdf
  COMMAND ${XACRO} ${HBOT_XACRO} use_sim:=true -o ${HBOT_GEN_DIR}/hbot_sim.urdf
  DEPENDS ${HBOT_XACRO_DEPS}
  COMMENT "xacro -> hbot_sim.urdf (Gazebo)")
set(HBOT_GENERATED ${HBOT_GEN_DIR}/hbot.urdf ${HBOT_GEN_DIR}/hbot_sim.urdf)

# The Pi's arm64 image has no Gazebo, so the SDF is optional.
find_program(GZ gz)
if(GZ)
  add_custom_command(
    OUTPUT ${HBOT_GEN_DIR}/hbot_sim.sdf
    COMMAND ${GZ} sdf -p ${HBOT_GEN_DIR}/hbot_sim.urdf > ${HBOT_GEN_DIR}/hbot_sim.sdf
    DEPENDS ${HBOT_GEN_DIR}/hbot_sim.urdf
    COMMENT "gz sdf -> hbot_sim.sdf")
  list(APPEND HBOT_GENERATED ${HBOT_GEN_DIR}/hbot_sim.sdf)
else()
  message(STATUS "gz not found - skipping hbot_sim.sdf")
endif()
add_custom_target(hbot_urdf ALL DEPENDS ${HBOT_GENERATED})

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
install(FILES ${HBOT_GENERATED}
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py
    TIMEOUT 60)
endif()

ament_package()
