# Run at install time (see CMakeLists.txt): expand hbot.urdf.xacro into
#   hbot.urdf      real robot - TF frames only (SLAM / Nav2 on the Pi)
#   hbot_sim.urdf  Gazebo model (sim:=true)
#   hbot_sim.sdf   SDF export of the Gazebo model, only where `gz` exists
# HBOT_URDF_DIR is set by the preceding install(CODE ...).

find_program(XACRO xacro REQUIRED)
function(hbot_xacro out)
  execute_process(
    COMMAND ${XACRO} hbot.urdf.xacro ${ARGN} -o ${out}
    WORKING_DIRECTORY ${HBOT_URDF_DIR}
    RESULT_VARIABLE rc)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "xacro failed to generate ${out}")
  endif()
endfunction()

hbot_xacro(hbot.urdf)
hbot_xacro(hbot_sim.urdf sim:=true)

find_program(GZ gz)
if(GZ)
  execute_process(
    COMMAND ${GZ} sdf -p hbot_sim.urdf
    OUTPUT_FILE hbot_sim.sdf
    WORKING_DIRECTORY ${HBOT_URDF_DIR})
else()
  message(STATUS "gz not found - skipping hbot_sim.sdf")
endif()
