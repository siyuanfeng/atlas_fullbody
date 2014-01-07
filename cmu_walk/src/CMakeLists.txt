find_package( Boost REQUIRED COMPONENTS thread )
#find_package( Boost 1.46 COMPONENTS thread )

include_directories("${PROJECT_SOURCE_DIR}/include/cmu_walk")

file(GLOB SF_COMMON_SRC "sdfast_common/*.cpp" "sdfast_common/*.c" "sdfast_common/*.h")
file(GLOB MODEL_SRC "drc_pelvis/pelv_id_controller.cpp" "drc_hat/hatns_id_controller.cpp" "drc_hat/hat_id_controller.cpp" "drc_pelvis/PelvRobotState.cpp" "drc_hat/HatNSRobotState.cpp" "drc_hat/HatRobotState.cpp" "drc_pelvis/pelv_ik_controller.cpp" "drc_hat/hatns_ik_controller.cpp" "drc_hat/hat_ik_controller.cpp" "drc_pelvis/sf_pelv_ik_controller.cpp" "drc_pelvis/sf_pelv_id_controller.cpp")

# for gazebo
file(GLOB SD_PELV_GZ "drc_pelvis/sdfast/drc_pelvis_gz.cpp")
file(GLOB SD_HAT_GZ "drc_hat/sdfast/drc_hat_gz.cpp")
file(GLOB SD_HAT_NS_GZ "drc_hat/sdfast/drc_hat_ns_gz.cpp")
# for the robot
#file(GLOB SD_PELV_ATLAS "drc_pelvis/sdfast/drc_pelvis_atlas.cpp")
#file(GLOB SD_HAT_ATLAS "drc_hat/sdfast/drc_hat.cpp")
#file(GLOB SD_HAT_NS "drc_hat/sdfast/drc_hat_ns.cpp")

file(GLOB MISC_SRC "mrdplot/mrdplot.c" "RobotState.cpp" "Logger.cpp" "Command.cpp" "Foot.cpp" "spline.cpp" "quaternion.cpp" "TrajEW.cpp" "Transmission.cpp")
file(GLOB LIPM_SRC "BalancingCon.cpp" "LipmWalkingCon.cpp" "WalkingCon.cpp" "DummyFSPlanner.cpp" "lipm_planner.cpp" "AtlasStaticWalkingCon.cpp" "Task2WalkingCon.cpp")
file(GLOB FILTER_STUFF "filter/filters.cpp" "filter/kinematicfilter.cpp" "filter/kinematicfilterEric.cpp")

add_library(CMUWalkSim SHARED ${SF_COMMON_SRC} ${MODEL_SRC} ${SD_PELV_GZ} ${SD_HAT_NS_GZ} ${SD_HAT_GZ} ${MISC_SRC} ${LIPM_SRC} ${FILTER_STUFF})
#add_library(CMUWalkSim SHARED ${SF_COMMON_SRC} ${MODEL_SRC} ${SD_PELV_ATLAS} ${SD_HAT_NS_GZ} ${SD_HAT_GZ} ${MISC_SRC} ${LIPM_SRC} ${FILTER_STUFF})
  set_target_properties(CMUWalkSim PROPERTIES COMPILE_FLAGS "-O4")

# for the robot
#add_library(CMUWalkAtlas SHARED ${SF_COMMON_SRC} ${MODEL_SRC} ${SD_PELV_ATLAS} ${SD_HAT_NS} ${SD_HAT_ATLAS} ${MISC_SRC} ${LIPM_SRC} ${FILTER_STUFF})
#  set_target_properties(CMUWalkAtlas PROPERTIES COMPILE_FLAGS "-O4 -DATLAS_ONLINE")

#rosbuild_add_executable(checkCOM tests/checkCOM.cpp)
#target_link_libraries(checkCOM CMUWalkAtlas) 
#rosbuild_link_boost(checkCOM thread signals)

#rosbuild_add_executable(test_transmission tests/test_transmission.cpp)
#target_link_libraries(test_transmission CMUWalkAtlas) 
#rosbuild_link_boost(test_transmission thread signals)

#rosbuild_add_executable(test_fk tests/test_fk.cpp)
#target_link_libraries(test_fk CMUWalkAtlas) 
#rosbuild_link_boost(test_fk thread signals) 

#rosbuild_add_executable(test_ik tests/test_ik.cpp)
#target_link_libraries(test_ik CMUWalkSim) 
#rosbuild_link_boost(test_ik thread signals)

#rosbuild_add_executable(test_bdi_data tests/test_bdi_data.cpp)
#rosbuild_link_boost(test_bdi_data thread signals)
#target_link_libraries(test_bdi_data CMUWalkAtlas) 
