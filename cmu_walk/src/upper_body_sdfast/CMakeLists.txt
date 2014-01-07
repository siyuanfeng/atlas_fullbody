add_executable(test sdlib.c drc_pelvis_upper_d.c drc_pelvis_upper_s.c main.c)
  set_target_properties(test PROPERTIES COMPILE_FLAGS "-g -Wall -pedantic")
target_link_libraries(test m) 

add_executable(atlas sdlib.c atlas_upper_d.c atlas_upper_s.c main.c)
  set_target_properties(atlas PROPERTIES COMPILE_FLAGS "-g -Wall -pedantic")
target_link_libraries(atlas m)  

#rosbuild_add_executable(test_ik tests/test_ik.cpp)
#target_link_libraries(test_ik CMUWalkSim) 
#rosbuild_link_boost(test_ik thread signals)

#rosbuild_add_executable(test_bdi_data tests/test_bdi_data.cpp)
#rosbuild_link_boost(test_bdi_data thread signals)
#target_link_libraries(test_bdi_data CMUWalkAtlas) 
