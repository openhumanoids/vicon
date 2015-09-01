add_executable(vicon-client-segments ViconHolodeckDriver.cpp)
target_link_libraries(vicon-client-segments lcm ViconDataStreamSDK_CPP)
pods_use_pkg_config_packages(vicon-client-segments lcm lcmtypes_vicon)
pods_install_executables(vicon-client-segments)
