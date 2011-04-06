set(vicon_lib_name "ViconDataStreamSDK_CPP_x86")

add_library(DataStreamSDK_Wrapper data_stream_client.cpp)
target_link_libraries(DataStreamSDK_Wrapper -l${vicon_lib_name} -m32)         

# Create an executable program vicon-client
add_executable(vicon-client vicon_client.cpp)
target_link_libraries(vicon-client ${LCMTYPES_LIBS} DataStreamSDK_Wrapper)
pods_use_pkg_config_packages(vicon-client lcm)

# make executable public
pods_install_executables(vicon-client)

#copy over/install the vicon sdk shared object
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/lib${vicon_lib_name}.so ${CMAKE_BINARY_DIR}/lib/lib${vicon_lib_name}.so COPYONLY)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib${vicon_lib_name}.so 
                DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)


