###
# mainboard_serial_bridge
###
add_executable(mainboard_serial_bridge
        src/hardware_bridge/main.cpp
        src/hardware_bridge/mainboard_serial_bridge_node.cpp)
target_compile_features(mainboard_serial_bridge PUBLIC c_std_99 cxx_std_17)
target_include_directories(mainboard_serial_bridge PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(mainboard_serial_bridge "${cpp_typesupport_target}")

ament_target_dependencies(mainboard_serial_bridge
        rclcpp
        sensor_msgs
        std_msgs
        std_srvs
)

INSTALL(TARGETS mainboard_serial_bridge
        DESTINATION lib/${PROJECT_NAME})
