cmake_minimum_required(VERSION 3.8)
project(krb2026b_behavior)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "bottle_collector::bottle_collector"
    EXECUTABLE bottle_collector
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "duck_collector::duck_collector"
    EXECUTABLE duck_collector
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)