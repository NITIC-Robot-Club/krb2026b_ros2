cmake_minimum_required(VERSION 3.8)
project(krb2026b_bottle_detection)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "bottle_detection::bottle_detection"
    EXECUTABLE bottle_detection
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
