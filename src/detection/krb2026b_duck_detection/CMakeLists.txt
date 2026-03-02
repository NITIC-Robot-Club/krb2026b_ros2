cmake_minimum_required(VERSION 3.8)
project(krb2026b_duck_detection)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenVINO REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  openvino::runtime
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "duck_detection::duck_detection"
    EXECUTABLE duck_detection
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)