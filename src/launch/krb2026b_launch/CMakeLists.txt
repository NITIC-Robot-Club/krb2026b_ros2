cmake_minimum_required(VERSION 3.8)
project(krb2026b_launch)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    rviz
    map
    param
)
