cmake_minimum_required(VERSION 3.5)
project(py_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlateInfo.srv"
  "msg/PlateData.msg"
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    py_srvcli/plate_client.py
    py_srvcli/plate_server.py
    py_srvcli/plate_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
