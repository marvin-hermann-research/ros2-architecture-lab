cmake_minimum_required(VERSION 3.5)
project(bipedal_robot_pkg)

# Finde benötigte ROS2-Pakete
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Deklariere ROS2-Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCondition.srv"
  "srv/SetWalkingMode.srv"
  "srv/GetStatus.srv"
  "srv/EmergencyStop.srv"
)

# Installiere dein Python-Modul
ament_python_install_package(${PROJECT_NAME})

# Exportiere Runtime-Abhängigkeiten
ament_export_dependencies(rosidl_default_runtime)

# Installiere Launch-Dateien o.ä., wenn vorhanden
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
