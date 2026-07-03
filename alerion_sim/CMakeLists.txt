cmake_minimum_required(VERSION 3.8)
project(alerion_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Instala los archivos de lanzamiento
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Instala config, models y worlds para que sean accesibles via
# ament_index_python.packages.get_package_share_directory('alerion_sim')
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds  DESTINATION share/${PROJECT_NAME})

# Register Python nodes so they are launchable via
#   Node(package="alerion_sim", executable="<name>", ...)
# and runnable with `ros2 run alerion_sim <name>`.
install(PROGRAMS
  scripts/drone_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME drone_visualizer
)

install(PROGRAMS
  scripts/gimbal_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gimbal_controller
)

install(PROGRAMS
  scripts/wind_turbulence.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wind_turbulence
)

install(PROGRAMS
  scripts/camera_distortion.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_distortion
)

install(PROGRAMS
  scripts/validation_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validation_node
)

ament_package()
