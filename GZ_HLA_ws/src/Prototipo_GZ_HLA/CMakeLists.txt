cmake_minimum_required(VERSION 3.8)
project(Prototipo_GZ_HLA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/spawn_robot.py
  scripts/pose_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
