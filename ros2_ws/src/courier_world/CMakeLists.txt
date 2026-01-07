cmake_minimum_required(VERSION 3.5)
project(courier_world)  # <-- matches package.xml

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
