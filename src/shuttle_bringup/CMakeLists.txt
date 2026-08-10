cmake_minimum_required(VERSION 3.16)
project(shuttle_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Worlds and maps live outside the package, at the repository root, because they
# are simulation assets rather than ROS package content. Install them into the
# package share so launch files can find them through FindPackageShare.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../simulation/worlds
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../data/maps)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../data/maps/
    DESTINATION share/${PROJECT_NAME}/maps
  )
endif()

ament_package()
