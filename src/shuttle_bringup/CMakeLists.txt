cmake_minimum_required(VERSION 3.16)
project(shuttle_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Wrapper that runs the Gazebo server in its own process group, so stopping a
# launch stops the server too (see the script for why that needs saying).
install(PROGRAMS scripts/gz_server
  DESTINATION lib/${PROJECT_NAME}
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

# Integration tests against the simulator. Each starts Gazebo headless with its
# own transport partition (see sim.launch.py); ctest runs them one after the
# other, which matters because two simulators rendering sensors at once only
# slow each other down.
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_safety_sim.py
    TARGET test_safety_sim
    TIMEOUT 600
  )
  add_launch_test(test/test_comparison_sim.py
    TARGET test_comparison_sim
    TIMEOUT 2000
  )
endif()

ament_package()
