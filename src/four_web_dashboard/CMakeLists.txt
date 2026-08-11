cmake_minimum_required(VERSION 3.8)
project(four_web_dashboard)

find_package(ament_cmake REQUIRED)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
