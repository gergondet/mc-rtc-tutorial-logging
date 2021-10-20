cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME TutorialLogging)
set(PROJECT_DESCRIPTION "TutorialLogging")
set(PROJECT_URL "")

project(${PROJECT_NAME} LANGUAGES C CXX VERSION 1.0.0)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/TutorialLogging.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/TutorialLogging.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/TutorialLogging.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

