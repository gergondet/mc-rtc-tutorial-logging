include(FindPkgConfig)
pkg_check_modules(sensor_msgs REQUIRED IMPORTED_TARGET sensor_msgs)

set(Joystick_SRC
  Joystick.cpp
)

set(Joystick_HDR
  Joystick.h
)

option(AUTOLOAD_Joystick_PLUGIN "Automatically load Joystick plugin" OFF)
add_plugin(Joystick "${Joystick_SRC}" "${Joystick_HDR}")
target_link_libraries(Joystick PUBLIC mc_rtc::mc_rtc_ros PkgConfig::sensor_msgs)
install(FILES etc/Joystick.yaml DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)
