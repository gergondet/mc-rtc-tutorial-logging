include(FindPkgConfig)
pkg_check_modules(sensor_msgs REQUIRED IMPORTED_TARGET sensor_msgs)

set(Joystick_SRC
  Joystick.cpp
)

set(Joystick_HDR
  Joystick.h
)

option(AUTOLOAD_JoystickPlugin_PLUGIN "Automatically load Joystick plugin" OFF)
add_plugin(JoystickPlugin "${Joystick_SRC}" "${Joystick_HDR}")
target_link_libraries(JoystickPlugin PUBLIC mc_rtc::mc_rtc_ros PkgConfig::sensor_msgs)
set_target_properties(JoystickPlugin PROPERTIES OUTPUT_NAME Joystick)
install(FILES etc/Joystick.yaml DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)
