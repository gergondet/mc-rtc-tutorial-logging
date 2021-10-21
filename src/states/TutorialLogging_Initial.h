#pragma once

#include <mc_control/fsm/State.h>

#include <mc_tasks/TransformTask.h>

#include "../plugins/JoystickState.h"

struct TutorialLogging_Initial : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  mc_rtc::Configuration config_;
  mc_tasks::TransformTaskPtr fb_task_;
  mc_tasks::TransformTaskPtr lh_task_;
  mc_tasks::TransformTaskPtr rh_task_;
  std::string joystick_;
  size_t joystick_task_ = 0;
  double joystick_translation_speed_ = 0.01;
  double joystick_rotation_speed_ = 0.01;
  bool joystick_local_transform_ = true;
  bool joystick_control_rotation_ = false;
  mc_joystick::State previous_state_;
  mc_joystick::State current_state_;
  bool done_ = false;

  void update_state(mc_control::fsm::Controller & ctl);

  mc_tasks::TransformTask & joystick_task();
};
