#include "TutorialLogging_Initial.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/MetaTaskLoader.h>

void TutorialLogging_Initial::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void TutorialLogging_Initial::start(mc_control::fsm::Controller & ctl)
{
  const auto & available_joysticks = ctl.datastore().get<std::vector<std::string>>("Joystick::connected");
  if(available_joysticks.size() == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No joystick connected, bye!", name());
  }
  joystick_ = available_joysticks[0] + "::state";
  update_state(ctl);
  previous_state_ = current_state_;

#if MC_RTC_VERSION_MAJOR > 1
  auto transform_task_without_fb = [](mc_tasks::TransformTask & tf) {
#else
  auto transform_task_without_fb = [&](mc_tasks::SurfaceTransformTask & tf) {
#endif
    std::vector<std::string> active_joints;
#if MC_RTC_VERSION_MAJOR > 1
    auto jointsPath = tf.frame().rbdJacobian().jointsPath();
#else
    auto jointsPath = rbd::Jacobian(ctl.robot().mb(), ctl.robot().surface(tf.surface()).bodyName()).jointsPath();
#endif
    for(const auto & jIndex : jointsPath)
    {
#if MC_RTC_VERSION_MAJOR > 1
      const auto & joint = tf.robot().mb().joint(jIndex);
#else
      const auto & joint = ctl.robot().mb().joint(jIndex);
#endif
      if(joint.dof() == 1)
      {
        active_joints.push_back(joint.name());
      }
    }
    tf.selectActiveJoints(active_joints);
  };

  auto fb_config = config_("FloatingBase", mc_rtc::Configuration{});
#if MC_RTC_VERSION_MAJOR > 1
  fb_config.add("type", "transform");
  fb_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), fb_config);
#else
  fb_config.add("type", "body6d");
  fb_config.add("body", fb_config("frame"));
  fb_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(ctl.solver(), fb_config);
#endif

  auto lh_config = config_("LeftHand", mc_rtc::Configuration{});
#if MC_RTC_VERSION_MAJOR > 1
  lh_config.add("type", "transform");
  lh_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), lh_config);
#else
  lh_config.add("type", "surfaceTransform");
  lh_config.add("surface", lh_config("frame"));
  lh_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::SurfaceTransformTask>(ctl.solver(), lh_config);
#endif
  transform_task_without_fb(*lh_task_);

  auto rh_config = config_("RightHand", mc_rtc::Configuration{});
#if MC_RTC_VERSION_MAJOR > 1
  rh_config.add("type", "transform");
  rh_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), rh_config);
#else
  rh_config.add("type", "surfaceTransform");
  rh_config.add("surface", rh_config("frame"));
  rh_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::SurfaceTransformTask>(ctl.solver(), rh_config);
#endif
  transform_task_without_fb(*rh_task_);

  auto joystick_config = config_("joystick", mc_rtc::Configuration{});
  joystick_config("translation_speed", joystick_translation_speed_);
  joystick_config("rotation_speed", joystick_rotation_speed_);

  ctl.solver().addTask(fb_task_);
  ctl.solver().addTask(lh_task_);
  ctl.solver().addTask(rh_task_);

#if MC_RTC_VERSION_MAJOR > 1
  ctl.gui().addElement(
#else
  ctl.gui()->addElement(
#endif
      {}, mc_rtc::gui::Label("Task controlled by joystick:", [this]() { return joystick_task().name(); }),
      mc_rtc::gui::Label("Control frame:", [this]() { return joystick_local_transform_ ? "Local" : "Global"; }),
      mc_rtc::gui::Label("Control type:",
                         [this]() { return joystick_control_rotation_ ? "Rotation" : "Translation"; }));
  output("OK");
}

bool TutorialLogging_Initial::run(mc_control::fsm::Controller & ctl)
{
  using Axis = mc_joystick::Axis;
  using Button = mc_joystick::Button;
  update_state(ctl);
  auto button_pressed = [this](Button b) { return current_state_.buttons[b] && !previous_state_.buttons[b]; };
  if(button_pressed(Button::L1))
  {
    joystick_task_ = joystick_task_ == 0 ? 2 : joystick_task_ - 1;
  }
  if(button_pressed(Button::R1))
  {
    joystick_task_ = (joystick_task_ + 1) % 3;
  }
  joystick_control_rotation_ = current_state_.axes[Axis::L2] <= 0.0;
  if(joystick_control_rotation_)
  {
    joystick_local_transform_ = current_state_.axes[Axis::R2] >= 0.0;
  }
  else
  {
    joystick_local_transform_ = current_state_.axes[Axis::R2] <= 0.0;
  }

#if MC_RTC_VERSION_MAJOR > 1
  auto & task = joystick_task();
  sva::PTransformd target = task.target();
#else
  sva::PTransformd target = joystick_task_target();
#endif
  sva::PTransformd offset = sva::PTransformd::Identity();
  if(joystick_control_rotation_)
  {
    Eigen::Vector3d rpy = {current_state_.axes[Axis::Left_LR], current_state_.axes[Axis::Left_UD],
                           current_state_.axes[Axis::Right_LR]};
    rpy *= joystick_rotation_speed_;
    offset.rotation() = sva::RotX(rpy.x()) * sva::RotY(rpy.y()) * sva::RotZ(rpy.z());
  }
  else
  {
    offset.translation().x() += current_state_.axes[Axis::Left_UD] * joystick_translation_speed_;
    offset.translation().y() += current_state_.axes[Axis::Left_LR] * joystick_translation_speed_;
    offset.translation().z() += current_state_.axes[Axis::Right_UD] * joystick_translation_speed_;
  }
  if(joystick_local_transform_)
  {
    target = offset * target;
  }
  else
  {
    target = target * offset;
  }
#if MC_RTC_VERSION_MAJOR > 1
  task.target(target);
#else
  joystick_task_target(target);
#endif
  return done_;
}

void TutorialLogging_Initial::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(fb_task_);
  ctl.solver().removeTask(lh_task_);
  ctl.solver().removeTask(rh_task_);
}

void TutorialLogging_Initial::update_state(mc_control::fsm::Controller & ctl)
{
  previous_state_ = current_state_;
  current_state_ = *ctl.datastore().get<const mc_joystick::State *>(joystick_);
}

#if MC_RTC_VERSION_MAJOR > 1
mc_tasks::TransformTask & TutorialLogging_Initial::joystick_task()
#else
mc_tasks::MetaTask & TutorialLogging_Initial::joystick_task()
#endif
{
  switch(joystick_task_)
  {
    case 0:
      return *fb_task_;
    case 1:
      return *lh_task_;
    default:
      return *rh_task_;
  }
}

sva::PTransformd TutorialLogging_Initial::joystick_task_target()
{
  switch(joystick_task_)
  {
    case 0:
      return fb_task_->get_ef_pose();
    case 1:
      return lh_task_->target();
    default:
      return rh_task_->target();
  }
}

void TutorialLogging_Initial::joystick_task_target(const sva::PTransformd & target)
{
  switch(joystick_task_)
  {
    case 0:
      fb_task_->set_ef_pose(target);
      break;
    case 1:
      lh_task_->target(target);
      break;
    default:
      rh_task_->target(target);
  }
}

EXPORT_SINGLE_STATE("TutorialLogging_Initial", TutorialLogging_Initial)
