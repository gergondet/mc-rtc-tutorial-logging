#include "TutorialLogging_Initial.h"

#include "../TutorialLogging.h"

void TutorialLogging_Initial::configure(const mc_rtc::Configuration & config) {}

void TutorialLogging_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TutorialLogging &>(ctl_);
}

bool TutorialLogging_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TutorialLogging &>(ctl_);
  output("OK");
  return true;
}

void TutorialLogging_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TutorialLogging &>(ctl_);
}

EXPORT_SINGLE_STATE("TutorialLogging_Initial", TutorialLogging_Initial)
