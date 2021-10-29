#include "Initial.h"

#include <mc_control/fsm/Controller.h>

void Initial::start(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->addElement({},
                        mc_rtc::gui::Button("Live", [this]() { output("Live"); }),
                        mc_rtc::gui::Button("Replay", [this]() { output("Replay"); }));
}

bool Initial::run(mc_control::fsm::Controller & ctl)
{
  return !output().empty();
}

void Initial::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElement({}, "Live");
  ctl.gui()->removeElement({}, "Replay");
}

EXPORT_SINGLE_STATE("Initial", Initial)
