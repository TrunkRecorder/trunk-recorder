#include "trunk-recorder/systems/control_channel_source_affinity.h"

#include <cassert>

int main() {
  assert(control_channel_retune_source_action(false, true) ==
         ControlChannelRetuneSourceAction::RetuneOnCurrentSource);
  assert(control_channel_retune_source_action(true, true) ==
         ControlChannelRetuneSourceAction::RetuneOnCurrentSource);
  assert(control_channel_retune_source_action(false, false) ==
         ControlChannelRetuneSourceAction::SearchOtherSources);
  assert(control_channel_retune_source_action(true, false) ==
         ControlChannelRetuneSourceAction::BlockCrossSourceRetune);
  return 0;
}
