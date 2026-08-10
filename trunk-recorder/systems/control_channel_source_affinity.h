#ifndef CONTROL_CHANNEL_SOURCE_AFFINITY_H
#define CONTROL_CHANNEL_SOURCE_AFFINITY_H

enum class ControlChannelRetuneSourceAction {
  RetuneOnCurrentSource,
  SearchOtherSources,
  BlockCrossSourceRetune,
};

constexpr ControlChannelRetuneSourceAction control_channel_retune_source_action(
    bool source_affinity_enabled,
    bool current_source_covers_control_channel) {
  if (current_source_covers_control_channel) {
    return ControlChannelRetuneSourceAction::RetuneOnCurrentSource;
  }

  return source_affinity_enabled
             ? ControlChannelRetuneSourceAction::BlockCrossSourceRetune
             : ControlChannelRetuneSourceAction::SearchOtherSources;
}

#endif
