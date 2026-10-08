#ifndef CALL_START_PROVENANCE_H
#define CALL_START_PROVENANCE_H

#include "systems/parser.h"

namespace call_detail {
constexpr bool started_from_update(MessageType message_type) {
  return message_type != GRANT;
}
}  // namespace call_detail

#endif
