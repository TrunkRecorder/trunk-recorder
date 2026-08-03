#include "../trunk-recorder/call_start_provenance.h"

int main() {
  if (call_detail::started_from_update(GRANT)) return 1;
  if (!call_detail::started_from_update(UPDATE)) return 1;
  return 0;
}
