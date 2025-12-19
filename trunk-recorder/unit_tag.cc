#include "unit_tag.h"
#include <boost/regex.hpp>

UnitTag::UnitTag(std::string p, std::string t) {
  pattern = p;
  tag = t;
}

UnitTagOTA::UnitTagOTA(long id, std::string a, std::string src, std::string w, std::string s, long tg, time_t ts) {
  unit_id = id;
  alias = a;
  source = src;
  wacn = w;
  sys = s;
  talkgroup_id = tg;
  timestamp = ts;
}
