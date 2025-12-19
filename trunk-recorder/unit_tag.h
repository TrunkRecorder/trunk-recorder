#ifndef UNIT_TAG_H
#define UNIT_TAG_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <boost/regex.hpp>

// User defined tag structure with regex pattern matching
class UnitTag {
public:
  boost::regex pattern;
  std::string tag;

  UnitTag(std::string p, std::string t);
};

// Long int OTA tag structure for fast number-to-alias lookups
class UnitTagOTA {
public:
  long unit_id;
  std::string alias;
  std::string source;
  std::string wacn;
  std::string sys;
  long talkgroup_id;
  time_t timestamp;

  UnitTagOTA(long id, std::string a, std::string src, std::string w, std::string s, long tg, time_t ts);
};

#endif // UNIT_TAG_H
