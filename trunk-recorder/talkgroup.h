#ifndef TALKGROUP_H
#define TALKGROUP_H

#include <iostream>
#include <stdio.h>
#include <string>
#include "global_structs.h"
//#include <sstream>

class Talkgroup {
public:
  long number;
  std::string mode;
  std::string alpha_tag;
  std::string description;
  std::string tag;
  std::string group;
  int priority;
  int sys_num;
  double squelch_db;
  bool signal_detection;


  // For Conventional
  double freq;
  double tone;          // legacy: CTCSS Hz, kept for backwards compatibility / display.
                        // tone_config below is the authoritative source for the analog
                        // recorder's gating + detection wiring.
  Tone_Config tone_config;

  Talkgroup(int sys_num, long num, std::string mode, std::string alpha_tag, std::string description, std::string tag, std::string group, int priority, unsigned long preferredNAC);
  Talkgroup(int sys_num, long num, double freq, const Tone_Config &tone_config, std::string alpha_tag, std::string description, std::string tag, std::string group, double squelch_db, bool signal_detection);

  bool is_active();
  int get_priority();
  unsigned long get_preferredNAC();
  void set_priority(int new_priority);
  void set_active(bool a);
  std::string menu_string();

private:
  unsigned long preferredNAC;
  bool active;
};

#endif
