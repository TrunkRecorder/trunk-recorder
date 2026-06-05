#include "talkgroup.h"

Talkgroup::Talkgroup(int sys_num, long num, std::string mode, std::string alpha_tag, std::string description, std::string tag, std::string group, int priority, unsigned long preferredNAC) {
  this->sys_num = sys_num;
  this->number = num;
  this->mode = mode;
  this->alpha_tag = alpha_tag;
  this->description = description;
  this->tag = tag;
  this->group = group;
  this->priority = priority;
  this->active = false;
  this->preferredNAC = preferredNAC;

  // This talkgroup is for a Trunked system and freq and tone are not used
  this->freq = 0;
  this->tone = 0;
  this->tone_config = Tone_Config{}; // TONE_OFF
  this->squelch_db = DB_UNSET;
  this->signal_detection = false;

}

Talkgroup::Talkgroup(int sys_num, long num, double freq, const Tone_Config &tone_config, std::string alpha_tag, std::string description, std::string tag, std::string group, double squelch_db, bool signal_detection) {
  this->sys_num = sys_num;
  this->number = num;
  this->mode = "Z";
  this->alpha_tag = alpha_tag;
  this->description = description;
  this->tag = tag;
  this->group = group;
  this->active = false;
  this->freq = freq;
  this->tone_config = tone_config;
  // Keep the legacy `tone` field populated for CTCSS so any downstream
  // consumer still reading it as a Hz value (logs, plugins, etc.) keeps
  // working. DCS / search modes leave it at 0.
  this->tone = (tone_config.mode == TONE_CTCSS) ? tone_config.ctcss_hz : 0.0;
  this->squelch_db = squelch_db;
  this->signal_detection = signal_detection;

  // This talkgroup is for a Conventional system and priority and preferredNAC are not used
  this->priority = 0;
  this->preferredNAC = 0;
}

std::string Talkgroup::menu_string() {
  char buff[150];

  // std::ostringstream oss;

  snprintf(buff, 150, "%5lu - %-15s %-20s %-15s %-40s", number, alpha_tag.c_str(), tag.c_str(), group.c_str(), description.c_str());

  // sprintf(buff, "%5lu - %s", number, alpha_tag.c_str());

  std::string buffAsStdStr = buff;

  return buffAsStdStr;
}

int Talkgroup::get_priority() {
  return priority;
}

unsigned long Talkgroup::get_preferredNAC() {
  return preferredNAC;
}

void Talkgroup::set_priority(int new_priority) {
  priority = new_priority;
  return;
}

bool Talkgroup::is_active() {
  return active;
}

void Talkgroup::set_active(bool a) {
  active = a;
}
