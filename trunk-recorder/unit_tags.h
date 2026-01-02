#ifndef UNIT_TAGS_H
#define UNIT_TAGS_H

#include "unit_tag.h"
#include "unit_tags_ota.h"

#include <string>
#include <vector>

enum UnitTagMode {
  TAG_USER_FIRST = 0, // Search user tags first, then OTA
  TAG_OTA_FIRST = 1,  // Search OTA tags first, then user
  TAG_USER_ONLY = 3,  // Only search user tags, ignore OTA (still write OTA to disk if configured)
  TAG_NONE = -1       // Don't search any tags
};

class UnitTags {
  std::vector<UnitTag *> unit_tags;                  // Manual tags from unitTagsFile (regex patterns)
  std::vector<UnitTagOTA *> unit_tags_ota;           // OTA tags: simple (unitID, alias) pairs
  std::string ota_filename;
  UnitTagMode mode = TAG_USER_FIRST;                 // Default to user tags first

public:
  void load_unit_tags(std::string filename);
  void load_unit_tags_ota(std::string filename);
  std::string find_unit_tag(long unitID);
  void add(std::string pattern, std::string tag);
  bool add_ota(const OTAAlias& ota_alias);
  void set_mode(UnitTagMode mode);
  UnitTagMode get_mode();
  std::vector<UnitTag *> get_unit_tags();
  std::vector<UnitTagOTA *> get_unit_tags_ota();
};
#endif // UNIT_TAGS_H
