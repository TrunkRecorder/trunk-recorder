#include "unit_tags.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/log/trivial.hpp>
#include <boost/tokenizer.hpp>
#include <boost/regex.hpp>

#include <csv-parser/csv.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>

using namespace csv;

void UnitTags::load_unit_tags(std::string filename) {
  if (filename == "") {
    return;
  }

  CSVFormat format;
  format.trim({' ', '\t'});
  format.header_row(-1);  // No header row expected
  format.column_names({"unit_id", "tag"});
  
  try {
    CSVReader reader(filename, format);
    
    int lines_loaded = 0;
    for (CSVRow &row : reader) {
      if (row.size() < 2) {
        continue;
      }
      
      std::string pattern = row["unit_id"].get<>();
      std::string tag = row["tag"].get<>();
      
      add(pattern, tag);
      lines_loaded++;
    }
    
    BOOST_LOG_TRIVIAL(info) << "Read " << lines_loaded << " unit tags.";
  } catch (std::exception &e) {
    BOOST_LOG_TRIVIAL(error) << "Error reading Unit Tag File: " << filename << " - " << e.what();
  }
}

void UnitTags::load_unit_tags_ota(std::string filename) {
  ota_filename = filename;
  
  if (filename == "") {
    return;
  }

  if (mode == TAG_NONE) {
    return;
  }

  std::ifstream test(filename);
  if (!test.good()) {
    return;  // File doesn't exist yet, that's ok!
  }
  test.close();

  CSVFormat format;
  format.trim({' ', '\t'});
  format.header_row(-1);  // No header row
  format.column_names({"unit_id", "tag", "source", "timestamp", "wacn", "sys", "talkgroup_id"});
  
  try {
    CSVReader reader(filename, format);
    
    int lines_loaded = 0;
    int lines_needing_update = 0;
    for (CSVRow &row : reader) {
      if (row.size() < 2) {
        continue;
      }

      // Mandatory fields
      long unit_id = std::stol(row["unit_id"].get<>());
      std::string tag = row["tag"].get<>();
      
      // Optional fields with defaults
      std::string source = (row.size() >= 3) ? row["source"].get<>() : "";
      time_t ts = (row.size() >= 4) ? std::stol(row["timestamp"].get<>()) : 0;
      std::string wacn = (row.size() >= 7) ? row["wacn"].get<>() : "";
      std::string sys = (row.size() >= 7) ? row["sys"].get<>() : "";
      std::string tg_str = (row.size() >= 7) ? row["talkgroup_id"].get<>() : "";
      long tg = tg_str.empty() ? -1 : std::stol(tg_str);
      
      if (wacn.empty()) {
        lines_needing_update++;
      }
      
      UnitTagOTA *ota_tag = new UnitTagOTA(unit_id, tag, source, wacn, sys, tg, ts);
      unit_tags_ota.push_back(ota_tag);
      lines_loaded++;
    }
    
    if (lines_loaded > 0) {
      BOOST_LOG_TRIVIAL(info) << "Loaded " << lines_loaded << " OTA unit tags.";
      
      // Check if data is already sorted by unit_id
      bool is_sorted = true;
      for (size_t i = 1; i < unit_tags_ota.size(); i++) {
        if (unit_tags_ota[i-1]->unit_id > unit_tags_ota[i]->unit_id) {
          is_sorted = false;
          break;
        }
      }
      
      // Deduplicate: keep newest entry per unit_id
      std::map<long, UnitTagOTA*> unique_tags;
      int duplicates_removed = 0;
      
      for (auto ota_tag : unit_tags_ota) {
        auto result = unique_tags.insert(std::make_pair(ota_tag->unit_id, ota_tag));
        
        if (!result.second) {
          // Duplicate found - compare timestamps and metadata completeness
          UnitTagOTA *current = result.first->second;
          bool replace = false;
          if (ota_tag->timestamp > current->timestamp) {
            replace = true;
          } else if (ota_tag->timestamp == current->timestamp && !ota_tag->wacn.empty() && current->wacn.empty()) {
            replace = true;
          }
          
          if (replace) {
            delete current;
            result.first->second = ota_tag;
          } else {
            delete ota_tag;
          }
          duplicates_removed++;
        }
      }
      
      if (duplicates_removed > 0 || !is_sorted || lines_needing_update > 0) {
        if (duplicates_removed > 0) {
          BOOST_LOG_TRIVIAL(info) << " Found " << duplicates_removed << " duplicate OTA entries";
        }
        if (!is_sorted) {
          BOOST_LOG_TRIVIAL(info) << " OTA CSV is unsorted, reorganizing by unit ID";
        }
        if (lines_needing_update > 0) {
          BOOST_LOG_TRIVIAL(info) << " " << lines_needing_update << " OTA tags with incomplete metadata, will update as discovered";
        }
        
        unit_tags_ota.clear();
        for (auto &pair : unique_tags) {
          unit_tags_ota.push_back(pair.second);
        }
        
        // Atomic rewrite: temp file + rename
        try {
          std::string temp_file = filename + ".tmp";
          std::ofstream out(temp_file, std::ios::trunc);
          if (out.is_open()) {
            CSVWriter<std::ofstream> writer(out);
            for (UnitTagOTA *ota_tag : unit_tags_ota) {
              writer << std::vector<std::string>{
                std::to_string(ota_tag->unit_id),
                ota_tag->alias,
                ota_tag->source,
                std::to_string(ota_tag->timestamp),
                ota_tag->wacn,
                ota_tag->sys,
                (ota_tag->talkgroup_id == -1) ? "" : std::to_string(ota_tag->talkgroup_id)
              };
            }
            out.close();
            
            if (std::rename(temp_file.c_str(), filename.c_str()) == 0) {
              BOOST_LOG_TRIVIAL(info) << "OTA CSV cleaned and sorted successfully (" << unique_tags.size() << " entries)";
            } else {
              BOOST_LOG_TRIVIAL(error) << "Failed to rename cleaned CSV";
            }
          }
        } catch (std::exception &e) {
          BOOST_LOG_TRIVIAL(error) << "Error rewriting CSV: " << e.what();
        }
      }
    }
  } catch (std::exception &e) {
    BOOST_LOG_TRIVIAL(error) << "Error reading OTA Unit Tag File: " << filename << " - " << e.what();
  }
}

std::string UnitTags::find_unit_tag(long tg_number) {
  // TAG_NONE: Don't search any tags
  if (mode == TAG_NONE) {
    return "";
  }

  std::string tg_num_str = std::to_string(tg_number);
  
  // Helper lambda: Search user tags
  auto search_user_tags = [&]() -> std::string {
    for (std::vector<UnitTag *>::iterator it = unit_tags.begin(); it != unit_tags.end(); ++it) {
      UnitTag *tg = (UnitTag *)*it;
      if (regex_match(tg_num_str, tg->pattern)) {
        return regex_replace(tg_num_str, tg->pattern, tg->tag, boost::regex_constants::format_no_copy | boost::regex_constants::format_all);
      }
    }
    return "";
  };
  
  // Helper lambda: Search OTA tags
  auto search_ota_tags = [&]() -> std::string {
    for (auto it = unit_tags_ota.rbegin(); it != unit_tags_ota.rend(); ++it) {
      UnitTagOTA *ota_tag = *it;
      if (ota_tag->unit_id == tg_number) {
        return ota_tag->alias;
      }
    }
    return "";
  };
  
  // TAG_USER_FIRST: Search user tags first, then OTA
  if (mode == TAG_USER_FIRST) {
    std::string tag = search_user_tags();
    if (!tag.empty()) return tag;
    return search_ota_tags();
  }
  
  // TAG_OTA_FIRST: Search OTA tags first, then user tags
  if (mode == TAG_OTA_FIRST) {
    std::string tag = search_ota_tags();
    if (!tag.empty()) return tag;
    return search_user_tags();
  }

  // TAG_USER_ONLY: Only search user tags
  if (mode == TAG_USER_ONLY) {
    return search_user_tags();
  }

  return "";
}

void UnitTags::add(std::string pattern, std::string tag) {
  // If the pattern is like /someregex/
  if (pattern.substr(0, 1).compare("/") == 0 && pattern.substr(pattern.length()-1, 1).compare("/") == 0) {
    // then remove the / at the beginning and end
    pattern = pattern.substr(1, pattern.length()-2);
  } else {
    // otherwise add ^ and $ to the pattern e.g. ^123$ to make a regex for simple IDs
    pattern = "^" + pattern + "$";
  }
  UnitTag *unit_tag = new UnitTag(pattern, tag);
  unit_tags.push_back(unit_tag);
}

bool UnitTags::add_ota(const OTAAlias& ota_alias) {
  if (!ota_alias.success) {
    return false;
  }
  
  // If mode is TAG_NONE, don't process or write OTA tags
  if (mode == TAG_NONE) {
    return false;
  }
  
  // Check if this unit already has an OTA tag (search OTA list only)
  UnitTagOTA *existing_ota = nullptr;
  for (auto it = unit_tags_ota.rbegin(); it != unit_tags_ota.rend(); ++it) {
    UnitTagOTA *ota_tag = *it;
    if (ota_tag->unit_id == ota_alias.radio_id) {
      existing_ota = ota_tag;
      break;
    }
  }
  
  if (existing_ota) {
    if (existing_ota->alias == ota_alias.alias) {
      // check if decode metadata is missing
      bool needs_enrichment = (existing_ota->wacn.empty() && !ota_alias.wacn.empty()) ||
                              (existing_ota->sys.empty() && !ota_alias.sys.empty()) ||
                              (existing_ota->talkgroup_id == -1 && ota_alias.talkgroup_id != -1);
      
      if (needs_enrichment) {
        BOOST_LOG_TRIVIAL(debug) << "Unit " << ota_alias.radio_id << " (" << ota_alias.alias << "): enriching with metadata (WACN: " << ota_alias.wacn << ", SYS: " << ota_alias.sys << ", TG: " << ota_alias.talkgroup_id << ")";
        
        if (!ota_alias.source.empty()) existing_ota->source = ota_alias.source;
        if (!ota_alias.wacn.empty()) existing_ota->wacn = ota_alias.wacn;
        if (!ota_alias.sys.empty()) existing_ota->sys = ota_alias.sys;
        if (ota_alias.talkgroup_id != -1) existing_ota->talkgroup_id = ota_alias.talkgroup_id;
        existing_ota->timestamp = std::time(nullptr);
        
        // Append enriched entry to CSV
        if (!ota_filename.empty()) {
          try {
            std::ofstream out(ota_filename, std::ios::app);
            if (out.is_open()) {
              CSVWriter<std::ofstream> writer(out);
              writer << std::vector<std::string>{std::to_string(ota_alias.radio_id), ota_alias.alias, ota_alias.source, std::to_string(existing_ota->timestamp), ota_alias.wacn, ota_alias.sys, (ota_alias.talkgroup_id == -1) ? "" : std::to_string(ota_alias.talkgroup_id)};
              out.close();
            } else {
              BOOST_LOG_TRIVIAL(error) << "Failed to open " << ota_filename << " for appending enriched entry";
            }
          } catch (std::exception &e) {
            BOOST_LOG_TRIVIAL(error) << "Error appending enriched OTA entry: " << e.what();
          }
        }
        return false;
      }
      BOOST_LOG_TRIVIAL(debug) << "Unit " << ota_alias.radio_id << " has existing OTA alias: '" << ota_alias.alias << "', skipping";
      return false;
    }
    BOOST_LOG_TRIVIAL(info) << "Unit " << ota_alias.radio_id << " OTA alias updated: '" << existing_ota->alias << "' -> '" << ota_alias.alias << "'";
  }
  
  UnitTagOTA *ota_tag = new UnitTagOTA(ota_alias.radio_id, ota_alias.alias, ota_alias.source, ota_alias.wacn, ota_alias.sys, ota_alias.talkgroup_id, std::time(nullptr));
  unit_tags_ota.push_back(ota_tag);

  // Write to OTA file if configured
  if (!ota_filename.empty()) {
    try {
      std::ofstream out(ota_filename, std::ios::app);
      if (out.is_open()) {
        CSVWriter<std::ofstream> writer(out);
        writer << std::vector<std::string>{std::to_string(ota_alias.radio_id), ota_alias.alias, ota_alias.source, std::to_string(ota_tag->timestamp), ota_alias.wacn, ota_alias.sys, (ota_alias.talkgroup_id == -1) ? "" : std::to_string(ota_alias.talkgroup_id)};
        out.close();
      } else {
        BOOST_LOG_TRIVIAL(error) << "Failed to open " << ota_filename << " for writing OTA alias.";
      }
    } catch (std::exception &e) {
      BOOST_LOG_TRIVIAL(error) << "Error writing to OTA file " << ota_filename << ": " << e.what();
    }
  }
  
  return true;
}

void UnitTags::set_mode(UnitTagMode mode) {
  this->mode = mode;
}

UnitTagMode UnitTags::get_mode() {
  return this->mode;
}

std::vector<UnitTag *> UnitTags::get_unit_tags() {
  return unit_tags;
}

std::vector<UnitTagOTA *> UnitTags::get_unit_tags_ota() {
  return unit_tags_ota;
}
