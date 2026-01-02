#ifndef SYSTEM_IMPL_H
#define SYSTEM_IMPL_H
#include "../talkgroups.h"
#include "../unit_tags.h"
#include <boost/foreach.hpp>
#include <boost/log/trivial.hpp>
#include <stdio.h>
//#include "../source.h"
#include "p25_trunking.h"
#include "parser.h"
#include "smartnet_trunking.h"
#include "system.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wint-in-bool-context"
//#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif

#include <lfsr/lfsr.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <boost/property_tree/ptree.hpp>

class Source;
class analog_recorder;
class p25_recorder;
class dmr_recorder;
class sigmf_recorder;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<analog_recorder> analog_recorder_sptr;
typedef boost::shared_ptr<p25_recorder> p25_recorder_sptr;
typedef boost::shared_ptr<dmr_recorder> dmr_recorder_sptr;
typedef boost::shared_ptr<sigmf_recorder> sigmf_recorder_sptr;
#else
typedef std::shared_ptr<analog_recorder> analog_recorder_sptr;
typedef std::shared_ptr<p25_recorder> p25_recorder_sptr;
typedef std::shared_ptr<dmr_recorder> dmr_recorder_sptr;
typedef std::shared_ptr<sigmf_recorder> sigmf_recorder_sptr;
#endif

class System_impl : public System {
  int sys_num;
  unsigned long sys_id;
  unsigned long wacn;
  unsigned long nac;
  int sys_rfss;
  int sys_site_id;

public:
  Talkgroups *talkgroups;
  UnitTags *unit_tags;
  p25p2_lfsr *lfsr;
  Source *source;
  std::string talkgroups_file;
  std::string channel_file;
  std::string unit_tags_file;
  std::string unit_tags_ota_file;
  std::string unit_tags_mode;
  std::string custom_freq_table_file;
  std::string short_name;
  std::string api_key;
  std::string bcfy_api_key;
  std::string default_mode;
  std::string system_type;
  std::string upload_script;
  int bcfy_system_id;
  int message_count;
  int decode_rate;
  int retune_attempts;
  time_t last_message_time;
  std::string bandplan;
  int bandfreq;
  double bandplan_base;
  double bandplan_high;
  double bandplan_spacing;
  int bandplan_offset;
  int max_dev;
  double filter_width;
  double min_call_duration;
  double max_call_duration;
  double min_transmission_duration;
  bool compress_wav;
  bool conversation_mode;
  bool qpsk_mod;
  double squelch_db;
  float tau;
  double analog_levels;
  double digital_levels;

  unsigned xor_mask_len;
  const char *xor_mask;
  std::vector<double> control_channels;
  unsigned int current_control_channel;
  std::vector<double> channels;
  std::vector<analog_recorder_sptr> conventional_recorders;
  std::vector<p25_recorder_sptr> conventionalP25_recorders;
  std::vector<dmr_recorder_sptr> conventionalDMR_recorders;
  std::vector<sigmf_recorder_sptr> conventionalSIGMF_recorders;
  bool transmission_archive;
  bool audio_archive;
  bool record_unknown;
  bool call_log;

  smartnet_trunking_sptr smartnet_trunking;
  p25_trunking_sptr p25_trunking;

  std::map<unsigned long, std::map<unsigned long, std::time_t>> talkgroup_patches;

  std::string get_short_name() override;
  void set_short_name(std::string short_name) override;
  std::string get_upload_script() override;
  void set_upload_script(std::string script) override;
  bool get_compress_wav() override;
  void set_compress_wav(bool compress) override;
  std::string get_api_key() override;
  void set_api_key(std::string api_key) override;
  std::string get_bcfy_api_key() override;
  void set_bcfy_api_key(std::string bcfy_api_key) override;
  int get_bcfy_system_id() override;
  void set_bcfy_system_id(int bcfy_system_id) override;
  double get_min_duration() override;
  void set_min_duration(double duration) override;
  double get_max_duration() override;
  void set_max_duration(double duration) override;
  double get_min_tx_duration() override;
  void set_min_tx_duration(double duration) override;
  bool get_audio_archive() override;
  void set_audio_archive(bool) override;
  bool get_transmission_archive() override;
  void set_transmission_archive(bool) override;
  bool get_record_unknown() override;
  void set_record_unknown(bool) override;
  bool get_call_log() override;
  void set_call_log(bool) override;
  bool get_conversation_mode() override;
  void set_conversation_mode(bool mode) override;
  void set_mdc_enabled(bool b) override;
  void set_fsync_enabled(bool b) override;
  void set_star_enabled(bool b) override;
  void set_tps_enabled(bool b) override;

  bool get_mdc_enabled() override;
  bool get_fsync_enabled() override;
  bool get_star_enabled() override;
  bool get_tps_enabled() override;

  void set_analog_levels(double r) override;
  double get_analog_levels() override;
  void set_digital_levels(double r) override;
  double get_digital_levels() override;
  void set_qpsk_mod(bool m) override;
  bool get_qpsk_mod() override;
  void set_squelch_db(double s) override;
  double get_squelch_db() override;
  void set_tau(float tau) override;
  float get_tau() const override;
  void set_max_dev(int max_dev) override;
  int get_max_dev() override;
  void set_filter_width(double f) override;
  double get_filter_width() override;
  gr::msg_queue::sptr get_msg_queue() override;
  std::string get_system_type() override;
  unsigned long get_sys_id() override;
  unsigned long get_wacn() override;
  unsigned long get_nac() override;
  int get_sys_rfss() override;
  int get_sys_site_id() override;
  void set_xor_mask(unsigned long sys_id, unsigned long wacn, unsigned long nac) override;
  const char *get_xor_mask() override;
  bool update_status(TrunkMessage message) override;
  bool update_sysid(TrunkMessage message) override;
  int get_sys_num() override;
  void set_system_type(std::string) override;
  std::string get_talkgroups_file() override;
  std::string get_unit_tags_file() override;
  Source *get_source() override;
  void set_source(Source *) override;
  Talkgroup *find_talkgroup(long tg) override;
  Talkgroup *find_talkgroup_by_freq(double freq) override;
  std::string find_unit_tag(long unitID) override;
  void set_talkgroups_file(std::string) override;
  void set_channel_file(std::string channel_file) override;
  bool has_channel_file() override;
  void set_unit_tags_file(std::string) override;
  void set_unit_tags_ota_file(std::string) override;
  std::string get_unit_tags_ota_file() override;
  void set_unit_tags_mode(std::string mode) override;
  std::string get_unit_tags_mode() override;
  void set_custom_freq_table_file(std::string custom_freq_table_file) override;
  std::string get_custom_freq_table_file() override;
  bool has_custom_freq_table_file() override;
  int control_channel_count() override;
  int get_message_count() override;
  void set_message_count(int count) override;
  int get_decode_rate() override;
  void set_decode_rate(int rate) override;
  void add_control_channel(double channel) override;
  double get_next_control_channel() override;
  double get_current_control_channel() override;
  int channel_count() override;
  void add_channel(double channel) override;
  void add_conventional_recorder(analog_recorder_sptr rec) override;
  void add_conventionalP25_recorder(p25_recorder_sptr rec) override;
  void add_conventionalSIGMF_recorder(sigmf_recorder_sptr rec) override;
  void add_conventionalDMR_recorder(dmr_recorder_sptr rec) override;
  std::vector<p25_recorder_sptr> get_conventionalP25_recorders() override;
  std::vector<analog_recorder_sptr> get_conventional_recorders() override;
  std::vector<sigmf_recorder_sptr> get_conventionalSIGMF_recorders() override;
  std::vector<dmr_recorder_sptr> get_conventionalDMR_recorders() override;
  std::vector<double> get_channels() override;
  std::vector<double> get_control_channels() override;
  std::vector<Talkgroup *> get_talkgroups() override;
  std::vector<UnitTag *> get_unit_tags() override;
  std::vector<UnitTagOTA *> get_unit_tags_ota() override;
  gr::msg_queue::sptr msg_queue;
  System_impl(int sys_id);
  void set_bandplan(std::string) override;
  std::string get_bandplan() override;
  void set_bandfreq(int) override;
  int get_bandfreq() override;
  void set_bandplan_base(double) override;
  double get_bandplan_base() override;
  void set_bandplan_high(double high) override;
  double get_bandplan_high() override;
  void set_bandplan_spacing(double) override;
  double get_bandplan_spacing() override;
  void set_bandplan_offset(int) override;
  int get_bandplan_offset() override;
  void set_talkgroup_display_format(TalkgroupDisplayFormat format) override;
  TalkgroupDisplayFormat get_talkgroup_display_format() override;

  bool get_hideEncrypted() override;
  void set_hideEncrypted(bool hideEncrypted) override;
  bool get_monitorEncrypted() override;
  void set_monitorEncrypted(bool monitorEncrypted) override;

  bool get_hideUnknown() override;
  void set_hideUnknown(bool hideUnknown) override;

  int get_freq_error() override;
  void finetune_control_freq(double f) override;
  int get_autotune_offset() override;
  void set_autotune_offset(int offset) override;

  boost::property_tree::ptree get_stats() override;
  boost::property_tree::ptree get_stats_current(float timeDiff) override;

  std::vector<unsigned long> get_talkgroup_patch(unsigned long talkgroup) override;
  void update_active_talkgroup_patches(PatchData f_data) override;
  void delete_talkgroup_patch(PatchData f_data) override;
  void clear_stale_talkgroup_patches() override;
  void print_active_talkgroup_patches() override;
  bool get_multiSite() override;
  void set_multiSite(bool multiSite) override;

  std::string get_multiSiteSystemName() override;
  void set_multiSiteSystemName(std::string multiSiteSystemName) override;

  unsigned long get_multiSiteSystemNumber() override;
  void set_multiSiteSystemNumber(unsigned long multiSiteSystemNumber) override;

private:
  TalkgroupDisplayFormat talkgroup_display_format;
  bool d_hideEncrypted;
  bool d_monitorEncrypted;
  bool d_hideUnknown;
  bool d_multiSite;
  std::string d_multiSiteSystemName;
  unsigned long d_multiSiteSystemNumber;

  bool d_mdc_enabled;
  bool d_fsync_enabled;
  bool d_star_enabled;
  bool d_tps_enabled;
};
#endif
