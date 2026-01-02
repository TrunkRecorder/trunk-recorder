#ifndef SYSTEM_H
#define SYSTEM_H
#include "../talkgroups.h"
#include "../unit_tags.h"
#include <boost/foreach.hpp>
#include <boost/log/trivial.hpp>
#include <gnuradio/msg_queue.h>
#include <stdio.h>
//#include "../source.h"
#include "parser.h"
#include <iomanip>

#ifdef __GNUC__
#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wint-in-bool-context"
//#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <boost/property_tree/ptree.hpp>

class Source;
class analog_recorder;
class p25_recorder;
class dmr_recorder;
class sigmf_recorder;
enum TalkgroupDisplayFormat { talkGroupDisplayFormat_id = 0,
                              talkGroupDisplayFormat_id_tag = 1,
                              talkGroupDisplayFormat_tag_id = 2 };

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

class System {

public:
  static System *make(int sys_id);
  virtual std::string get_short_name() = 0;
  virtual void set_short_name(std::string short_name) = 0;
  virtual std::string get_upload_script() = 0;
  virtual void set_upload_script(std::string script) = 0;
  virtual bool get_compress_wav() = 0;
  virtual void set_compress_wav(bool compress) = 0;
  virtual std::string get_api_key() = 0;
  virtual void set_api_key(std::string api_key) = 0;
  virtual std::string get_bcfy_api_key() = 0;
  virtual void set_bcfy_api_key(std::string bcfy_api_key) = 0;
  virtual int get_bcfy_system_id() = 0;
  virtual void set_bcfy_system_id(int bcfy_system_id) = 0;
  virtual double get_min_duration() = 0;
  virtual void set_min_duration(double duration) = 0;
  virtual double get_max_duration() = 0;
  virtual void set_max_duration(double duration) = 0;
  virtual double get_min_tx_duration() = 0;
  virtual void set_min_tx_duration(double duration) = 0;
  virtual bool get_audio_archive() = 0;
  virtual void set_audio_archive(bool) = 0;
  virtual bool get_transmission_archive() = 0;
  virtual void set_transmission_archive(bool) = 0;
  virtual bool get_record_unknown() = 0;
  virtual void set_record_unknown(bool) = 0;
  virtual bool get_call_log() = 0;
  virtual void set_call_log(bool) = 0;
  virtual bool get_conversation_mode() = 0;
  virtual void set_conversation_mode(bool mode) = 0;
  virtual void set_mdc_enabled(bool b) = 0;
  virtual void set_fsync_enabled(bool b) = 0;
  virtual void set_star_enabled(bool b) = 0;
  virtual void set_tps_enabled(bool b) = 0;

  virtual bool get_mdc_enabled() = 0;
  virtual bool get_fsync_enabled() = 0;
  virtual bool get_star_enabled() = 0;
  virtual bool get_tps_enabled() = 0;

  virtual void set_analog_levels(double r) = 0;
  virtual double get_analog_levels() = 0;
  virtual void set_digital_levels(double r) = 0;
  virtual double get_digital_levels() = 0;
  virtual void set_qpsk_mod(bool m) = 0;
  virtual bool get_qpsk_mod() = 0;
  virtual void set_squelch_db(double s) = 0;
  virtual double get_squelch_db() = 0;
  virtual void set_tau(float tau) = 0;
  virtual float get_tau() const = 0;
  virtual void set_max_dev(int max_dev) = 0;
  virtual int get_max_dev() = 0;
  virtual void set_filter_width(double f) = 0;
  virtual double get_filter_width() = 0;
  virtual gr::msg_queue::sptr get_msg_queue() = 0;
  virtual std::string get_system_type() = 0;
  virtual unsigned long get_sys_id() = 0;
  virtual unsigned long get_wacn() = 0;
  virtual unsigned long get_nac() = 0;
  virtual int get_sys_rfss() = 0;
  virtual int get_sys_site_id() = 0;
  virtual void set_xor_mask(unsigned long sys_id, unsigned long wacn, unsigned long nac) = 0;
  virtual const char *get_xor_mask() = 0;
  virtual bool update_status(TrunkMessage message) = 0;
  virtual bool update_sysid(TrunkMessage message) = 0;
  virtual int get_sys_num() = 0;
  virtual void set_system_type(std::string) = 0;
  virtual std::string get_talkgroups_file() = 0;
  virtual std::string get_unit_tags_file() = 0;
  virtual Source *get_source() = 0;
  virtual void set_source(Source *) = 0;
  virtual Talkgroup *find_talkgroup(long tg) = 0;
  virtual Talkgroup *find_talkgroup_by_freq(double freq) = 0;
  virtual std::string find_unit_tag(long unitID) = 0;
  virtual void set_talkgroups_file(std::string) = 0;
  virtual void set_channel_file(std::string channel_file) = 0;
  virtual bool has_channel_file() = 0;
  virtual void set_unit_tags_file(std::string) = 0;
  virtual void set_unit_tags_ota_file(std::string) = 0;
  virtual std::string get_unit_tags_ota_file() = 0;
  virtual void set_unit_tags_mode(std::string mode) = 0;
  virtual std::string get_unit_tags_mode() = 0;
  virtual void set_custom_freq_table_file(std::string custom_freq_table_file) = 0;
  virtual std::string get_custom_freq_table_file() = 0;
  virtual bool has_custom_freq_table_file() = 0;
  virtual int control_channel_count() = 0;
  virtual void add_control_channel(double channel) = 0;
  virtual double get_next_control_channel() = 0;
  virtual double get_current_control_channel() = 0;
  virtual int channel_count() = 0;
  virtual int get_message_count() = 0;
  virtual void set_message_count(int count) = 0;
  virtual void set_decode_rate(int rate) = 0;
  virtual int get_decode_rate() = 0;
  virtual void add_channel(double channel) = 0;
  virtual void add_conventional_recorder(analog_recorder_sptr rec) = 0;
  virtual void add_conventionalSIGMF_recorder(sigmf_recorder_sptr rec) = 0;
  virtual void add_conventionalP25_recorder(p25_recorder_sptr rec) = 0;
  virtual void add_conventionalDMR_recorder(dmr_recorder_sptr rec) = 0;
  virtual std::vector<analog_recorder_sptr> get_conventional_recorders() = 0;
  virtual std::vector<sigmf_recorder_sptr> get_conventionalSIGMF_recorders() = 0;
  virtual std::vector<p25_recorder_sptr> get_conventionalP25_recorders() = 0;
  virtual std::vector<dmr_recorder_sptr> get_conventionalDMR_recorders() = 0;
  virtual std::vector<double> get_channels() = 0;
  virtual std::vector<double> get_control_channels() = 0;
  virtual std::vector<Talkgroup *> get_talkgroups() = 0;
  virtual std::vector<UnitTag *> get_unit_tags() = 0;
  virtual std::vector<UnitTagOTA *> get_unit_tags_ota() = 0;
  virtual void set_bandplan(std::string) = 0;
  virtual std::string get_bandplan() = 0;
  virtual void set_bandfreq(int) = 0;
  virtual int get_bandfreq() = 0;
  virtual void set_bandplan_base(double) = 0;
  virtual double get_bandplan_base() = 0;
  virtual void set_bandplan_high(double high) = 0;
  virtual double get_bandplan_high() = 0;
  virtual void set_bandplan_spacing(double) = 0;
  virtual double get_bandplan_spacing() = 0;
  virtual void set_bandplan_offset(int) = 0;
  virtual int get_bandplan_offset() = 0;
  virtual void set_talkgroup_display_format(TalkgroupDisplayFormat format) = 0;
  virtual TalkgroupDisplayFormat get_talkgroup_display_format() = 0;

  virtual bool get_hideEncrypted() = 0;
  virtual void set_hideEncrypted(bool hideEncrypted) = 0;
  virtual bool get_monitorEncrypted() = 0;
  virtual void set_monitorEncrypted(bool monitorEncrypted) = 0;

  virtual bool get_hideUnknown() = 0;
  virtual void set_hideUnknown(bool hideUnknown) = 0;

  virtual int get_freq_error() = 0;
  virtual void finetune_control_freq(double f) = 0;
  virtual int get_autotune_offset() = 0;
  virtual void set_autotune_offset(int offset) = 0;

  virtual boost::property_tree::ptree get_stats() = 0;
  virtual boost::property_tree::ptree get_stats_current(float timeDiff) = 0;

  virtual std::vector<unsigned long> get_talkgroup_patch(unsigned long talkgroup) = 0;
  virtual void update_active_talkgroup_patches(PatchData f_data) = 0;
  virtual void delete_talkgroup_patch(PatchData f_data) = 0;
  virtual void clear_stale_talkgroup_patches() = 0;
  virtual void print_active_talkgroup_patches() = 0;

  virtual bool get_multiSite() = 0;
  virtual void set_multiSite(bool multiSite) = 0;

  virtual std::string get_multiSiteSystemName() = 0;
  virtual void set_multiSiteSystemName(std::string multiSiteSystemName) = 0;

  virtual unsigned long get_multiSiteSystemNumber() = 0;
  virtual void set_multiSiteSystemNumber(unsigned long multiSiteSystemName) = 0;

};
#endif
