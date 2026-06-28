#ifndef DMR_RECORDER_H
#define DMR_RECORDER_H

#define _USE_MATH_DEFINES

#include <cstdio>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../source.h"
#include "recorder.h"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>

class Source;
class dmr_recorder;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<dmr_recorder> dmr_recorder_sptr;
#else
typedef std::shared_ptr<dmr_recorder> dmr_recorder_sptr;
#endif

dmr_recorder_sptr make_dmr_recorder(Source *src, bool conventional);

// Single DMR recorder that hosts both TDMA slots of one RF channel. The xlat
// channelizer, FSK4 demod chain, and OP25 frame_assembler are shared; each slot
// has its own transmission_sink and its own Call*. Slots are started and
// stopped independently, so one slot can record while the other is idle.
// Conventional and trunked DMR use the same class — the difference is whether
// the recorder is dedicated to a fixed channel (conventional) or gets retuned
// to a grant freq (trunked). That distinction is carried by the `conventional`
// flag on the base Recorder.
class dmr_recorder : virtual public gr::hier_block2, virtual public Recorder {
public:
  dmr_recorder() {}
  virtual ~dmr_recorder() {}

  // Tune the channelizer to a new RF freq. Only safe when both slots are idle;
  // callers (the Source allocator) are responsible for that invariant.
  virtual void tune_freq(double f) = 0;
  virtual double get_freq() = 0;
  virtual int get_freq_error() = 0;
  virtual int get_num() = 0;
  virtual Source *get_source() = 0;

  // Per-slot lifecycle. start() reads call->get_tdma_slot() to pick the slot.
  virtual bool start(Call *call) = 0;
  virtual void stop(int slot) = 0;
  virtual void stop() = 0;  // stops both slots; only used on shutdown

  // Per-slot queries (read by Call_impl via the call's tdma_slot).
  virtual State get_state(int slot) = 0;
  virtual bool is_active(int slot) = 0;
  virtual bool is_idle(int slot) = 0;
  virtual double since_last_write(int slot) = 0;
  virtual double get_current_length(int slot) = 0;
  virtual std::vector<Transmission> get_transmission_list(int slot) = 0;

  // Bring the slot-less aggregates from Recorder back into scope — the slot
  // overloads above would otherwise hide them, so callers holding a
  // dmr_recorder_sptr could only call get_state(int). The aggregates are used
  // by reporting code (print_recorders, get_stats) that doesn't know a slot.
  using Recorder::get_state;
  using Recorder::is_active;
  using Recorder::is_idle;
  using Recorder::since_last_write;
  using Recorder::get_current_length;
  using Recorder::get_transmission_list;
  using Recorder::stop;

  // Allocator helpers. is_slot_available is the "attach a new Call here" check;
  // is_fully_available is the "can I retune this recorder to a new freq" check.
  virtual bool is_slot_available(int slot) = 0;
  virtual bool is_fully_available() = 0;

  // Recorder pipeline (shared across slots).
  virtual void set_enabled(bool enabled) {}
  virtual bool is_enabled() { return false; }
  virtual bool is_squelched() = 0;
  virtual double get_pwr() = 0;
  virtual int lastupdate() = 0;
  virtual long elapsed() = 0;
};

#endif // DMR_RECORDER_H
