#ifndef SETUP_SYSTEMS_H
#define SETUP_SYSTEMS_H

#include <stdlib.h>

#include <gnuradio/top_block.h>

#include "./global_structs.h"
#include "call.h"
#include "call_conventional.h"
#include "config.h"
#include "source.h"
#include "systems/p25_trunking.h"
//#include "systems/smartnet_trunking.h"
#include "systems/smartnet_impl.h"
#include "systems/system.h"

bool setup_conventional_channel(System *system, double frequency, long channel_index, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<Call *> &calls);
bool setup_conventional_system(System *system, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<Call *> &calls);
bool setup_systems(Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<System *> &systems, std::vector<Call *> &calls);

#endif