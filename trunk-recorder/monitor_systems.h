#ifndef MONITOR_SYSTEMS_H
#define MONITOR_SYSTEMS_H
#include <signal.h>
#include <stdlib.h>

#include "./global_structs.h"
#include "call.h"
#include "config.h"
#include "source.h"
#include "systems/p25_parser.h"
#include "systems/p25_trunking.h"
#include "systems/smartnet_parser.h"
#include "systems/smartnet_trunking.h"
#include "systems/system.h"
#include <gnuradio/top_block.h>

int monitor_messages(Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<System *> &systems, std::vector<Call *> &calls);
void check_sources(std::vector<Source *> &sources);
void retune_system(System *sys, gr::top_block_sptr &tb, std::vector<Source *> &sources);
#endif