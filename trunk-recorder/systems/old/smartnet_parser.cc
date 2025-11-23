#include "smartnet_parser.h"
#include "../formatter.h"

using namespace std;
SmartnetParser::SmartnetParser() {
  lastaddress = 0;
  lastcmd = 0;
  numStacked = 0;
  numConsumed = 3; // "preload" the stack to where the scrubber head is before starting to parse
}

double SmartnetParser::getfreq(int cmd, System *sys) {
  double freq = 0.0;
  std::string band = sys->get_bandplan();
  if (sys->get_bandfreq() == 800) {
    /*
      BANDPLAN 800Mhz:
      800_standard * Is default base plan
      800_splinter
      800_reband
    */
    if (cmd < 0 || cmd > 0x3FE)
      return freq;
    if (cmd <= 0x2CF) {
      if (band == "800_reband" && cmd >= 0x1B8 &&
          cmd <= 0x22F) { /* Re Banded Site */
        freq = 851.0250 + (0.025 * ((double)(cmd - 0x1B8)));
      } else if (band == "800_splinter" && cmd <= 0x257) { /* Splinter Site */
        freq = 851.0 + (0.025 * ((double)cmd));
      } else {
        freq = 851.0125 + (0.025 * ((double)cmd));
      }
    } else if (cmd <= 0x2f7) {
      freq = 866.0000 + (0.025 * ((double)(cmd - 0x2D0)));
    } else if (cmd >= 0x32F && cmd <= 0x33F) {
      freq = 867.0000 + (0.025 * ((double)(cmd - 0x32F)));
    } else if (cmd == 0x3BE) {
      freq = 868.9750;
    } else if (cmd >= 0x3C1 && cmd <= 0x3FE) {
      freq = 867.4250 + (0.025 * ((double)(cmd - 0x3C1)));
    }
  } else if (sys->get_bandfreq() == 400) {
    /*double test_freq;
    //Step * (channel - low) + Base
    if ((cmd >= 0x17c) && (cmd < 0x2b0)) {
      test_freq = ((cmd - 380) * 25000)  + 489087500;
    } else {
      test_freq = 0;
    }*/

    double high_cmd = sys->get_bandplan_offset() +
                      (sys->get_bandplan_high() - sys->get_bandplan_base()) /
                          sys->get_bandplan_spacing();

    if ((cmd >= sys->get_bandplan_offset()) &&
        (cmd < high_cmd)) { // (cmd <= sys->get_bandplan_base() +
                            // sys->get_bandplan_offset() )) {
      freq = sys->get_bandplan_base() +
             (sys->get_bandplan_spacing() * (cmd - sys->get_bandplan_offset()));
    }
   //cout << "Orig: " <<fixed << " " << cmd << " Freq: " << freq << endl;
  }
  return freq * 1000000;
}

bool SmartnetParser::is_chan_outbound(int cmd, System *sys) {
  if (sys->get_bandfreq() == 800) {
    if ((cmd >= OSW_CHAN_BAND_1_MIN && cmd <= OSW_CHAN_BAND_1_MAX) ||
        (cmd >= OSW_CHAN_BAND_2_MIN && cmd <= OSW_CHAN_BAND_2_MAX) ||
        (cmd == OSW_CHAN_BAND_3) ||
        (cmd >= OSW_CHAN_BAND_4_MIN && cmd <= OSW_CHAN_BAND_4_MAX)) {
      return true;
    }
  } else if (sys->get_bandfreq() == 400) {
    //
    if (cmd >= sys->get_bandplan_offset() &&
        cmd < sys->get_bandplan_offset() + 380) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool SmartnetParser::is_chan_inbound_obt(int cmd, System *sys) {
  return cmd < sys->get_bandplan_offset();
}

bool SmartnetParser::is_first_normal(int cmd, System *sys) {
  if (sys->get_bandfreq() == 800) {
    // anything "800" should be replaced with 8/9 compatible switching
    return ((cmd == OSW_FIRST_NORMAL) ||
            (cmd == OSW_FIRST_ASTRO));
  } else {
    // if we're looking at an OBT trunk, inbound channel commands are first normals too
    // anything "400" should be replaced as "OBT" in the future =/
    return (is_chan_inbound_obt(cmd, sys) ||
            (cmd == OSW_FIRST_NORMAL) ||
            (cmd == OSW_FIRST_ASTRO));
  }
}

std::vector<TrunkMessage> SmartnetParser::parse_message(std::string s,
                                                        System *system) {
  std::vector<TrunkMessage> messages;
  TrunkMessage message;

  // char tempArea[512];
  // unsigned short blockNum;
  // char banktype;
  // unsigned short tt1, tt2;
  // static unsigned int ott1, ott2;

  // print_osw(s);
  message.message_type = UNKNOWN;
  message.encrypted = false;
  message.phase2_tdma = false;
  message.tdma_slot = 0;
  message.source = -1;
  message.sys_id = 0;
  message.sys_num = system->get_sys_num();
  message.emergency = false;
  message.opcode = 0;

  std::vector<std::string> x;
  boost::split(x, s, boost::is_any_of(","), boost::token_compress_on);

  if (x.size() < 3) {
    BOOST_LOG_TRIVIAL(error)
        << "SmartNet Parser recieved invalid message." << x.size();
    return messages;
  }

  int full_address = atoi(x[0].c_str());
  int status = full_address & 0x000F;
  long address = full_address & 0xFFF0;
  int groupflag = atoi(x[1].c_str());
  int command = atoi(x[2].c_str());

  struct osw_stru bosw;
  bosw.id = address;
  bosw.full_address = full_address;
  bosw.address = address;
  bosw.status = status;
  bosw.grp = groupflag;
  bosw.cmd = command;

  // struct osw_stru* Inposw = &bosw;
  cout.precision(0);

  // maintain a sliding stack of 5 OSWs. If previous iteration used more than
  // one, don't utilize stack until all used ones have slid past.

  switch (numStacked) // note: drop-thru is intentional!
  {
  case 5:
  case 4:
    stack[4] = stack[3];

  case 3:
    stack[3] = stack[2];

  case 2:
    stack[2] = stack[1];

  case 1:
    stack[1] = stack[0];

  case 0:
    stack[0] = bosw;
    break;

  default:
    BOOST_LOG_TRIVIAL(info) << "corrupt value for nstacked" << endl;
    break;
  }

  if (numStacked < 5) {
    ++numStacked;
  }

  // Rename this in the future to "additional shift" or "overconsumed"
  // or something. For now this will do though.
  // Theoretically will never be more than 1, but maybe we'll want to add
  // erroring in the future to cover impossible situations.
  if (numConsumed > 0) {
    --numConsumed;
    messages.push_back(message);
    return messages;
  }

  x.clear();
  vector<string>().swap(x);

  // raw OSW stream
   //BOOST_LOG_TRIVIAL(info)
   //    << "[" << system->get_short_name()
   //    << "] [OSW!] [[["<< std::hex << stack[0].cmd << " " << std::hex << stack[0].grp << " " << std::hex << stack[0].full_address << "]]]";

  // Message parsing strategy
  // OSW stack:      [0  1  2  3  4] (consume) - consume is how many OSWs to consume. This includes the 1-OSW regular increment.
  //                           ^
  // Direction:       Tail --> Head
  // Order of tests: [0  1  2  3  4] (consume) - consume is how many OSWs to consume. This includes the 1-OSW regular increment.
  //                                 (1) test if [3] is a 1-OSW message, such as:
  //                          [3]            1-OSW message dynamic command: call continue
  //                          [3]            1-OSW message static  command: IDLE, RfA, etc.
  //                          [3]            1-OSW message dynamic command: AMSS site announcement
  //                    [1  2  3]    (n) test if [3] is an OSW with a FIRST_NORMAL or STATUS, both of which can be variable in length.
  //                    [1  2  3]            If [3] is a FIRST_NORMAL or STATUS, ++numConsumed.
  //                    [1  2  3]            If [2] is SECOND, ++numConsumed, and see if [1] is an EXTENDED_FCN.
  //                    [1  2  3]                If [1] is an EXTENDED_FCN, then we have a valid 3-OSW message.
  //                    [1  2  3]                If not, then we have a malformed 3-OSW message. Process accordingly.
  //                    [1  2  3]            If [2] is not SECOND, then see if it's an EXTENDED.
  //                    [1  2  3]                If it is, then we have a valid 2-OSW message. Process accordingly.
  //                    [1  2  3]                If not, then we have a malformed 2-OSW message.
  // An OSW is 32 bits - round that up to 36 bits, 3600 b/s CC -> 100 OSWs/s -> 10 ms/OSW.
  // Even though we're now looking 2 OSWs later than the previous parser, that's only a 20 ms hit, 40 ms past the system.
  // If we wish to eek 10 ms better performance, we can shift over 1 and test [0 1 2] instead of [1 2 3] instead.
  // If we wish to eek 20 ms on top of that, we could check call continues on [0] and do weird message history backfilling,
  // but this really isn't worth it.

  // 1-OSW message: dynamic - call continue
  // is_chan_outbound(stack[3].cmd) returns if a command is a valid outbound channel indicator
  //                       (taking into consideration band and bandplan base/high/spacing/offset).
  // If stack[3].cmd is a valid outbound word, then we have a valid call continue.
  if (is_chan_outbound(stack[3].cmd, system)) {
    // this is a call continue
    if (stack[3].grp) {
      // this is a group call continue
       BOOST_LOG_TRIVIAL(trace)
           << "[" << system->get_short_name() << "] [group call continue] [ "
           << std::hex << stack[0].cmd << " " << std::hex << stack[0].grp << " " << std::dec << stack[0].full_address << " " << stack[0].address << " |  "
           << std::hex << stack[1].cmd << " " << std::hex << stack[1].grp << " " << std::dec << stack[1].full_address << " " << stack[1].address << "  |  "
           << std::hex << stack[2].cmd << " " << std::hex << stack[2].grp << " " << std::dec << stack[2].full_address << " " << stack[2].address << "  | >"
           <<  getfreq(stack[3].cmd, system) << " " << std::hex << stack[3].grp << " " << std::dec << stack[3].full_address << " " << stack[3].address << "< |  "
           << std::hex << stack[4].cmd << " " << std::hex << stack[4].grp << " " << std::dec << stack[4].full_address << " " << stack[4].address << " ]";
      message.message_type = UPDATE;
      message.freq = getfreq(stack[3].cmd, system);
      message.talkgroup = stack[3].address;
      if (stack[3].status >= 8) {
        message.encrypted = true;
        if ((stack[3].status == 10) || (stack[3].status == 12) || (stack[3].status == 13)) {
          message.emergency = true;
        }
      }
      if ((stack[3].status == 2) || (stack[3].status == 4) || (stack[3].status == 5)) {
        message.emergency = true;
      }
      messages.push_back(message);
      return messages;
    } else {
      // this is an individual call continue
       BOOST_LOG_TRIVIAL(trace)
           << "[" << system->get_short_name() << "] [individual call continue] [ "
           << std::hex << stack[0].cmd << " " << std::hex << stack[0].grp << " " << std::hex << stack[0].full_address << "  |  "
           << std::hex << stack[1].cmd << " " << std::hex << stack[1].grp << " " << std::hex << stack[1].full_address << "  |  "
           << std::hex << stack[2].cmd << " " << std::hex << stack[2].grp << " " << std::hex << stack[2].full_address << "  | >"
           << std::hex << stack[3].cmd << " " << std::hex << stack[3].grp << " " << std::hex << stack[3].full_address << "< |  "
           << std::hex << stack[4].cmd << " " << std::hex << stack[4].grp << " " << std::hex << stack[4].full_address << " ]";
      message.message_type = UNKNOWN;
      messages.push_back(message);
      return messages;
    }
  }

  // 1-OSW message: static
  // Is there any useful information in the group/full address of an idle 1-OSW message?
  if (stack[3].cmd == OSW_BACKGROUND_IDLE) {
    message.message_type = UNKNOWN;
    messages.push_back(message);
    return messages;
  }
  if (stack[3].cmd == 0x32a) {
    // System sent request for affiliation from radio full_address
    message.message_type = UNKNOWN;
    messages.push_back(message);
    return messages;
  }

  // 1-OSW message: dynamic - AMSS/SmartZone site # announcement
  // There is also potentially a site name that would need to be cobbled together
  // over multiple runs of parse_message since 3 unique ones are needed.
  if ((OSW_AMSS_ID_MIN <= stack[3].cmd) && (stack[3].cmd <= OSW_AMSS_ID_MAX)) {
    // BOOST_LOG_TRIVIAL(warning)
    //     << "[" << system->get_short_name() << "] [Type II AMSS/SmartZone Site Announcement] Site $"
    //     << std::hex << stack[3].cmd - OSW_AMSS_ID_MIN + 1;
    message.message_type = UNKNOWN;
    messages.push_back(message);
    return messages;
  }

  // n-OSW messages (which must have a known static head)
  // Net/System status messages can be delivered standalone or paired with
  // an extended function individual OSW. This seems to happen right after
  // a radio joins up (so that the radio immediately gets status information).
  // Since we know about this case, consume that additional OSW.
  // No handling required though since that 2-OSW message already comes with
  // the header being a group OSW.
  // There's also the possibility that Net/Status messages can be 3-OSW.
  // We don't know about this, so we'll catch them by failing through to the end.
  if ((stack[3].cmd == OSW_SYS_NETSTAT) || (stack[3].cmd == OSW_SYS_STATUS)) {
    if (stack[2].cmd == OSW_SECOND_NORMAL) {
      ++numConsumed;
      if (stack[1].cmd == OSW_EXTENDED_FCN) {
        // we have a 3-OSW message.

        // if we don't have a valid 3-OSW message but the second OSW
        // is still a OSW_SECOND_NORMAL, then we want to know about it in development.
        // we'll still consume 2-OSWs because we incremented after checking OSW_SECOND_NORMAL.
        ++numConsumed;
        message.message_type = UNKNOWN;
        messages.push_back(message);
        return messages;
      }
    }
    if (stack[2].cmd == OSW_EXTENDED_FCN) {
      // we have a 2-OSW message.
      ++numConsumed;
      message.message_type = UNKNOWN;
      messages.push_back(message);
      return messages;
    }
    // we have a 1-OSW message.
    message.message_type = UNKNOWN;
    messages.push_back(message);
    return messages;
  }

  // n-OSW messages (which must have a known static head)
  // FIRST_NORMAL
  if (is_first_normal(stack[3].cmd, system)) {
    if (is_chan_outbound(stack[2].cmd, system)) {
      // this is a call grant
      ++numConsumed;
      if (stack[2].grp) {
        // this is a group call grant
         BOOST_LOG_TRIVIAL(trace)
             << "[" << system->get_short_name() << "] [group call grant] [ "
             << std::hex << stack[0].cmd << " " << std::hex << stack[0].grp << " " << std::hex << stack[0].full_address << "  |  "
             << std::hex << stack[1].cmd << " " << std::hex << stack[1].grp << " " << std::hex << stack[1].full_address << "  | >"
             << std::hex << stack[2].cmd << " " << std::hex << stack[2].grp << " " << std::hex << stack[2].full_address << "  |  "
             << std::hex << stack[3].cmd << " " << std::hex << stack[3].grp << " " << std::hex << stack[3].full_address << "< |  "
             << std::hex << stack[4].cmd << " " << std::hex << stack[4].grp << " " << std::hex << stack[4].full_address << " ]";
        message.message_type = GRANT;
        message.freq = getfreq(stack[2].cmd, system);
        message.talkgroup = stack[2].address;
        if (stack[2].status >= 8) {
          message.encrypted = true;
          if ((stack[2].status == 10) || (stack[2].status == 12) || (stack[2].status == 13)) {
            message.emergency = true;
          }
        }
        if ((stack[2].status == 2) || (stack[2].status == 4) || (stack[2].status == 5)) {
          message.emergency = true;
        }
        message.source = stack[3].full_address;
        messages.push_back(message);
        return messages;
      } else {
        // this is an individual call grant
         BOOST_LOG_TRIVIAL(trace)
             << "[" << system->get_short_name() << "] [individual call grant] [ "
             << std::hex << stack[0].cmd << " " << std::hex << stack[0].grp << " " << std::hex << stack[0].full_address << "  |  "
             << std::hex << stack[1].cmd << " " << std::hex << stack[1].grp << " " << std::hex << stack[1].full_address << "  | >"
             << std::hex << stack[2].cmd << " " << std::hex << stack[2].grp << " " << std::hex << stack[2].full_address << "  |  "
             << std::hex << stack[3].cmd << " " << std::hex << stack[3].grp << " " << std::hex << stack[3].full_address << "< |  "
             << std::hex << stack[4].cmd << " " << std::hex << stack[4].grp << " " << std::hex << stack[4].full_address << " ]";
        message.message_type = UNKNOWN;
        messages.push_back(message);
        return messages;
      }
    }
    if (stack[2].cmd == OSW_SECOND_NORMAL) {
      ++numConsumed;
      if (stack[1].cmd == OSW_EXTENDED_FCN) {
        // valid 3-OSW message.
        // if not, then we only consume and discard 2-OSWs.
        ++numConsumed;
        message.message_type = UNKNOWN;
        messages.push_back(message);
        return messages;
      }
    }
    if (stack[2].cmd == OSW_EXTENDED_FCN) {
      // we have a 2-OSW message, process it.
      ++numConsumed;
      // we have an extended function
      // lots of possibilities here: reg, dereg, patch/msel, Sys ID, etc.
      // For now use if statements, but some sort of lookup table for function
      // type may be more useful in the future.
      if (stack[2].full_address == 0x2021) {
        // Patch Termination
        // BOOST_LOG_TRIVIAL(warning)
        //     << "[" << system->get_short_name() << "] [patch/msel termination] TG $"
        //     << std::hex << stack[3].full_address;
        message.message_type = UNKNOWN;
        message.talkgroup = stack[3].address;
        messages.push_back(message);
        return messages;
      }
      if (stack[2].full_address == 0x261b) {
        // Registration
        // BOOST_LOG_TRIVIAL(warning)
        //     << "[" << system->get_short_name() << "] [  registration] RID $"
        //     << std::hex << stack[3].full_address;
        message.message_type = REGISTRATION;
        message.source = stack[3].full_address;
        messages.push_back(message);
        return messages;
      }
      if (stack[2].full_address == 0x261c) {
        // Dereg
        // BOOST_LOG_TRIVIAL(warning)
        //     << "[" << system->get_short_name() << "] [deregistration] RID $"
        //     << std::hex << stack[3].full_address;
        message.message_type = DEREGISTRATION;
        message.source = stack[3].full_address;
        messages.push_back(message);
        return messages;
      }
      if (stack[2].full_address == 0x2c47) {
        // Busy Override Deny
        // BOOST_LOG_TRIVIAL(warning)
        //     << "[" << system->get_short_name() << "] [busy override deny] RID $"
        //     << std::hex << stack[3].full_address;
        message.message_type = UNKNOWN;
        message.source = stack[3].full_address;
        messages.push_back(message);
        return messages;
      }
      if (stack[2].full_address == 0x2c65) {
        // Access Deny
        // BOOST_LOG_TRIVIAL(warning)
        //     << "[" << system->get_short_name() << "] [deny ($2c65)] RID $"
        //     << std::hex << stack[3].full_address;
        message.message_type = UNKNOWN;
        message.source = stack[3].full_address;
        messages.push_back(message);
        return messages;
      }
      if ((0 <= (stack[2].full_address - 0x2800)) && ((stack[2].full_address - 0x2800) < 0x2f7)) {
        // System ID
        // BOOST_LOG_TRIVIAL(warning)
        //     << "[" << system->get_short_name() << "] [Sys ID] SID $"
        //     << std::hex << stack[3].full_address
        //     << ", CC $" << std::hex << stack[2].full_address - 0x2800
        //     << " -> " << getfreq(stack[2].full_address - 0x2800, system);
        message.message_type = SYSID;
        message.freq = getfreq(stack[2].full_address - 0x2800, system);
        message.sys_id = stack[3].full_address;
        messages.push_back(message);
        return messages;
      }
    }
    // further work needs to be done on patches/msels
    if (stack[2].cmd == 0x340) {
      // we have a patch
      ++numConsumed;
      message.message_type = UNKNOWN;
      messages.push_back(message);
      return messages;
    }
    if (stack[2].cmd == OSW_TY2_AFFILIATION) {
      // we have an affiliation
      ++numConsumed;
      // BOOST_LOG_TRIVIAL(warning)
      //     << "[" << system->get_short_name() << "] [affiliation] RID $"
      //     << std::hex << stack[3].full_address
      //     << " TG $" << std::hex << stack[2].full_address;
      message.message_type = AFFILIATION;
      message.talkgroup = stack[2].address;
      message.source = stack[3].full_address;
      messages.push_back(message);
      return messages;
    }
  }

  // If we get here, we don't know about this OCW (and/or a combination of it with others beside it).
  // Error accordingly and log the stack.

  // If we just got started, then we also could just be looking at the tail of a known message.
  // We preloaded the stack on a fresh start so there shouldn't be errors, but we have not
  // emptied and reloaded the stack on a CC retune. Future request.

  // There is also the possibility of missing OSWs and having incomplete messages.
  // Adding the logic to test for this might be nice to have (test could be "if we got here,
  // this OSW is is missing a header or other OSWs that comprise a valid message -
  // test if we know this OSW command though, and if we do, discard the OSW and move on")
   BOOST_LOG_TRIVIAL(trace)
       << "[" << system->get_short_name()
       << "] [Unknown OSW!] [ "
       << std::hex << stack[0].cmd << " " << std::hex << stack[0].grp << " " << std::hex << stack[0].full_address << "  |  "
       << std::hex << stack[1].cmd << " " << std::hex << stack[1].grp << " " << std::hex << stack[1].full_address << "  |  "
       << std::hex << stack[2].cmd << " " << std::hex << stack[2].grp << " " << std::hex << stack[2].full_address << "  | >"
       << std::hex << stack[3].cmd << " " << std::hex << stack[3].grp << " " << std::hex << stack[3].full_address << "< |  "
       << std::hex << stack[4].cmd << " " << std::hex << stack[4].grp << " " << std::hex << stack[4].full_address << " ]";
  message.message_type = UNKNOWN;
  messages.push_back(message);
  return messages;
}
