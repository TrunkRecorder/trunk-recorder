#include "../../trunk-recorder/plugin_manager/plugin_api.h"
#include "../../trunk-recorder/systems/system.h"
#include <boost/dll/alias.hpp> // for BOOST_DLL_ALIAS
#include <boost/foreach.hpp>

int freq;
float timeDiff2;
float lastTime;
char decode_rates_file[255];

class Decode_Rates : public Plugin_Api {

public:

  int system_rates(std::vector<System *> systems, float timeDiff) override {
    time_t now = time(NULL);
    if (now-lastTime > freq) {
      std::ofstream myfile(decode_rates_file, std::ofstream::app);
      if (myfile.is_open()) {
        myfile.fill('0');
        myfile << now;
        for (std::vector<System *>::iterator it = systems.begin(); it != systems.end(); it++) {
          System *sys = *it;
          if ((sys->get_system_type() != "conventional") && (sys->get_system_type() != "conventionalP25") && (sys->get_system_type() != "conventionalDMR")) {
            int msgs_decoded_per_second = std::floor(sys->get_message_count() / timeDiff);
            myfile << "," << msgs_decoded_per_second;
          }
        }
        myfile << "\n";
        myfile.close();
        lastTime = now;
      } else {
        return 1;
      }
      return 0;
    } else {
      return 0;
    }
  }

  int parse_config(boost::property_tree::ptree &cfg) {
    freq = cfg.get<int>("logDecodeRates", 0);
    BOOST_LOG_TRIVIAL(info) << " Decode rate logging frequency (secs): " << freq;
    int nchars;
    std::string capture_dir = cfg.get<std::string>("captureDir", boost::filesystem::current_path().string());
    size_t pos = capture_dir.find_last_of("/");
    if (pos == capture_dir.length() - 1) {
      capture_dir.erase(capture_dir.length() - 1);
    }
    nchars = snprintf(decode_rates_file, 255, "%s/decode_rates.csv", capture_dir.c_str());
    std::ofstream myfile(decode_rates_file, std::ofstream::app);
    if (myfile.is_open()) {
      myfile << "timestamp";
      BOOST_FOREACH (boost::property_tree::ptree::value_type &node, cfg.get_child("systems")) {
        myfile << "," << node.second.get<std::string>("shortName", "");
      }
      myfile << "\n";
      myfile.close();
    } else {
      return 1;
    }
    return nchars;
  }

  static boost::shared_ptr<Decode_Rates> create() {
    return boost::shared_ptr<Decode_Rates>(
        new Decode_Rates());
  }
};

BOOST_DLL_ALIAS(
    Decode_Rates::create, // <-- this function is exported with...
    create_plugin             // <-- ...this alias name
)
