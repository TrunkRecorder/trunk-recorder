#include "../../trunk-recorder/plugin_manager/plugin_api.h"
#include "../../trunk-recorder/state.h"

#include <boost/dll/alias.hpp>
#include <boost/log/trivial.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <unistd.h>

class Bench_Logger : public Plugin_Api {
  std::string csv_path;
  FILE *csv_file = nullptr;
  std::mutex write_mu;
  std::atomic<uint64_t> sample_count{0};
  pid_t pid = 0;
  std::string branch_tag;

public:
  int parse_config(json config_data) override {
    csv_path = config_data.value("bench_csv", std::string("bench_active.csv"));
    branch_tag = config_data.value("branch_tag", std::string(""));
    return 0;
  }

  int start() override {
    pid = getpid();
    csv_file = std::fopen(csv_path.c_str(), "w");
    if (!csv_file) {
      BOOST_LOG_TRIVIAL(error) << "bench_logger: failed to open " << csv_path;
      return -1;
    }
    std::setvbuf(csv_file, nullptr, _IOLBF, 0);
    std::fprintf(csv_file, "epoch_ms,pid,n_active,n_recording,branch_tag\n");
    BOOST_LOG_TRIVIAL(info) << "bench_logger: writing to " << csv_path
                            << " (pid=" << pid << ", tag=" << branch_tag << ")";
    return 0;
  }

  int stop() override {
    std::lock_guard<std::mutex> lock(write_mu);
    if (csv_file) {
      std::fclose(csv_file);
      csv_file = nullptr;
    }
    BOOST_LOG_TRIVIAL(info) << "bench_logger: wrote " << sample_count.load()
                            << " samples to " << csv_path;
    return 0;
  }

  int calls_active(std::vector<Call *> calls) override {
    int n_active = static_cast<int>(calls.size());
    int n_recording = 0;
    for (Call *c : calls) {
      if (c && c->get_state() == RECORDING) {
        ++n_recording;
      }
    }

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();

    std::lock_guard<std::mutex> lock(write_mu);
    if (csv_file) {
      std::fprintf(csv_file, "%lld,%d,%d,%d,%s\n",
                   static_cast<long long>(now_ms), pid, n_active, n_recording,
                   branch_tag.c_str());
      sample_count.fetch_add(1, std::memory_order_relaxed);
    }
    return 0;
  }

  static boost::shared_ptr<Bench_Logger> create() {
    return boost::shared_ptr<Bench_Logger>(new Bench_Logger());
  }
};

BOOST_DLL_ALIAS(
    Bench_Logger::create,
    create_plugin)
