/*
 * @author: Rumman Waqar
 *
 * Custom datatypes
 */

#ifndef _AU_SONAR_DATATYPES_H_
#define _AU_SONAR_DATATYPES_H_

#include <sstream>
#include <chrono>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <mutex>
#include <condition_variable>

using namespace std::chrono_literals;

namespace au_sonar {
  /*
   * Base time for data objects with timestamp
   */
  struct DataWithTime {
    // time when object is made
    std::chrono::high_resolution_clock::time_point timestamp;

    // initialize with current time
    DataWithTime() : timestamp(std::chrono::high_resolution_clock::now()){}

    // return object as string for debugging
    virtual std::string to_string() {
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>
        (timestamp.time_since_epoch()).count();
      return std::to_string(millis);
    }
  };

  /*
   * ping stats
   */
  struct PingInfo : DataWithTime {
    // dictionary of all stats key value pairs
    std::map<std::string, float> data;

    // return object as string for debugging
    std::string to_string() {
      std::ostringstream ostream;
      ostream << "Ping Info @ " << DataWithTime::to_string() << " {";
      for(auto const& x : data) {
        ostream << x.first  // string (key)
                << ':'
                << x.second // string's value
                << ",";
      }
      ostream << "}";
      return ostream.str();
    }
  };

  /*
   * single hydrophone-set data point
   */
  struct AdcSample {
    float a;
    float refa;
    float b;
    float refb;
  };

  /*
   * adc ping data from DAQ
   */
  struct PingData : DataWithTime {
    // vector of all adc samples for a single ping
    std::vector<AdcSample> data;

    // return object as string for debugging
    std::string to_string() {
      std::ostringstream ostream;
      ostream << "Ping Data @ " << DataWithTime::to_string()
              << " with " << data.size() << " samples.";
      return ostream.str();
    }
  };

  struct SonarData : DataWithTime {
  public:
    PingInfo info;
    PingData adcData;

    // add ping info data
    void add_data(PingInfo && data) {
      // acquire mutex
      std::lock_guard<std::mutex> lock(mux_);
      // write ping info data
      info = data;
      // if info and data timestamps match
      if(info.timestamp - adcData.timestamp < 500ms && adcData.data.size() > 0) {
        // send notification for data available
        timestamp = adcData.timestamp;
        convar_.notify_one();
      }
    }

    // add ping adc data
    void add_data(PingData && data) {
      // acquire mutex
      std::lock_guard<std::mutex> lock(mux_);
      // write ping info data
      adcData = data;
      // if info and data timestamps match
      if(adcData.timestamp - info.timestamp < 500ms && info.data.size() > 0) {
        // send notification for data available
        timestamp = info.timestamp;
        convar_.notify_one();
      }
    }

    // wait for data to be available and then pass it to callback
    void wait_and_process(std::function<void(std::chrono::high_resolution_clock::time_point timestamp, PingInfo& info, PingData& data)> func) {
      std::unique_lock<std::mutex> lock(mux_);
      // wait until full data object ready
      convar_.wait(lock);
      func(timestamp, info, adcData);
    }
  private:
     std::mutex mux_;
     std::condition_variable convar_;
  };

} // namspace au_sonar

#endif
