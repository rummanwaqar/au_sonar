/*
 * @author: Rumman Waqar
 *
 * Custom datatypes
 */

#ifndef _AU_SONAR_DATATYPES_H_
#define _AU_SONAR_DATATYPES_H_

#include <chrono>
#include <string>
#include <map>
#include <vector>
#include <algorithm>

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

} // namspace au_sonar

#endif
