/*
 * @author: Rumman Waqar
 *
 * Custom datatypes
 */

#ifndef _AU_SONAR_DATATYPES_H_
#define _AU_SONAR_DATATYPES_H_

#include <chrono>
#include <map>
#include <vector>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <iostream>

namespace au_sonar {
  struct PingInfo {
    std::chrono::high_resolution_clock::time_point timestamp;
    std::map<std::string, std::string> data;

    PingInfo(std::vector<std::string>& tokens) :
      timestamp(std::chrono::high_resolution_clock::now()) {
      for_each(tokens.begin()+1, tokens.end(), [&](std::string const& token) {
        std::vector<std::string> pair;
        boost::split(pair, token, boost::is_any_of("="));
        data[pair[0]] = pair[1];
      });
    }

    // overloading operator <<
    friend std::ostream& operator<< (std::ostream &out, const PingInfo& ping) {
      out << "Ping @ "
          << std::chrono::duration_cast<std::chrono::milliseconds>(
                ping.timestamp.time_since_epoch()).count()
          << " {";
      for (auto const& x : ping.data) {
          out << x.first  // string (key)
              << ':'
              << x.second // string's value
              << ",";
      }
      out  << "}";
      return out;
    }
  };



} // namspace au_sonar

#endif
