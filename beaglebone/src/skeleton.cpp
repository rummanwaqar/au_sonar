#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include "datatypes.hpp"

void producer_1(au_sonar::SonarData& sonar_data) { // preprocessor
  while(1) {
    // simulate wait for data
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // got data
    au_sonar::PingInfo info;
    info.data["cal"] = 1.;
    info.data["gain"] = 23.;
    std::cout << info.to_string() << std::endl;
    sonar_data.add_data(std::move(info));
  }
}

void producer_2(au_sonar::SonarData& sonar_data) { // adc
  while(1) {
    // simulate wait for data
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // got data
    au_sonar::PingData data;
    au_sonar::AdcSample sample{1,2,3,4};
    data.data.push_back(sample);
    data.data.push_back(sample);
    data.data.push_back(sample);
    std::cout << data.to_string() << std::endl;
    sonar_data.add_data(std::move(data));
  }
}

void process_sonar_data(std::chrono::high_resolution_clock::time_point timestamp, au_sonar::PingInfo& info, au_sonar::PingData& data) {
  std::cout << "got data" << std::endl;
}

int main() {
  au_sonar::SonarData sonar_data;

  std::thread threadA{producer_1, std::ref(sonar_data)};
  std::thread threadB{producer_2, std::ref(sonar_data)};

  while(1) {
    sonar_data.wait_and_process(process_sonar_data);
  }

  return 0;
}
