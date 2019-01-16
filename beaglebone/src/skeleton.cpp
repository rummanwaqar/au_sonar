#include <iostream>
#include <thread>
#include <mutex>
#include <sys/time.h>

struct Item_t {
  long long timestamp_;
  std::string data_;

  Item_t() : timestamp_(0), data_("") {}
  Item_t(std::string data) : timestamp_(time(NULL)), data_(data) {}
};

struct Data_t {
  Item_t preprocessor_;
  Item_t adc_;
};

void producer_1(Data_t& data, std::mutex& mux, std::condition_variable& convar) { // preprocessor
  while(1) {
    // simulate wait for data
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // got data
    Item_t preprocessor_data("Preprocessor stuff");
    // acquire mutex
    std::lock_guard<std::mutex> lock(mux);
    // write preprocessor data
    data.preprocessor_ = preprocessor_data;
    // if adc timestamp match
    if(std::abs(data.adc_.timestamp_ - data.preprocessor_.timestamp_) < 1) {
      // send notification
      convar.notify_one();
    }
  }
}

void producer_2(Data_t& data, std::mutex& mux, std::condition_variable& convar) { // adc
  while(1) {
    // simulate wait for data
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // got data
    Item_t adc_data("ADC stuff");
    // acquire mutex
    std::lock_guard<std::mutex> lock(mux);
    // write adc data
    data.adc_ = adc_data;
    // if preprocessor timestamp match
    if(std::abs(data.adc_.timestamp_ - data.preprocessor_.timestamp_) < 1) {
      // send notification
      convar.notify_one();
    }
  }
}

int main() {
  Data_t data;
  std::mutex mux;
  std::condition_variable convar;

  std::thread threadB{producer_2, std::ref(data), std::ref(mux), std::ref(convar)};
  std::thread threadA{producer_1, std::ref(data), std::ref(mux), std::ref(convar)};


  while(1) {
    std::unique_lock<std::mutex> lock(mux);
    // wait until full data object ready
    convar.wait(lock);
    // send data out
    std::cout << time(NULL) << ": Got data" << "\t"
              << data.adc_.timestamp_ << "@" << data.adc_.data_ << "\t"
              << data.preprocessor_.timestamp_ << "@" << data.preprocessor_.data_ << std::endl;
  }
  return 0;
}
