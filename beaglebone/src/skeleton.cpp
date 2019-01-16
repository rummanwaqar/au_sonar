#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;

class BaseInterface {
public:
  BaseInterface(std::string out) : output_(out), thread_(), stop_flag_(false) {}
  virtual ~BaseInterface() {
    stop_flag_ = true;
    if(thread_.joinable()) thread_.join();
  }

  virtual bool start() {
    thread_ = std::thread(&BaseInterface::run, this);
    return true;
  }

  virtual void run() {
    while(!stop_flag_) {
      std::this_thread::sleep_for(1s);
      // set data
      std::lock_guard<std::mutex> lock(mux_);
      data_ = output_;
      if(available_) {
        std::cout << "overwritten" << std::endl;
      }
      available_ = true;

    }
  }

  bool is_available() {
    return available_;
  }

  std::string get_data() {
    std::lock_guard<std::mutex> lock(mux_);
    available_ = false;
    return data_;
  }

private:
  const std::string output_;
  std::thread thread_;
  bool stop_flag_;

  std::string data_;
  std::atomic<bool> available_;
  std::mutex mux_;

};

int main() {
  // make objects
  BaseInterface preprocessor("Preprocessor data");
  BaseInterface adc("Adc data");

  // start
  preprocessor.start();
  adc.start();

  while(1) {
    // poll
    if(preprocessor.is_available() && adc.is_available()) {
      std::cout << preprocessor.get_data() << "\t"
                << adc.get_data() << std::endl;
    }
  }
  return 0;
}
