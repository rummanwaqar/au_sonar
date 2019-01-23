#include <iostream>
#include <string>
#include <csignal>
#include <zmq.hpp>

#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#include "datatypes.hpp"
#include "preprocessor.hpp"

#define ZMQ_COMMAND_SERVER "tcp://*:5555"

bool exit_flag = false;

void signalHandler(int signum) {
   BOOST_LOG_TRIVIAL(info) << "Interrupt signal (" << signum << ") received.";
   exit_flag = true;
}

void setup_logging(std::string log_file) {
  boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");

  boost::log::add_file_log(
    log_file,
    boost::log::keywords::format = "[%TimeStamp%] [%ThreadID%] [%Severity%] %Message%"
  );
  boost::log::add_console_log(std::cout, boost::log::keywords::format = "[%TimeStamp%] [%Severity%] %Message%");
  boost::log::core::get()->set_filter(
    boost::log::trivial::severity >= boost::log::trivial::debug);
  boost::log::add_common_attributes();
}

void process_sonar_data(std::chrono::high_resolution_clock::time_point timestamp, au_sonar::PingInfo& info, au_sonar::PingData& data) {
  std::cout << "got data" << std::endl;
}

void command_thread(au_sonar::Preprocessor& preprocessor) {
    // setup zmq server
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);
    socket.bind(ZMQ_COMMAND_SERVER);

    while(1) {
      zmq::message_t request;
      //  Wait for next request from client
      socket.recv(&request);
      std::string command(static_cast<char*>(request.data()));

      // send command to preprocessor and wait for response
      std::string response;
      try {
        response = preprocessor.write_command(command, 1000);
      } catch (std::runtime_error& e) {
        BOOST_LOG_TRIVIAL(error) << e.what();
      }
      if(response == "") {
        //  Send reply back to client
        response = std::string("error: unable to write");
      }

      zmq::message_t reply(response.size());
      std::memcpy(reply.data(), response.data(), response.size());
      socket.send(reply);
    }
}

int main() {
  signal(SIGINT, signalHandler);
  setup_logging("sample.log");

  // create sonar data object
  au_sonar::SonarData sonar_data;

  // init preprocessor
  au_sonar::Preprocessor preprocessor("/dev/tty.usbmodem31796101", std::ref(sonar_data));
  if(!preprocessor.init()) {
    BOOST_LOG_TRIVIAL(info) << "EXITING";
    return 2;
  }

  std::thread thread_A{command_thread, std::ref(preprocessor)};

  while(!exit_flag) {
    sonar_data.wait_and_process(process_sonar_data);
  }

  return 0;
}
