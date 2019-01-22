#include <iostream>
#include <string>
#include <csignal>

#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#include "preprocessor.hpp"

au_sonar::Preprocessor* preprocessor;

void signalHandler(int signum) {
   BOOST_LOG_TRIVIAL(info) << "Interrupt signal (" << signum << ") received.";
   delete preprocessor;
   exit(signum);
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

int main() {
  signal(SIGINT, signalHandler);
  setup_logging("sample.log");

  // init preprocessor
  preprocessor = new au_sonar::Preprocessor("/dev/tty.usbmodem31796101");
  if(!preprocessor->init()) {
    BOOST_LOG_TRIVIAL(info) << "EXITING";
    return 2;
  }

  while(1);

  return 0;
}
