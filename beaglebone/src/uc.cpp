#include <iostream>
#include <string>
#include <csignal>
#include <atomic>

#include <zmq.hpp>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>

#include "datatypes.hpp"
#include "preprocessor.hpp"
#include "pru_reader.hpp"

#include "json.hpp"
using json = nlohmann::json;

#define ZMQ_COMMAND_SERVER "tcp://*:5555"

std::atomic<bool> keepRunning{true};

void signalHandler(int signum) {
   LOG_INFO << "Interrupt signal (" << signum << ") received.";
   keepRunning = false;
}

void process_sonar_data(std::chrono::system_clock::time_point timestamp, au_sonar::PingInfo& info, au_sonar::PingData& data) {
  json output;
  output["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>
    (timestamp.time_since_epoch()).count();
  output["data"] = data.to_json();
  output["info"] = info.to_json();
  LOG_INFO << "Got synced data frame for transmission (" << output.dump().size()/1024.0 << " kB)";
}

void command_thread(au_sonar::Preprocessor& preprocessor) {
    // setup zmq server
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);
    socket.bind(ZMQ_COMMAND_SERVER);

    while(keepRunning) {
      zmq::message_t request;
      //  Wait for next request from client
      socket.recv(&request);
      std::string command(static_cast<char*>(request.data()));
      LOG_DEBUG << "Got command: " << command;

      // send command to preprocessor and wait for response
      std::string response;
      try {
        response = preprocessor.write_command(command);
      } catch (std::runtime_error& e) {
        LOG_ERROR << e.what();
        response = std::string("error: unable to write");
      }
      LOG_DEBUG << "Returned command response: " << response;

      zmq::message_t reply(response.size());
      std::memcpy(reply.data(), response.data(), response.size());
      socket.send(reply);
    }
}

int main() {
  // sigint
  signal(SIGINT, signalHandler);

  // logging
  static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
  plog::init(plog::info, "log.txt").addAppender(&consoleAppender);

  // create sonar data object
  au_sonar::SonarData sonar_data;

  // init preprocessor
  au_sonar::Preprocessor preprocessor("/dev/ttyO4", std::ref(sonar_data));
  if(!preprocessor.init()) {
    LOG_INFO << "Exiting program";
    return 2;
  }

  // init pru Reader
  au_sonar::PruReader pruReader("pru0-clock.bin", "pru1-read-data.bin", std::ref(sonar_data));
  if(!pruReader.init()) {
    LOG_INFO << "Exiting program";
    return 2;
  }

  std::thread thread_A{command_thread, std::ref(preprocessor)};
  thread_A.detach();

  while(keepRunning) {
    sonar_data.wait_and_process(process_sonar_data);
  }

  return 0;
}
