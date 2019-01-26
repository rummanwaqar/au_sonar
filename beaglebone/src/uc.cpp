#include <string>
#include <csignal>
#include <atomic>
#include <memory>

#include <zmq.hpp>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>

#include "datatypes.hpp"
#include "preprocessor.hpp"
#include "pru_reader.hpp"

#include "json.hpp"
using json = nlohmann::json;

#define ZMQ_COMMAND_SERVER "tcp://*:5555"
#define ZMQ_DATA_SERVER "tcp://*:5556"

std::atomic<bool> keepRunning{true};
zmq::context_t context(1);
std::unique_ptr<zmq::socket_t> publisher;
// zmq::socket_t* publisher;

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

  std::string output_string = output.dump();
  LOG_INFO << "Got synced data frame for transmission (" << output_string.size()/1024.0 << " kB)";

  // publish ping data
  zmq::message_t message(output_string.size());
  memcpy(message.data(), output_string.data(), output_string.size());
  publisher->send(message);
}

void command_thread(au_sonar::Preprocessor& preprocessor) {
    // setup zmq server for commands
    zmq::socket_t socket(context, ZMQ_REP);
    socket.bind(ZMQ_COMMAND_SERVER);

    LOG_INFO << "Started ZMQ command server at " << ZMQ_COMMAND_SERVER;

    while(keepRunning) {
      zmq::message_t request;
      //  Wait for next request from client
      socket.recv(&request);
      std::string command(static_cast<char*>(request.data()));
      LOG_INFO << "Got command: " << command;

      // send command to preprocessor and wait for response
      std::string response;
      try {
        response = preprocessor.write_command(command);
      } catch (std::runtime_error& e) {
        LOG_ERROR << e.what();
        response = std::string("error: unable to write");
      }
      LOG_INFO << "Returned command response: " << response;

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

  // start zmq data publisher
  publisher = std::make_unique<zmq::socket_t>(context, ZMQ_PUB);
  // publisher = new zmq::socket_t(context, ZMQ_PUB);
  publisher->bind(ZMQ_DATA_SERVER);
  LOG_INFO << "Started ZMQ data publisher at " << ZMQ_DATA_SERVER;

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
