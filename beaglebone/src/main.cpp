#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Log.h>
#include <zmq.hpp>

#include "datatypes.hpp"
#include "preprocessor.hpp"
#include "pru_reader.hpp"

#include "cxxopts.hpp"
#include "json.hpp"
using json = nlohmann::json;

#define ZMQ_COMMAND_SERVER "tcp://127.0.0.1:1234"
#define ZMQ_DATA_SERVER "tcp://127.0.0.1:1235"

std::atomic<bool> keepRunning{true};
zmq::context_t context(1);
std::unique_ptr<zmq::socket_t> publisher;
// zmq::socket_t* publisher;

void signalHandler(int signum) {
  LOG_INFO << "Interrupt signal (" << signum << ") received.";
  keepRunning = false;
}

void process_sonar_data(std::chrono::system_clock::time_point timestamp,
                        au_sonar::PingInfo& info, au_sonar::PingData& data) {
  json output;
  output["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                            timestamp.time_since_epoch())
                            .count();
  output["data"] = data.to_json();
  output["info"] = info.to_json();

  std::string output_string = output.dump();
  LOG_INFO << "Got synced data frame for transmission ("
           << output_string.size() / 1024.0 << " kB)";

  // publish ping data
  zmq::message_t message(output_string.size());
  memcpy(message.data(), output_string.data(), output_string.size());
  publisher->send(message);
}

void command_thread(au_sonar::Preprocessor& preprocessor,
                    const std::string server_addr) {
  // setup zmq server for commands
  zmq::socket_t socket(context, ZMQ_REP);
  socket.bind(server_addr);
  LOG_INFO << "Started ZMQ command server at " << server_addr;

  while (keepRunning) {
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

int main(int argc, char** argv) {
  // parse cli
  cxxopts::Options options("sonar_daq", "Sonar data acquisition software");
  // clang-format off
  options.add_options()
    ("pru0", "PRU0 firmware file",cxxopts::value<std::string>())
    ("pru1", "PRU1 firmware file", cxxopts::value<std::string>())
    ("log", "Log file", cxxopts::value<std::string>()->default_value("log.txt"))
    ("cmd_server", "Address for zmq command server",
      cxxopts::value<std::string>()->default_value(ZMQ_COMMAND_SERVER))
    ("data_server", "Address for zmq data publisher",
      cxxopts::value<std::string>()->default_value(ZMQ_DATA_SERVER))
    ("d,debug", "Enable debugging logs");
  // clang-format on
  auto arguments = options.parse(argc, argv);
  bool debug_flag;
  std::string pru0_fname, pru1_fname, log_fname, cmd_server, data_server;
  try {
    debug_flag = arguments["d"].as<bool>();
    pru0_fname = arguments["pru0"].as<std::string>();
    pru1_fname = arguments["pru1"].as<std::string>();
    cmd_server = arguments["cmd_server"].as<std::string>();
    data_server = arguments["data_server"].as<std::string>();
    log_fname = arguments["log"].as<std::string>();
  } catch (std::domain_error& e) {
    std::cerr << options.help() << std::endl;
    exit(2);
  }

  // sigint
  signal(SIGINT, signalHandler);

  // logging
  plog::Severity severity = plog::info;
  if (debug_flag) {
    severity = plog::debug;
  }
  static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
  plog::init(severity, log_fname.c_str()).addAppender(&consoleAppender);

  // start zmq data publisher
  publisher = std::make_unique<zmq::socket_t>(context, ZMQ_PUB);
  publisher->bind(data_server);
  LOG_INFO << "Started ZMQ data publisher at " << data_server;

  // create sonar data object
  au_sonar::SonarData sonar_data;

  // // init preprocessor
  au_sonar::Preprocessor preprocessor("/dev/ttyO4", std::ref(sonar_data));
  if (!preprocessor.init()) {
    LOG_INFO << "Exiting program";
    return 2;
  }

  // init pru Reader
  au_sonar::PruReader pruReader(pru0_fname, pru1_fname, std::ref(sonar_data));
  if (!pruReader.init()) {
    LOG_INFO << "Exiting program";
    return 2;
  }

  std::thread thread_A{command_thread, std::ref(preprocessor), cmd_server};
  thread_A.detach();

  while (keepRunning) {
    sonar_data.wait_and_process(process_sonar_data);
  }

  return 0;
}
