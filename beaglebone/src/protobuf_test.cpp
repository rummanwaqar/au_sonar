#include <iostream>
#include "sonardata.pb.h"
#include <sys/time.h>
#include <string>
#include <zmq.hpp>
#include <chrono>
#include <thread>

#define SERVER_ADDRESS "tcp://127.0.0.1:9999"

int main() {
  // Verify protobuf library version with version of headers
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  au_sonar::SonarData sonar_data;

  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(time(NULL));
  timestamp.set_nanos(0);

  std::string output;
  timestamp.SerializeToString(&output);

  // start zmq publisher
  zmq::context_t context(1);
  zmq::socket_t publisher(context, ZMQ_PUB);
  publisher.bind(SERVER_ADDRESS);

  std::cout << "Started ZMQ publisher at " << SERVER_ADDRESS << std::endl;

  while(1) {
    std::cout << "Sending" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    zmq::message_t message(output.size());
    memcpy(message.data(), output.c_str(), output.size());
    publisher.send(message);
  }

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
