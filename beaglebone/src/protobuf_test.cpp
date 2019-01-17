#include <iostream>
#include "sonardata.pb.h"
#include <sys/time.h>
#include <string>
#include <zmq.hpp>
#include <chrono>
#include <thread>

#define SERVER_ADDRESS "tcp://127.0.0.1:9999"

void send(std::string& output, zmq::socket_t& pub) {
  std::size_t len = output.size();
  zmq::message_t message(len);
  memcpy(message.data(), output.c_str(), len);
  pub.send(message);
  std::cout << "Sent " << len << " bytes" << std::endl;
}

struct Data {
  int a;
  int refa;
  int b;
  int refb;

  au_sonar::SonarData::Data to_proto() {
    au_sonar::SonarData::Data data;
    data.set_a(a);
    data.set_refa(refa);
    data.set_b(b);
    data.set_refb(refb);
    return data;
  }
};
int main() {
  // Verify protobuf library version with version of headers
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  // start zmq publisher
  zmq::context_t context(1);
  zmq::socket_t publisher(context, ZMQ_PUB);
  publisher.bind(SERVER_ADDRESS);
  std::cout << "Started ZMQ publisher at " << SERVER_ADDRESS << std::endl;

  // gen fake data
  Data d = {10,9,8,7};
  au_sonar::SonarData::Data b = d.to_proto();
  std::vector<au_sonar::SonarData::Data> data_vector;
  for(int i=0; i<16; i++) {
    data_vector.push_back(b);
  }

  au_sonar::SonarData sonar_data;
  while(1) {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto timestamp = sonar_data.mutable_timestamp();
    timestamp->set_seconds(time(NULL));
    timestamp->set_nanos(0);

    // add data
    *sonar_data.mutable_data() = {data_vector.begin(), data_vector.end()};

    // add stats
    auto map = sonar_data.mutable_stats();
    (*map)["cal"] = 1;
    (*map)["gain"] = 40.0;

    // std::cout << sonar_data.DebugString() << std::endl;

    std::string output;
    sonar_data.SerializeToString(&output);
    send(output, publisher);
    sonar_data.Clear();
  }

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
