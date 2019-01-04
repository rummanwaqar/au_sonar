/*
 * Deps: boost

*/
#include <iostream>
#include "serial.hpp"

void callback(const uint8_t* buf, size_t len) {
  std::cout.write((char*)buf, len);
  std::cout << std::endl;
}

int main() {
  au_sonar::Serial serial("/dev/tty.usbmodem31796101", 115200);
  serial.register_receive_callback(&callback);
  if(!serial.init()) {
    std::cerr << "Failed to initialize the serial port" << std::endl;
    return 2;
  }

  // serial.write("$set gain 60.0\n");
  while(1);
  serial.close();
  return 0;
}
