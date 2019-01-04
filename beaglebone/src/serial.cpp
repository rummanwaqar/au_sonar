#include "serial.hpp"

#include <iostream>

using namespace au_sonar;

Serial::Serial(std::string port, unsigned int baud_rate) :
  io_service_(), port_(port), baud_rate_(baud_rate), serial_port_(io_service_)  {}

Serial::~Serial() {}

bool Serial::init() {
  try {
    serial_port_.open(port_);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  } catch(boost::system::system_error e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

  async_read();
  io_thread_ = std::thread([&]{ io_service_.run(); });
  return true;
}

void Serial::close() {
  serial_port_.close();
  mutex_lock lock(mutex_);
  io_service_.stop();

  if(io_thread_.joinable()) {
    io_thread_.join();
  }
}

bool Serial::is_open() {
  return serial_port_.is_open();
}

void Serial::register_receive_callback(std::function<void(const uint8_t*, size_t)> fun) {
  receive_callback_ = fun;
}

void Serial::async_read() {
  if(!is_open()) return;
  serial_port_.async_read_some(boost::asio::buffer(read_buffer_, 1024),
                               boost::bind(&Serial::read_complete, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void Serial::read_complete(const boost::system::error_code& error, size_t bytes_transferred) {
  if(error) {
    std::cerr << error.message() << std::endl;
    close();
    return;
  }
  receive_callback_(read_buffer_, bytes_transferred);
  async_read();
}
