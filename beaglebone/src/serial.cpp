#include "serial.hpp"

using namespace au_sonar;

Serial::Serial(std::string port, unsigned int baud_rate) :
  port_(port), baud_rate_(baud_rate), io_service_(), serial_port_(io_service_) {
    LOG_VERBOSE << "Serial object created";
}

Serial::~Serial() {
  close();
}


bool Serial::init() {
  // open serial port
  try {
    serial_port_.open(port_);
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  } catch(boost::system::system_error e) {
    LOG_ERROR << e.what();
    return false;
  }
  // start read operation
  async_read();
  // create a thread for io service to run on
  io_thread_ = std::thread([&]{ io_service_.run(); });
  LOG_ERROR << "Serial port open: " << port_;
  return true;
}

void Serial::close() {
  try {
    serial_port_.close();
    io_service_.stop();
    if(io_thread_.joinable()) {
      io_thread_.join();
    }
  } catch(std::exception& e) {
    LOG_ERROR << e.what();
  }
  LOG_INFO << "Serial port closed";
}

bool Serial::is_open() {
  return serial_port_.is_open();
}

int Serial::write(const std::string &buf) {
  boost::system::error_code ec;
  if(!is_open()) return 0;
  if(buf.size() == 0) return 0;
  LOG_VERBOSE << "Wrote to serial: " << buf;
  return serial_port_.write_some(boost::asio::buffer(buf.c_str(), buf.size()), ec);
}

void Serial::register_receive_callback(std::function<void(const uint8_t*, size_t)> fun) {
  receive_callback_ = fun;
}

void Serial::async_read() {
  if(!is_open()) return;
  serial_port_.async_read_some(boost::asio::buffer(read_buffer_, READ_BUFFER_SIZE_),
                               std::bind(&Serial::read_complete, this, std::placeholders::_1, std::placeholders::_2));
}

void Serial::read_complete(const boost::system::error_code& error, size_t bytes_transferred) {
  if(error) {
    LOG_ERROR << "Reading: " << error.message();
    close();
    return;
  }
  receive_callback_(read_buffer_, bytes_transferred);
  async_read();
}
