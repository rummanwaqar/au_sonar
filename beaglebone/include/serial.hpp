#ifndef _AU_SONAR_SERIAL_H_
#define _AU_SONAR_SERIAL_H_

#include <string>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include <boost/bind.hpp>
#include <boost/asio.hpp>

namespace au_sonar {
  class Serial {
  public:
    Serial(std::string port, unsigned int baud_rate);
    virtual ~Serial();

    bool init();
    void close();
    bool is_open();



    void send_bytes(const uint8_t* src, size_t len);
    inline void send_byte(uint8_t data) { send_bytes(&data, 1); }
    void register_receive_callback(std::function<void(const uint8_t*, size_t)> fun);
  private:
    boost::asio::io_service io_service_;
    std::thread io_thread_;
    std::recursive_mutex mutex_;
    uint8_t read_buffer_[1024];
    std::string port_;
    unsigned int baud_rate_;
    boost::asio::serial_port serial_port_;

    std::function<void(const uint8_t*, size_t)> receive_callback_;

    typedef std::lock_guard<std::recursive_mutex> mutex_lock;

    void async_write();
    void async_read();

    void read_complete(const boost::system::error_code& error, size_t bytes_transferred);
  };
} // namespace au_sonar

#endif // _AU_SONAR_SERIAL_H_
