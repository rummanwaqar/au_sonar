/*
 * @author: Rumman Waqar
 *
 * Serial class
 * Read operations are done async and a callback is used to return the data
 * Write operations are blocking
 */

#ifndef _AU_SONAR_SERIAL_H_
#define _AU_SONAR_SERIAL_H_

#include <string>
#include <cstdint>
#include <functional>
#include <thread>
#include <boost/asio.hpp>
#include <boost/log/trivial.hpp>

namespace au_sonar {
  class Serial {
  public:
    /*
     * Create serial object with port and baud_rate
     */
    Serial(std::string port, unsigned int baud_rate = 115200);

    /*
     * calls close()
     */
    virtual ~Serial();

    /*
     * opens serial connection and create io service thread
     */
    bool init();
    /*
     * closes serial connection and joins the thread
     */
    void close();

    /*
     * returns true if serial connection is open
     */
    bool is_open();

    /*
     * Writes string to serial device. blocking
     * returns no of chars written
     */
    int write(const std::string &buf);

    /*
     * registers callback for reads
     */
    void register_receive_callback(std::function<void(const uint8_t*, size_t)> fun);

  private:
    /*
     * starts async read operation
     */
    void async_read();

    /*
     * called when read operation is complete. This function triggers the read callback
     */
    void read_complete(const boost::system::error_code& error, size_t bytes_transferred);

    static const unsigned int READ_BUFFER_SIZE_ = 128;  // read buffer size
    std::string port_;  // serial port address
    unsigned int baud_rate_;  // serial port baud rate
    uint8_t read_buffer_[READ_BUFFER_SIZE_]; // read buffer
    std::function<void(const uint8_t*, size_t)> receive_callback_;  // receive callback function
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
    std::thread io_thread_;
  };
} // namespace au_sonar

#endif // _AU_SONAR_SERIAL_H_
