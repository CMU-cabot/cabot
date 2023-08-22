#ifndef ARDUINOSERIAL_H_
#define ARDUINOSERIAL_H_


#include <string>
#include <vector>
#include <thread>
#include <mutex> // lock
#include <chrono>
#include <unistd.h>


#include <queue>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <serial/serial.h>

class CaBotSerialNode;

class CaBotArduinoSerialDelegate {
public:
  virtual ~CaBotArduinoSerialDelegate() = default;

  virtual std::tuple<int, int> system_time() = 0;
  virtual void stopped() = 0;
  virtual void log(int level, const std::string & text) = 0;
  virtual void log_throttle(int level, int interval, const std::string & text) = 0;
  virtual void get_param(const std::string & name, std::function <void(const std::vector<int>&)> callback) = 0;
  virtual void publish(uint8_t cmd, const std::vector<uint8_t>&data) = 0; 

  CaBotArduinoSerialDelegate() = default;

protected:
  CaBotArduinoSerialDelegate(CaBotSerialNode* delegate_node);
  CaBotSerialNode* delegate_node_;
};

class CaBotArduinoSerial : public rclcpp::Node{
public:
  CaBotArduinoSerial(const std::string& port, int baud, std::chrono::milliseconds timeout =std::chrono::milliseconds(1000));
  
  void start();
  void send_command(uint8_t command, const std::vector<uint8_t>& arg);
  void stop();

  template<typename T>
  void send_param(const T& data);

  CaBotArduinoSerialDelegate* delegate_;
  void reset_serial();


private:
  rclcpp::Logger logger_;
  serial::Serial port_;
  int baud_;
  std::chrono::milliseconds timeout_;
  // CaBotArduinoSerialDelegate* delegate_;
  std::thread read_thread_;
  std::mutex read_mutex_;
  std::thread write_thread_;
  std::queue<std::vector<uint8_t>> write_queue_;
  std::mutex write_mutex_;
  bool is_alive_;
  int read_count_;
  bool time_synced_;
  int no_input_count_;
  // void reset_serial();
  void run_once();
  bool process_write_once();
  bool try_read(int length, std::vector<uint8_t>& result);
  bool process_read_once();
  void send_time_sync(const std::vector<uint8_t>& data);
  uint8_t checksum(const std::vector<uint8_t>& data);

  //friend class CaBotSerialNode;
};


template<typename T>
void CaBotArduinoSerial::send_param(const T& data){
  std::vector<uint8_t> temp;
  for(typename T::const_iterator D = data.begin(); D != data.end(); ++D){
    typename T::value_type d = *D;
    for(int i = 0; i < sizeof(typename T::value_type); i++){
      temp.push_back((d >> (8 * i))& 0xFF);
    }
  }
  send_command(0x08, temp);
  std::string data_str;
  for(typename T::const_iterator D = data.begin(); D != data.end(); ++D){
    typename T::value_type d = *D;
    data_str += std::to_string(d) +" ";
  }
  RCLCPP_INFO(get_logger(), "%s", data_str.c_str());
}


#endif /* ARDUINOSERIAL_H_ */
