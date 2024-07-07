#include "api/Binary.h"
// If DEBUG is set to true, the arduino will send back all the received messages
#define DEBUG false
//#define SERIAL_SIZE 64  // OLED display height, in pixels

#include <Arduino.h>
#include <map>
#include <functional>

class SerialProtocolManager {
public:
  using callbackSignature = std::function<void(/* parameters go here like this: float a, int b | or: void* that means whatever inputs you want */)>;
  //std::function<void()>
  void GetMessagesFromSerial(float& var1, float& var2);
  [[deprecated("use Serial.readBytes instead")]] void ReadSignedBytes(int8_t* buffer, size_t n);
  void WaitForBytes(uint8_t num_bytes, unsigned long timeout);

  template<typename T = uint8_t>
  void Write(T val);
  template<typename T>
  T Read();

  enum Order : uint8_t {
    UNKNOWN = 0,
    HELLO = 1,
    POSITION = 2,
    VELOCITY = 3,
    TORQUE = 4,
    ALREADY_CONNECTED = 5,
    RECEIVED = 6,
    ERROR = 7
  };

  void RegisterCallback(Order order, std::function<void()> callback);

  std::map<Order, std::function<void()> > callbacks;




  bool is_connected = false;  ///< True if the connection with the master is available
  //Order Order;
};

template<typename T = uint8_t>
void SerialProtocolManager::Write(T val) {
  constexpr size_t SIZE = sizeof(T) / sizeof(uint8_t);
  int8_t buffer[SIZE];
  for (size_t i = 0; i < SIZE; ++i) {
    buffer[i] = (int8_t)(val >> 8 * i & 0xff);
  }
  Serial.write((uint8_t*)&buffer, SIZE);
}


template<typename T>  //specialize uint8 for speed
T SerialProtocolManager::Read() {
  constexpr size_t SIZE = sizeof(T) / sizeof(uint8_t);
  byte buffer[SIZE];
  WaitForBytes(SIZE, 200);
  Serial.readBytes(buffer, SIZE);

  long result = NULL;
  long mask = 0xff;
  for (size_t i = 0; i < SIZE; ++i) {
    result |= ((T)buffer[i]) << 8 * i & mask;
    mask <<= 8;
  }
  return (T)result;
}