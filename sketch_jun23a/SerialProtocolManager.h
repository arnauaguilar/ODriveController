// If DEBUG is set to true, the arduino will send back all the received messages
#define DEBUG false
#define SERIAL_BUFFER_SIZE 64  // OLED display height, in pixels

#include <Arduino.h>

class SerialProtocolManager {
public:

  void GetMessagesFromSerial(float& var1, float& var2);
  void ReadSignedBytes(int8_t* buffer, size_t n);
  void WaitForBytes(uint8_t num_bytes, unsigned long timeout);

  template<typename T = uint8_t>
  void Write(T val);
  template<typename T>
  T Read();



  // Define the orders that can be sent and received
  enum Order: uint8_t {
    UNKNOWN = 0,
    HELLO = 1,
    VAR1 = 2,
    VAR2 = 3,
    ALREADY_CONNECTED = 4,
    ERROR = 5,
    RECEIVED = 6,
    STOP = 7
  };

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
  int8_t buffer[SIZE];
  WaitForBytes(SIZE, 200);
  ReadSignedBytes(buffer, SIZE);
  long result = NULL;
  long mask = 0xff;
  for (size_t i = 0; i < SIZE; ++i) {
    result |= ((T)buffer[i]) << 8 * i & mask;
    mask <<= 8;
  }
  return (T)result;
}

//int8_t read_i8() {
//  wait_for_bytes(1, 100);  // Wait for 1 byte with a timeout of 100 ms
//  return (int8_t)Serial.read();
//}
//
//int16_t read_i16() {
//  int8_t buffer[2];
//  wait_for_bytes(2, 100);  // Wait for 2 bytes with a timeout of 100 ms
//  read_signed_bytes(buffer, 2);
//  return (((int16_t)buffer[0]) & 0xff) | (((int16_t)buffer[1]) << 8 & 0xff00);
//}
//
//int32_t read_i32() {
//  int8_t buffer[4];
//  wait_for_bytes(4, 200);  // Wait for 4 bytes with a timeout of 200 ms
//  read_signed_bytes(buffer, 4);
//  return (((int32_t)buffer[0]) & 0xff) | (((int32_t)buffer[1]) << 8 & 0xff00) | (((int32_t)buffer[2]) << 16 & 0xff0000) | (((int32_t)buffer[3]) << 24 & 0xff000000);
//}



//void write_order(enum Order myOrder) {
//  uint8_t* Order = (uint8_t*)&myOrder;
//  Serial.write(Order, sizeof(uint8_t));
//}
//
//void write_i8(int8_t num) {
//  Serial.write(num);
//}
//
//void write_i16(int16_t num) {
//  int8_t buffer[2] = { (int8_t)(num & 0xff), (int8_t)(num >> 8) };
//  Serial.write((uint8_t*)&buffer, 2 * sizeof(int8_t));
//}
//
//void write_i32(int32_t num) {
//  int8_t buffer[4] = { (int8_t)(num & 0xff),
//                       (int8_t)(num >> 8 & 0xff),
//                       (int8_t)(num >> 16 & 0xff),
//                       (int8_t)(num >> 24 & 0xff) };
//  Serial.write((uint8_t*)&buffer, 4 * sizeof(int8_t));
//}
//