#pragma once
#include "Arduino.h"
#include <functional>
#include "SerialProtocolManager.h"

void SerialProtocolManager::GetMessagesFromSerial(float& var1,float& var2) {

  // The first byte received is the instruction
  Order readedOrder = Read<Order>();
  callbacks[readedOrder]();
}

void SerialProtocolManager::RegisterCallback(SerialProtocolManager::Order order, std::function<void()> callback){
  callbacks[order] = callback;
}

void SerialProtocolManager::ReadSignedBytes(int8_t* buffer, size_t n) {
  size_t i = 0;
  int c;
  while (i < n) {
    c = Serial.read();
    if (c < 0) break;
    *buffer++ = (int8_t)c;  // buffer[i] = (int8_t)c;
    i++;
  }
}

void SerialProtocolManager::WaitForBytes(uint8_t num_bytes, unsigned long timeout) {
  unsigned long startTime = millis();
  //Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)) {}
}