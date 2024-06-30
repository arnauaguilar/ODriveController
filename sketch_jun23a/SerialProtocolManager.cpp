#pragma once
#include "SerialProtocolManager.h"

void SerialProtocolManager::GetMessagesFromSerial(float& var1,float& var2) {

  // The first byte received is the instruction
  Order order_received = Read<Order>();

  if (order_received == HELLO) {
    // If the cards haven't say hello, check the connection
    if (!is_connected) {
      is_connected = true;
      Write(HELLO);
    } else {
      // If we are already connected do not send "hello" to avoid infinite loop
      Write(ALREADY_CONNECTED);
    }
  } else if (order_received == ALREADY_CONNECTED) {
    is_connected = true;
  } else {
    switch (order_received) {
      case STOP:
        {
          if (DEBUG) {
            Write(STOP);
          }
          break;
        }
      case VAR1:
        {
          var1 = float(Read<int32_t>())/1000.0f;
          if (DEBUG) {
            Write(VAR1);
            Write<int32_t>(var1*1000);
          }
          break;
        }
      case VAR2:
        {
          var2 = float(Read<int32_t>())/1000.0f;
          if (DEBUG) {
            Write(VAR2);
            Write<int32_t>(var2*1000);
          }
          break;
        }
      // Unknown order
      default:
        Write(ERROR);
        Write<int16_t>(404);
        return;
    }
  }
  Write(RECEIVED);  // Confirm the reception
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