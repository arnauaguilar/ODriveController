#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "SerialProtocolManager.h"
#include "ODriveController.h"


// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif

// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN


// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
//

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
//
//
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
long displayedFrames = 0;
float position = 0, velocity = 18, torque = 10;

ODriveCAN odrv0(wrap_can_intf(can_intf), 0);  // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), 1);  // Standard CAN message ID
//
SerialProtocolManager serialManager;
ODriveController oDriveController(&odrv0, &odrv1);

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  oDriveController.OnCanMessage(msg);
}


unsigned long lastFrameTime{ 0U };

unsigned long CalculateDeltaTime() {
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastFrameTime;
  lastFrameTime = currentTime;
  return deltaTime;
}

float lerp(float a, float b, float x) {
  return a + x * (b - a);
}


float currentPos, currentVel;
float targetRotation;
float startRotation;
float estimatedMicroseconds;
float estimatedFrames;
float elapsedFrames = 0.f;
float moveProgress = 2.f;
float step = 0.f;

void DrawScreen(String DebugMessage) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.print("Current Pos: ");
  display.println(currentPos);
  display.print("Current Vel: ");
  display.println(currentVel);
  display.print("Position: ");
  display.println(position);
  display.print("Velocity: ");
  display.println(velocity);
  display.print("Torque: ");
  display.println(torque);
  display.setCursor(0, 64 - 8);
  display.print(DebugMessage);


  display.display();
  displayedFrames++;
}

void SendRecievedMessage() {
  serialManager.Write(SerialProtocolManager::RECEIVED);
  serialManager.Write<int32_t>(currentPos*1000);
  serialManager.Write<int32_t>(currentVel*1000);
}

void RegisterSerialCallbacks() {
  serialManager.RegisterCallback(SerialProtocolManager::VELOCITY, []() {
    velocity = float(serialManager.Read<int32_t>()) / 1000.0f;
    if (DEBUG) {
      serialManager.Write(SerialProtocolManager::VELOCITY);
      serialManager.Write<int32_t>(velocity * 1000);
    }
    SendRecievedMessage();
  });

  serialManager.RegisterCallback(SerialProtocolManager::POSITION, []() {
    position = float(serialManager.Read<int32_t>()) / 1000.0f;
    if (DEBUG) {
      serialManager.Write(SerialProtocolManager::POSITION);
      serialManager.Write<int32_t>(position * 1000);
    }
    SendRecievedMessage();
  });

  serialManager.RegisterCallback(SerialProtocolManager::TORQUE, []() {
    torque = float(serialManager.Read<int32_t>()) / 1000.0f;
    if (DEBUG) {
      serialManager.Write(SerialProtocolManager::TORQUE);
      serialManager.Write<int32_t>(torque * 1000);
    }
    SendRecievedMessage();
  });

  serialManager.RegisterCallback(SerialProtocolManager::HELLO, []() {
    if (!serialManager.is_connected) {
      serialManager.is_connected = true;
      serialManager.Write(SerialProtocolManager::HELLO);
    } else {
      // If we are already connected do not send "hello" to avoid infinite loop
      serialManager.Write(SerialProtocolManager::ALREADY_CONNECTED);
    }
    SendRecievedMessage();
  });
}

void setup() {
  // Init Serial
  Serial.begin(115200);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  //Serial.println("Starting ODriveCAN demo");
  oDriveController.SetODrivesCallbacks();

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true)
      ;  // spin indefinitely
  }

   //Serial.println("Waiting for ODrive...");
  oDriveController.SetUpODrivesConnection();
  //Serial.println("ODrive running!");
  oDriveController.SetInitialStates();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  DrawScreen(String("Awaiting Connection"));
  RegisterSerialCallbacks();

  // Wait until the arduino is connected to master
  while (!serialManager.is_connected) {
    serialManager.Write(SerialProtocolManager::HELLO);
    serialManager.WaitForBytes(1, 1000);
    if (Serial.available() > 0) serialManager.GetMessagesFromSerial(position, velocity);
  }

  DrawScreen(String("SetupDone"));
  delay(2000);
  oDriveController.SetODrivePosition(0, 1000);
}



void loop() {
  unsigned long frameTime = CalculateDeltaTime();
  pumpEvents(can_intf);          // This is required on some platforms to handle incoming feedback CAN messages
  if (Serial.available() > 0) {  ///////////////////////move the if out of the get message from serial
    serialManager.GetMessagesFromSerial(position, velocity);
    oDriveController.SetODriveLimits(1, velocity, torque);
    delay(10);
    oDriveController.SetODrivePosition(1, position);
    //Get_Encoder_Estimates_msg_t feedback;
    //odrv0.getFeedback(feedback);
    //currentPos = feedback.Pos_Estimate;
    //currentVel = feedback.Vel_Estimate;
    DrawScreen(String(displayedFrames));  //////////////////////take out of here
  }

  // print position and velocity for Serial Plotter
  auto userData = oDriveController.GetUserData(1);
  if (userData->received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = userData->last_feedback;
    currentPos = feedback.Pos_Estimate;
    currentVel = feedback.Vel_Estimate;
    userData->received_feedback = false;
    //DrawScreen(String(displayedFrames*-1));
  }
  //if (odrv0_user_data.received_heartbeat) {
  // if (odrv0_user_data.last_heartbeat.Axis_Error != 0) {
  //   //Serial.print("State: ");
  //   //Serial.print(odrv0_user_data.last_heartbeat.Axis_State);
  //   //Serial.print("Errors: ");
  //   //Serial.println(odrv0_user_data.last_heartbeat.Axis_Error);
  //   DrawScreen(String(odrv0_user_data.last_heartbeat.Axis_State) + " : " + String(odrv0_user_data.last_heartbeat.Axis_Error));
  //   odrv0_user_data.received_heartbeat = false;
  // }
  //}
}