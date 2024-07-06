#pragma once
#include "ODriveController.h"
#include "algorithm"


ODriveController::ODriveController(ODriveCAN* oDrive1, ODriveCAN* oDrive2) 
{
  oDrivesUserData.emplace_back(ODriveUserData());
  oDrivesUserData.emplace_back(ODriveUserData());
  oDrives.emplace_back(oDrive1);
  oDrives.emplace_back(oDrive2);
  //oDrives.reserve(ODRIVES_AMOUNT);
  for(int i = 0; i < ODRIVES_AMOUNT; ++i){
    //oDrives.emplace_back(wrap_can_intf(can_intf), i);
  }
  //oDrives[0] = oDrive1;
  //oDrives[1] = oDrive2;
}

void ODriveController::SetODrivesCallbacks()
{
  for(auto i = 0; i < ODRIVES_AMOUNT; ++i)
  {
    // Register callbacks for the heartbeat and encoder feedback messages
    oDrives[i]->onFeedback(onFeedback, &oDrivesUserData[i]);
    oDrives[i]->onStatus(onHeartbeat, &oDrivesUserData[i]);
  }
}

void ODriveController::SetUpODrivesConnection(int retryDelays){
  for(auto i = 0; i < ODRIVES_AMOUNT; ++i){
    while(!oDrivesUserData[i].received_heartbeat)
    {
      pumpEvents(can_intf);
      delay(retryDelays);
    }
    //Serial.println("found ODrive");
    // request bus voltage and current (1sec timeout)
    //Serial.println("attempting to read bus voltage and current");
    
    Get_Bus_Voltage_Current_msg_t vbus;
    if (!oDrives[i]->request(vbus, 1)) 
    {
      Serial.print("vbus request failed for: ");
      Serial.println(i);
      while (true)
        ;  // spin indefinitely
    }
  }

  for(auto i = 0; i < ODRIVES_AMOUNT; ++i)
  {
    while (oDrivesUserData[i].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) 
    {
      oDrives[i]->clearErrors();
      delay(1);
      oDrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
      // Pump events for 150ms. This delay is needed for two reasons;
      // 1. If there is an error condition, such as missing DC power, the ODrive might
      //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
      //    on the first heartbeat response, so we want to receive at least two
      //    heartbeats (100ms default interval).
      // 2. If the bus is congested, the setState command won't get through
      //    immediately but can be delayed.
      for (int i = 0; i < 15; ++i) 
      {
        delay(10);
        pumpEvents(can_intf);
      }
    }
  }
}

void ODriveController::SetODrivePosition(int index, float pos = DEFAULT_POS)
{
  oDrives[index]->setPosition(pos);
}

void ODriveController::SetODriveLimits(int index, float vel = MAX_SPEED, float torque = MAX_TORQUE)
{
  oDrives[index]->setLimits(std::min(vel, (float)MAX_SPEED), std::min(torque, (float)MAX_TORQUE));
}

void ODriveController::SetInitialStates()
{
  for(auto i = 0; i < ODRIVES_AMOUNT; ++i)
  {
    SetODrivePosition(i);
    delay(100);
    SetODrivePosition(i);
  }
}

void ODriveController::OnCanMessage(const CanMsg& msg) {
  for (auto odrive : oDrives) {
    onReceive(msg, *odrive);
  }
}

// Called every time a Heartbeat message arrives from the ODrive
void ODriveController::onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void ODriveController::onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}