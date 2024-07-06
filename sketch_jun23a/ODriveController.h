#include <Arduino.h>
#include <ODriveCAN.h>
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#include <vector>


static HardwareCAN& can_intf = CAN;



struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

class ODriveController {
  public:
    ODriveController();
    void SetODrivesCallbacks();
    void SetUpODrivesConnection(int retryDelays = 100);
    void SetODrivePosition(int index, float pos);
    void SetODriveLimits(int index, float vel, float torque);
    void SetInitialStates();
    void OnCanMessage(const CanMsg& msg);

    static constexpr int DEFAULT_POS = 0;
    static constexpr int MAX_SPEED = 18;
    static constexpr int MAX_TORQUE = 10;

    ODriveUserData* GetUserData(int index) { return &oDrivesUserData[index]; }

  private:
    static void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
    static void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

    static constexpr int ODRIVES_AMOUNT = 2; // Todo: Expose it via intTotype
    std::vector<ODriveCAN*> oDrives; // = { &odrv0, &odrv1 };
    std::vector<ODriveUserData> oDrivesUserData;
    
};