#ifndef INDOORUAV_DJI_DRIVER_MAVIC_3T_DATA_SUBSCRIPTION_MANAGER_H_
#define INDOORUAV_DJI_DRIVER_MAVIC_3T_DATA_SUBSCRIPTION_MANAGER_H_

#include <cstdint>

#include <dji_error.h>
#include <dji_fc_subscription.h>
#include <dji_logger.h>

#ifdef __cplusplus
extern "C" {
#endif

T_DjiReturnCode Dji_FcSubscriptionReceiveQuaternionCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveVelocityCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveAngularRateFusionedCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveRCCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveGimbalAnglesCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveStatusFlightCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveStatusDisplayModeCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveStatusMotorStartErrorCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveControlDeviceCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveRCWithFlagDataCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveFlightAnomalyCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceivePositionVoCallback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);
T_DjiReturnCode Dji_FcSubscriptionReceiveBatterySingleInfoIndex1Callback(
    const uint8_t* data, uint16_t data_size,
    const T_DjiDataTimestamp* timestamp);

#ifdef __cplusplus
}
#endif

class DjiDataSubscriptionManager {
 public:
  DjiDataSubscriptionManager();
  DjiDataSubscriptionManager(const DjiDataSubscriptionManager& other) = delete;
  DjiDataSubscriptionManager& operator=(const DjiDataSubscriptionManager& other) = delete;
  ~DjiDataSubscriptionManager() = default;

  bool CheckUsability() const;

 private:
  DjiErrorCode Init();
  DjiErrorCode RunSub();

  bool usability_;
};

#endif  // INDOORUAV_DJI_DRIVER_MAVIC_3T_DATA_SUBSCRIPTION_MANAGER_H_
