#include "indooruav_dji_driver/mavic_3t_data_subscription_manager.h"

#include <cmath>

#define USER_UTIL_UNUSED(x) ((void)(x))

//四元数
//提供了 FRD机体坐标系 和 NED地面惯性系 之间的四元数
//遵循hamilton convention: q0 = w, q1 = x, q2 = y, q3 = z
//我自己做了一些转换
//把机体坐标系FRD和地面惯性系NED之间的绝对欧拉角，转换成了
//当前姿态与初始姿态（启动订阅话题那一刻的姿态）之间的相对欧拉角，（也是FRD机体坐标系下的姿态，右偏航为正，抬头为正，右滚转为正）
T_DjiReturnCode Dji_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *) data;
    USER_UTIL_UNUSED(dataSize);

    //1.由四元数转换出绝对欧拉角
    static dji_f64_t roll_absolute;
    static dji_f64_t pitch_absolute;
    static dji_f64_t yaw_absolute;
    pitch_absolute = (dji_f64_t) asinf(-2 * quaternion->q1 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q2) * 57.3;
    roll_absolute  = (dji_f64_t) atan2f(2 * quaternion->q2 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q1, -2 * quaternion->q1 * quaternion->q1 - 2 * quaternion->q2 * quaternion->q2 + 1) * 57.3;
    yaw_absolute   = (dji_f64_t) atan2f(2 * quaternion->q1 * quaternion->q2 + 2 * quaternion->q0 * quaternion->q3, -2 * quaternion->q2 * quaternion->q2 - 2 * quaternion->q3 * quaternion->q3 + 1) * 57.3;

    //2.保存第一次的绝对欧拉角
    static int once = 0;
    static dji_f64_t roll_absolute_init;
    static dji_f64_t pitch_absolute_init;
    static dji_f64_t yaw_absolute_init;
    if (once == 0) {
        once++;
        roll_absolute_init  = roll_absolute;
        pitch_absolute_init = pitch_absolute;
        yaw_absolute_init   = yaw_absolute;
    }

    //3.由绝对欧拉角转换出相对欧拉角
    dji_f64_t roll_relative = roll_absolute - roll_absolute_init;
    dji_f64_t pitch_relative = pitch_absolute - pitch_absolute_init;
    dji_f64_t yaw_relative = yaw_absolute - yaw_absolute_init;
    
    //4.相对欧拉角标准化
    while(roll_relative <= -180.0) {
        roll_relative = roll_relative + 360.0;
    }
    while(roll_relative >= 180.0) {
        roll_relative = roll_relative - 360.0;
    }
    while(pitch_relative <= -180.0) {
        pitch_relative = pitch_relative + 360.0;
    }
    while(pitch_relative >= 180.0) {
        pitch_relative = pitch_relative - 360.0;
    }
    while(yaw_relative <= -180.0) {
        yaw_relative = yaw_relative + 360.0;
    }
    while(yaw_relative >= 180.0) {
        yaw_relative = yaw_relative - 360.0;
    }

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive quaternion data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("quaternion: %f %f %f %f.", quaternion->q0, quaternion->q1, quaternion->q2, quaternion->q3);
        USER_LOG_INFO("euler angles: pitch_absolute = %.2f roll_absolute = %.2f yaw_absolute = %.2f.", pitch_absolute, roll_absolute, yaw_absolute);
        USER_LOG_INFO("euler angles: pitch_relative = %.2f roll_relative = %.2f yaw_relative = %.2f.\n", pitch_relative, roll_relative, yaw_relative);
        // DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, yaw, roll);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

//相对于NEU地面坐标系的速度 m/s
//速度数据是由GNSS、气压计、视觉里程计融合得到的，室内环境下只能依赖视觉里程计
//这里如果希望拿到NED地面坐标系的速度数据，直接对z轴速度取反就可以了
T_DjiReturnCode Dji_FcSubscriptionReceiveVelocityCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionVelocity *velocity = (T_DjiFcSubscriptionVelocity *) data;
    USER_UTIL_UNUSED(dataSize);

    dji_f32_t v_x = velocity->data.x;
    dji_f32_t v_y = velocity->data.y;
    //对z轴速度取反，拿到NED地面坐标系下的数据
    dji_f32_t v_z = -1.0 * velocity->data.z;
    velocity->health;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive Velocity data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("v_x: %f, v_y: %f, v_z: %f", v_x, v_y, v_z);
        USER_LOG_INFO("health: %d\n", velocity->health); //这个int数据一直是1
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

//相对于NED地面坐标系的融合角速度 rad/s
//大概只有z轴的角速度是有用的，它反映了无人机转向的速度
T_DjiReturnCode Dji_FcSubscriptionReceiveAngularRateFusionedCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAngularRateFusioned *angularFusioned = (T_DjiFcSubscriptionAngularRateFusioned *) data;
    USER_UTIL_UNUSED(dataSize);

    angularFusioned->x;
    angularFusioned->y;
    angularFusioned->z;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive AngularRateFusioned data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("angular_x: %f", angularFusioned->x);
        USER_LOG_INFO("angular_y: %f", angularFusioned->y);
        USER_LOG_INFO("angular_z: %f", angularFusioned->z);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

//遥控摇杆信息
//我们只用美国手
//左摇杆前推到底，throttle从0增大到9999
//左摇杆后推到底，throttle从0减小到-9999
//左摇杆右推到底，yaw从0增大到9999
//左摇杆左推到底，yaw从0减小到-9999
//右摇杆前推到底，roll从0增大到9999
//右摇杆后推到底，roll从0减小到-9999
//右摇杆右推到底，pitch从0增大到9999
//右摇杆右推到底，pitch从0减小到-9999
//中间的FNS按钮，处于中间N时，mode为8000
//中间的FNS按钮，处于右边S时，mode为0
//中间的FNS按钮，处于左边F时，mode为-8000
//gear这个字段，由于没有起落架功能，所以一直为0
T_DjiReturnCode Dji_FcSubscriptionReceiveRCCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionRC *RC = (T_DjiFcSubscriptionRC *) data;
    USER_UTIL_UNUSED(dataSize);

    RC->throttle;
    RC->roll;
    RC->pitch;
    RC->yaw;
    RC->mode; 
    // RC->gear;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive RC data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("roll: %d", RC->roll);
        USER_LOG_INFO("pitch: %d", RC->pitch);
        USER_LOG_INFO("yaw: %d", RC->yaw);
        USER_LOG_INFO("throttle: %d", RC->throttle);
        USER_LOG_INFO("mode: %d", RC->mode);
        USER_LOG_INFO("gear: %d\n", RC->gear);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

//云台角度
//云台角度的参考坐标系是NED地面坐标系，遵循右手定则，抬头pitch为正低头pitch为负
//云台角度的订阅，只支持频率50Hz
//这里控制好无人机的yaw，云台与无人机之间保持0yaw，只关心pitch俯仰角就可以了
//云台的yaw在跟踪无人机的yaw时，完全是一个一阶惯性环节，完全没有超调
//云台的yaw(相对于地面NED坐标系)和无人机的yaw(相对于地面NED坐标系)误差不会超过0.5度
T_DjiReturnCode Dji_FcSubscriptionReceiveGimbalAnglesCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionGimbalAngles *gimbalAngles = (T_DjiFcSubscriptionGimbalAngles *) data;
    USER_UTIL_UNUSED(dataSize);
    
    dji_f32_t gimbalPitch = gimbalAngles->x;
    dji_f32_t gimbalRoll  = gimbalAngles->y;
    dji_f32_t gimbalYaw   = gimbalAngles->z;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive GimbalAngles data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("gimbalRoll: %f", gimbalRoll);
        USER_LOG_INFO("gimbalPitch: %f", gimbalPitch);
        USER_LOG_INFO("gimbalYaw: %f\n", gimbalYaw);
    }
#endif 

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

//飞行状态
// typedef enum {
//     DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED = 0, /*!< Aircraft is on ground and motors are still. */
//     DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND = 1, /*!< Aircraft is on ground but motors are rotating. */
//     DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR = 2, /*!< Aircraft is in air. */
// } E_DjiFcSubscriptionFlightStatus;
T_DjiReturnCode Dji_FcSubscriptionReceiveStatusFlightCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionFlightStatus *flightStatus = (T_DjiFcSubscriptionFlightStatus *) data;
    USER_UTIL_UNUSED(dataSize);

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive StatusFlight data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("flightStatus: %d", *flightStatus);
        if (*flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED) {    //0
            USER_LOG_INFO("Aircraft is on ground and motors are still.\n");
        }
        if (*flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND) { //1
            USER_LOG_INFO("Aircraft is on ground but motors are rotating.\n");
        }
        if (*flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {    //2
            USER_LOG_INFO("Aircraft is in air.\n");
        }
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

//飞行模式状态
//提供更加细粒度的飞行状态，与StatusFlight话题一起订阅，能对飞行状态有更好的认知
/**
 * @attention 放在桌面上进行测试时，打印为6，这个状态的变化还需要好好测试一下
 */
T_DjiReturnCode Dji_FcSubscriptionReceiveStatusDisplayModeCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionDisplaymode *displayMode = (T_DjiFcSubscriptionDisplaymode *) data;
    USER_UTIL_UNUSED(dataSize);
// typedef enum {
//     /*! This mode requires the user to manually
//      * control the aircraft to remain stable in air. */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_MANUAL_CTRL = 0,
//     /*! In this mode, the aircraft can keep
//      * attitude stabilization and only use the
//      * barometer for positioning to control the altitude. <br>
//      * The aircraft can not autonomously locate and hover stably.*/
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE = 1,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_2 = 2,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_3 = 3,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_4 = 4,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_5 = 5,
//     /*! The aircraft is in normal GPS mode. <br>
//      * In normal GPS mode, the aircraft can
//      * autonomously locate and hover stably. <br>
//      *  The sensitivity of the aircraft to the
//      *  command response is moderate.
//      */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS = 6,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_7 = 7,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_8 = 8,
//     /*! In hotpoint mode */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_HOTPOINT_MODE = 9,
//     /*! In this mode, user can push the throttle
//      * stick to complete stable take-off. */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF = 10,
//     /*! In this mode, the aircraft will autonomously
//      * start motor, ascend and finally hover. */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF = 11,
//     /*! In this mode, the aircraft can land autonomously. */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING = 12,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_13 = 13,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_14 = 14,
//     /*! In this mode, the aircraft can antonomously return the
//      * last recorded Home Point. <br>
//      * There are three types of this mode: Smart RTH(Return-to-Home),
//      * Low Batterry RTH, and Failsafe RTTH.  */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME = 15,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_16 = 16,
//     /*! In this mode, the aircraft is controled by SDK API. <br>
//      * User can directly define the control mode of horizon
//      * and vertical directions and send control datas to aircraft. */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_SDK_CTRL = 17,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_18 = 18,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_19 = 19,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_20 = 20,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_21 = 21,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_22 = 22,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_23 = 23,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_24 = 24,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_25 = 25,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_26 = 26,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_27 = 27,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_28 = 28,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_29 = 29,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_30 = 30,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_31 = 31,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_32 = 32,
//     /*! drone is forced to land, might due to low battery */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING = 33,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_34 = 34,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_35 = 35,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_36 = 36,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_37 = 37,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_38 = 38,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_39 = 39,
//     /*! drone will search for the last position where the rc is not lost */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_SEARCH_MODE = 40,
//     /*! Mode for motor starting. <br>
//      * Every time user unlock the motor, this will be the first mode. */
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START = 41,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_42 = 42,
//     DJI_FC_SUBSCRIPTION_DISPLAY_MODE_RESERVED_43 = 42,
// } E_DjiFcSubscriptionDisplayMode;
    displayMode;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive StatusDisplayMode data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("displayMode: %d\n", *displayMode);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

//电机启动错误码
/**
 * @attention 放在桌面上进行测试时，没有任何内容被打印
 */
T_DjiReturnCode Dji_FcSubscriptionReceiveStatusMotorStartErrorCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionMotorStartError *motorStartError = (T_DjiFcSubscriptionMotorStartError *) data;
    USER_UTIL_UNUSED(dataSize);

    motorStartError;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive StatusMotorStartError data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("motorStartError: %d\n", *motorStartError);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

//设备控制信息
//订阅这个话题可以知道是谁在控制无人机，RC/PSDK，以及控制权限被改变的原因是什么
T_DjiReturnCode Dji_FcSubscriptionReceiveControlDeviceCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionControlDevice *controlDevice = (T_DjiFcSubscriptionControlDevice *) data;
    USER_UTIL_UNUSED(dataSize);

// typedef enum {
//     DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_RC = 0, /*!< Authority is in remote control */
//     DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_MSDK = 1, /*!< Authority is in MSDK */
//     DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_PSDK = 4, /*!< Authority is in PSDK */
//     DJI_FC_SUBSCRIPTION_CONTROL_AUTHORITY_DOCK = 5, /*!< Authority is in dock */
// } E_DJIFcSubscriptionConstrolAuthority;
    controlDevice->controlAuthority;
// typedef enum {
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_UNKNOWN = 0, /*!< Reason unknown */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_MSDK_REQUEST = 1, /*!< Contro authority changed by MSDK request. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_USER_REQUEST = 2, /*!< Contro authority changed by user request. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_PSDK_REQUEST = 3, /*!< Contro authority changed by PSDK request. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_RC_LOST = 4, /*!< Contro authority changed for remote control lost. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_RC_NOT_P_MODE = 5, /*!< Contro authority changed for remote control not in P mode. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_RC_SWITCH = 6, /*!< Contro authority changed for remote control switching mode. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_RC_PAUSE_STOP = 7, /*!< Contro authority changed for remote control stop key paused. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_RC_ONE_KEY_GO_HOME = 8, /*!< Contro authority changed for remote control go-home key paused. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_BATTERY_LOW_GO_HOME = 9, /*!< Contro authority changed for remote control go-home key paused. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_BATTERY_SUPER_LOW_LANDING = 10, /*!< Contro authority changed for going home caused by low batter power. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_PSDK_LOST = 11, /*!< Contro authority changed for PSDK lost. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_NEAR_BOUNDARY = 13, /*!< Contro authority changed for nearing boundary. */
//     DJI_FC_SUBSCRIPTION_AUTHORITY_CHANGE_REASON_AIRPORT_REQUEST = 14, /*!< Contro authority changed by airport request. */
// } E_DJIFcSubscriptionAuthorityChangeReason;
    controlDevice->controlAuthorityChangeReason;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive ControlDevice data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("controlAuthority: %d", controlDevice->controlAuthority);
        USER_LOG_INFO("controlAuthorityChangeReason: %d\n", controlDevice->controlAuthorityChangeReason);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;   
}

//带标记遥控摇杆信息
//测试发现，如果关闭RC，3秒后logicConnected和skyConnected都会变成0
T_DjiReturnCode Dji_FcSubscriptionReceiveRCWithFlagDataCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionRCWithFlagData *rcWithFlagData = (T_DjiFcSubscriptionRCWithFlagData *) data;
    USER_UTIL_UNUSED(dataSize);

// typedef struct RCWithFlagData {
//     dji_f32_t pitch;       /*!< down = -0.999, middle = 0.000, up   =0.999 */
//     dji_f32_t roll;        /*!< left = -0.999, middle = 0.000, right=0.999 */
//     dji_f32_t yaw;         /*!< left = -0.999, middle = 0.000, right=0.999 */
//     dji_f32_t throttle;    /*!< down = -0.999, middle = 0.000, up   =0.999 */
//     struct {
//         uint8_t logicConnected: 1;  /*!< 0 if sky or ground side is disconnected for 3 seconds   */
//         uint8_t skyConnected: 1;  /*!< Sky side is connected, i.e., receiver is connected to FC */
//         uint8_t groundConnected: 1;  /*!< Ground side is connected, i.e., RC is on and connected to FC */
//         uint8_t appConnected: 1;  /*!< Mobile App is connected to RC */
//         uint8_t reserved: 4;
//     } flag;
// } T_DjiFcSubscriptionRCWithFlagData;

    rcWithFlagData->roll;
    rcWithFlagData->pitch;
    rcWithFlagData->yaw;
    rcWithFlagData->throttle;

    rcWithFlagData->flag.logicConnected;
    rcWithFlagData->flag.skyConnected;
    rcWithFlagData->flag.groundConnected;
    rcWithFlagData->flag.appConnected;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive RCWithFlagData data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("throttle: %f", rcWithFlagData->throttle);
        USER_LOG_INFO("yaw: %f", rcWithFlagData->yaw);
        USER_LOG_INFO("roll: %f", rcWithFlagData->roll);
        USER_LOG_INFO("pitch: %f", rcWithFlagData->pitch);
        USER_LOG_INFO("flag.logicConnected: %d", rcWithFlagData->flag.logicConnected);
        USER_LOG_INFO("flag.skyConnected: %d", rcWithFlagData->flag.skyConnected);
        USER_LOG_INFO("flag.groundConnected: %d", rcWithFlagData->flag.groundConnected);
        USER_LOG_INFO("flag.appConnected: %d\n", rcWithFlagData->flag.appConnected);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;   
}

//飞行异常信息
//订阅这个话题可以实现一些飞行过程中的异常处理
T_DjiReturnCode Dji_FcSubscriptionReceiveFlightAnomalyCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionFlightAnomaly *flightAnomaly = (T_DjiFcSubscriptionFlightAnomaly *) data;
    USER_UTIL_UNUSED(dataSize);

// typedef struct FlightAnomaly {
//     uint32_t impactInAir: 1;  /*!< 0: No impact,                      1: Impact happens in Air */
//     uint32_t randomFly: 1;  /*!< 0: Normal,                         1: Randomly fly in GPS mode without stick input*/
//     uint32_t heightCtrlFail: 1;  /*!< 0: Height control normal,          1: Height control failed */
//     uint32_t rollPitchCtrlFail: 1;  /*!< 0: Tilt control normal,            1: Tilt control failed */
//     uint32_t yawCtrlFail: 1;  /*!< 0: Yaw control normal,             1: Yaw control failed */
//     uint32_t aircraftIsFalling: 1;  /*!< 0: Aircraft is not falling,        1: Aircraft is falling */
//     uint32_t strongWindLevel1: 1;  /*!< 0: Wind is under big wind level 1, 1: wind is stronger than  big wind level 1*/
//     uint32_t strongWindLevel2: 1;  /*!< 0: Wind is under big wind level 2, 1: wind is stronger than  big wind level 2*/
//     uint32_t compassInstallationError: 1;  /*!< 0: Compass install right,          1: Compass install error */
//     uint32_t imuInstallationError: 1;  /*!< 0: IMU install right,              1: IMU install error */
//     uint32_t escTemperatureHigh: 1;  /*!< 0: ESC temperature is normal,      1: ESC temperature is high */
//     uint32_t atLeastOneEscDisconnected: 1;  /*!< 0: No ESC disconnected,            1: At least one ESC is disconnected */
//     uint32_t gpsYawError: 1;  /*!< 0: No GPS yaw error,               1: GPS yaw error */
//     uint32_t reserved: 19;
// } T_DjiFcSubscriptionFlightAnomaly;
    flightAnomaly->impactInAir;
    flightAnomaly->randomFly;
    flightAnomaly->heightCtrlFail;
    flightAnomaly->rollPitchCtrlFail;
    flightAnomaly->yawCtrlFail;
    flightAnomaly->aircraftIsFalling;
    flightAnomaly->strongWindLevel1;
    flightAnomaly->strongWindLevel2;
    flightAnomaly->compassInstallationError;
    flightAnomaly->imuInstallationError;
    flightAnomaly->escTemperatureHigh;
    flightAnomaly->atLeastOneEscDisconnected;
    flightAnomaly->gpsYawError;

    //判断每一位字段是不是1，如果是1立刻进行异常处理

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive  FlightAnomaly data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("impactInAir: %d", flightAnomaly->impactInAir);
        USER_LOG_INFO("randomFly: %d", flightAnomaly->randomFly);
        USER_LOG_INFO("heightCtrlFail: %d", flightAnomaly->heightCtrlFail);
        USER_LOG_INFO("rollPitchCtrlFail: %d", flightAnomaly->rollPitchCtrlFail);
        USER_LOG_INFO("yawCtrlFail: %d", flightAnomaly->yawCtrlFail);
        USER_LOG_INFO("aircraftIsFalling: %d", flightAnomaly->aircraftIsFalling);
        // USER_LOG_INFO("strongWindLevel1: %d", flightAnomaly->strongWindLevel1);
        // USER_LOG_INFO("strongWindLevel2: %d", flightAnomaly->strongWindLevel2);
        USER_LOG_INFO("compassInstallationError: %d", flightAnomaly->compassInstallationError);
        USER_LOG_INFO("imuInstallationError: %d", flightAnomaly->imuInstallationError);
        USER_LOG_INFO("escTemperatureHigh: %d", flightAnomaly->escTemperatureHigh);
        USER_LOG_INFO("atLeastOneEscDisconnected: %d", flightAnomaly->atLeastOneEscDisconnected);
        // USER_LOG_INFO("gpsYawError: %d\n", flightAnomaly->gpsYawError);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;   
}

//笛卡尔坐标位置
//无人机上电后，在机体中心位置建立一个坐标原点，以此建立NED地面坐标系
//不能纯依赖这个话题做精确导航，必须额外检测坐标连续性或GPS/RTK辅助
//这是视觉特征点和惯性导航融合得出的位置信息
//可以用来进行相对控制
T_DjiReturnCode Dji_FcSubscriptionReceivePositionVoCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionPositionVO *positionVO = (T_DjiFcSubscriptionPositionVO *) data;
    USER_UTIL_UNUSED(dataSize);

    positionVO->x;      //北（尽可能
    positionVO->xHealth;
    positionVO->y;      //东（尽可能
    positionVO->yHealth;
    positionVO->z;      //地（尽可能
    positionVO->zHealth;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive PositionVO data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("xHealth: %d", positionVO->xHealth);
        USER_LOG_INFO("x: %f", positionVO->x);
        USER_LOG_INFO("yHealth: %d", positionVO->yHealth);
        USER_LOG_INFO("y: %f", positionVO->y);        
        USER_LOG_INFO("zHealth: %d", positionVO->zHealth);
        USER_LOG_INFO("z: %f\n", positionVO->z);        
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;  
}

//1号电池信息
T_DjiReturnCode Dji_FcSubscriptionReceiveBatterySingleInfoIndex1Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionSingleBatteryInfo *singleBatteryInfo = (T_DjiFcSubscriptionSingleBatteryInfo *) data;
    USER_UTIL_UNUSED(dataSize);

// typedef struct BatterySingleInfo {
//     uint8_t reserve;
//     uint8_t batteryIndex;
//     int32_t currentVoltage;          /*!< uint: mV. */
//     int32_t currentElectric;         /*!< uint: mA. */
//     uint32_t fullCapacity;           /*!< uint: mAh. */
//     uint32_t remainedCapacity;       /*!< uint: mAh. */
//     int16_t batteryTemperature;      /*!< uint: 0.1℃. */
//     uint8_t cellCount;
//     uint8_t batteryCapacityPercent;  /*!< uint: %. */
//     T_DjiFcSubscriptionSingleBatteryState batteryState;
//     uint8_t reserve1;
//     uint8_t reserve2;
//     uint8_t SOP;                     /*!< Relative power percentage. */
// } T_DjiFcSubscriptionSingleBatteryInfo;

    singleBatteryInfo->currentVoltage;
    singleBatteryInfo->currentElectric;
    singleBatteryInfo->fullCapacity;
    singleBatteryInfo->remainedCapacity;
    singleBatteryInfo->batteryTemperature; 
    singleBatteryInfo->batteryCapacityPercent;

#if 0
    if (USER_FC_SUBSCRIPTION_DATA_SHOW == true) {
        USER_LOG_INFO("receive PositionVO data.");
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond, timestamp->microsecond);
        USER_LOG_INFO("data size: %d", dataSize);
        USER_LOG_INFO("batteryTemperature: %d", singleBatteryInfo->batteryTemperature);
        USER_LOG_INFO("batteryCapacityPercent: %d\n", singleBatteryInfo->batteryCapacityPercent);
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;  
}

DjiDataSubscriptionManager::DjiDataSubscriptionManager() {
    this->usability_ = false;

    T_DjiReturnCode djiStat;
    
    djiStat = this->Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        USER_LOG_ERROR("DjiDataSubscriptionManager.Init() failed.");
        return;
    }

    djiStat = this->RunSub();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        USER_LOG_ERROR("DjiDataSubscriptionManager.RunSub() failed.");
        return;
    }
    
    this->usability_ = true;
}

bool DjiDataSubscriptionManager::CheckUsability() const {
    return this->usability_;
}

DjiErrorCode DjiDataSubscriptionManager::Init() {
    T_DjiReturnCode djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

DjiErrorCode DjiDataSubscriptionManager::RunSub() {
    T_DjiReturnCode djiStat;

    //1.订阅四元数（订阅相对于初始姿态的姿态角）
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, 
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic Quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //2.订阅速度
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, 
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveVelocityCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic Velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //3.订阅融合角速度
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveAngularRateFusionedCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic AngularRateFusioned error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //4.订阅遥控遥感信息
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveRCCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic RC error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //5.订阅云台角度
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, 
                                               Dji_FcSubscriptionReceiveGimbalAnglesCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic GimbalAngles error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //6.订阅飞行状态
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveStatusFlightCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic StatusFlight error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //7.订阅飞行模式状态
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveStatusDisplayModeCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic StatusDislayMode error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    // //8.订阅电机启动错误码
    // djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR,
    //                                            DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
    //                                            Dji_FcSubscriptionReceiveStatusMotorStartErrorCallback);
    // if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic StatusMotorStartError error.");
    //     return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    // }

    //9.订阅设备控制信息
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveControlDeviceCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic ControlDevice error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //10.订阅带标记摇杆信息
    //带标记遥控摇杆信息
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveRCWithFlagDataCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic RCWithFlagData error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //11.订阅飞行异常信息
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceiveFlightAnomalyCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic FlightAnomaly error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //12.订阅笛卡尔坐标位置
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 
                                               Dji_FcSubscriptionReceivePositionVoCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic PositionVO error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    //13.订阅1号电池信息
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, 
                                               Dji_FcSubscriptionReceiveBatterySingleInfoIndex1Callback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic BatterySingleInfoIndex1 error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
