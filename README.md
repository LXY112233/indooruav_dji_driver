# indooruav_dji_driver

`indooruav_dji_driver` 是一个面向 ROS1 的 DJI Payload SDK 驱动包，当前主要服务于 `DJI Mavic 3T` 的机载接入场景。它把飞控、云台、相机的部分能力封装成 ROS 接口，方便上层自主飞行、状态机、任务调度或实验程序直接调用。

当前实现以 `mavic_3t_driver_node` 为核心，完成了以下几件事：

- 初始化 DJI Payload SDK 运行环境
- 订阅 DJI 飞控内部状态数据，供控制逻辑和安全切换使用
- 提供起飞、降落、云台俯仰、相机拍照等 ROS Service
- 提供两种控制方式
  - 基于两路 `nav_msgs/Odometry` 的位置控制模式
  - 基于 `geometry_msgs/TwistStamped` 的机体系速度控制模式

这个包更适合作为“DJI PSDK 到 ROS 的底层适配层”，而不是完整的状态桥接包。当前版本不会把 DJI 内部订阅数据发布成标准 ROS 话题，也不提供完整导航状态输出。

## 功能概览

### 1. 飞控控制

- 提供起飞服务 `TakeOffService`
- 提供降落服务 `LandingService`
- 支持机体系 `FLU` 速度控制输入
- 支持基于期望/当前 `Odometry` 的闭环位置控制
- 起飞后会启动 RC 摇杆检测逻辑，在 RC 和 PSDK 之间自动切换控制权

### 2. 云台控制

- 提供云台俯仰角服务
- 云台初始化时会设置为 `DJI_GIMBAL_MODE_YAW_FOLLOW`
- 俯仰角请求会被限制在 `[-90, 0]` 度

### 3. 相机控制

- 提供单张拍照服务
- 拍照前可选择设置一次光学变焦
- 当前代码将光学变焦限制为 `[1.0, 6.9]`

### 4. 内部数据订阅

驱动启动后会通过 DJI PSDK 订阅多类飞控内部 topic，例如姿态、速度、RC、控制权、飞行模式、异常信息、电池信息等。这些数据当前主要用于：

- 起飞过程状态判断
- RC/PSDK 控制权切换
- 云台控制中的姿态参考
- 运行期安全判断与后续扩展

注意：这些订阅目前不会直接桥接为 ROS publisher。

## 项目结构

```text
indooruav_dji_driver/
├── config/
│   └── mavic_3t_config.yaml
├── include/
│   ├── dependences/
│   └── indooruav_dji_driver/
├── launch/
│   └── mavic_3t_driver.launch
├── node/
│   └── mavic_3t_driver.cpp
├── psdk_lib/
│   ├── include/
│   └── lib/
├── src/
│   ├── dependences/
│   └── indooruav_dji_driver/
└── srv/
```

主要文件说明：

- `node/mavic_3t_driver.cpp`：主节点入口
- `src/dependences/application.cpp`：PSDK 平台环境初始化
- `src/indooruav_dji_driver/mavic_3t_flight_controller.cpp`：飞控控制与位置控制逻辑
- `src/indooruav_dji_driver/mavic_3t_gimbal_controller.cpp`：云台控制
- `src/indooruav_dji_driver/mavic_3t_camera_controller.cpp`：相机控制
- `src/indooruav_dji_driver/mavic_3t_data_subscription_manager.cpp`：DJI 内部数据订阅
- `config/mavic_3t_config.yaml`：运行时 ROS 参数

## 运行环境与依赖

### 软件依赖

- Ubuntu + ROS1 Catkin 工作空间
- `roscpp`
- `geometry_msgs`
- `nav_msgs`
- `message_filters`
- `std_msgs`
- `message_generation`
- `message_runtime`

### DJI Payload SDK

仓库已经包含 PSDK 头文件和静态库，`CMakeLists.txt` 会根据 `CMAKE_SYSTEM_PROCESSOR` 自动选择对应架构的库文件。目前支持：

- `x86_64` / `amd64`
- `aarch64` / `arm64`
- `armv7l` / `armv7` / `arm`

如果当前平台不在上述列表中，编译会直接失败。

### 硬件连接方式

当前工程在 [`include/dependences/dji_sdk_config.h`](/home/lxy/indooruav_ws/src/indooruav_dji_driver/include/dependences/dji_sdk_config.h) 中固定为：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_ONLY_UART
```

也就是说，当前默认走 UART 连接。

串口设备名在 [`include/dependences/hal_uart.h`](/home/lxy/indooruav_ws/src/indooruav_dji_driver/include/dependences/hal_uart.h) 中定义，默认值为：

- `LINUX_UART_DEV1 = "/dev/ttyTHS0"`
- `LINUX_UART_DEV2 = "/dev/ttyACM0"`

如果你的机载平台、转接板或接线方式不同，需要按实际环境修改。

### 挂载位置要求

程序启动时会检查设备安装位置或适配器类型。当前代码要求至少满足以下之一：

- 挂载在 extension port
- 使用 E-Port V2 ribbon cable
- 使用 SkyPort V3

如果不满足，节点会在启动阶段直接报错退出。

### Payload 口位

当前云台和相机控制都固定使用：

```c
DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1
```

如果你的设备不在 `payload port no1`，需要同时修改相机和云台控制代码中的挂载口常量。

## 编译前准备

### 1. 配置本地 DJI 凭据

先基于模板创建本地头文件：

```bash
cp include/dependences/dji_sdk_app_info_local.h.example \
   include/dependences/dji_sdk_app_info_local.h
```

然后填写本地文件 `include/dependences/dji_sdk_app_info_local.h` 中的真实内容。程序启动时会实际读取以下宏：

- `USER_APP_NAME`
- `USER_APP_ID`
- `USER_APP_KEY`
- `USER_APP_LICENSE`
- `USER_DEVELOPER_ACCOUNT`
- `USER_BAUD_RATE`
- `USER_PSDK_ALIAS`
- `USER_PSDK_SERIAL_NUMBER`
- `USER_PSDK_LOG_PATH`

如果没有填写真实值，代码通常仍然可以编译，但运行时会在应用初始化阶段报错并退出。

仓库内的 [`include/dependences/dji_sdk_app_info.h`](/home/lxy/indooruav_ws/src/indooruav_dji_driver/include/dependences/dji_sdk_app_info.h) 只保留占位符；本地文件 `include/dependences/dji_sdk_app_info_local.h` 已在 `.gitignore` 中忽略。

### 2. 检查串口与波特率

需要同时确认两处配置：

- 串口设备路径：[`include/dependences/hal_uart.h`](/home/lxy/indooruav_ws/src/indooruav_dji_driver/include/dependences/hal_uart.h)
- 波特率：`USER_BAUD_RATE`

### 3. 检查串口权限

运行用户需要有目标串口设备的访问权限，否则 PSDK 初始化会失败。

### 4. 检查日志路径

日志根路径由 `USER_PSDK_LOG_PATH` 指定。默认模板值为：

```text
Logs/DJI
```

控制台日志会输出到终端，本地日志文件会写到 `Logs/` 目录下。

## 编译与启动

在 catkin 工作空间根目录执行：

```bash
catkin_make --pkg indooruav_dji_driver
source devel/setup.bash
```

或者直接编译整个工作空间：

```bash
catkin_make
source devel/setup.bash
```

启动命令：

```bash
roslaunch indooruav_dji_driver mavic_3t_driver.launch
```

该 launch 文件会：

- 加载 `config/mavic_3t_config.yaml`
- 启动可执行文件 `mavic_3t_driver_node`

默认 ROS 节点名为：

```text
/mavic_3t_driver
```

## 控制模式

### 1. 位置控制模式

默认情况下，`parameters.position_control.enabled=true`，驱动会启用基于 `Odometry` 的位置控制逻辑，而不是直接订阅速度控制话题。

该模式下会订阅两路输入：

- `desired_odometry`：期望位置与姿态
- `current_odometry`：当前位置与姿态

两路消息都要求是 `nav_msgs/Odometry`，并且应处在同一个参考系中。当前代码假设它们都表达为同一个“世界系 / 雷达参考系下的 `FLU`”。

控制流程如下：

1. 使用 `message_filters::Synchronizer` 对两路 `Odometry` 做近似时间同步
2. 读取 `pose.pose.position` 和四元数姿态中的 `yaw`
3. 分别计算 `x / y / z / yaw` 误差
4. 应用各轴死区、比例增益和输出限幅
5. 将水平速度从世界系 `FLU` 转换到机体系 `FLU`
6. 再映射到 DJI 的机体系 `FRU` joystick 命令并下发

这套控制器当前只使用 `Odometry` 中的位姿信息，不使用速度协方差等其他字段。

如果同步后的 `Odometry` 超过 `odometry_timeout_s` 没有更新：

- 节点会打印节流警告
- 当 `send_zero_on_timeout=true` 时，会发送一次零速度命令作为保护

### 2. 直接速度控制模式

当 `parameters.position_control.enabled=false` 时，驱动切换到兼容模式，订阅机体系 `FLU` 速度命令：

- 话题类型：`geometry_msgs/TwistStamped`
- 默认话题名：`/PSDK/DjiFlightController/CommandInBodyFLU`

该模式下实际使用的字段是：

- `twist.linear.x`
- `twist.linear.y`
- `twist.linear.z`
- `twist.angular.z`

然后按以下规则映射到 DJI joystick 命令：

- `x_dji = x_flu`
- `y_dji = -y_flu`
- `z_dji = z_flu`
- `yaw_dji = -yaw_rate_rad_per_sec * 180 / pi`

Joystick mode 固定为：

- 水平控制：速度模式
- 垂直控制：速度模式
- 偏航控制：角速度模式
- 水平坐标系：机体系
- 稳定模式：开启

示例：

```bash
rostopic pub -r 20 /PSDK/DjiFlightController/CommandInBodyFLU geometry_msgs/TwistStamped '{
  header: {stamp: now, frame_id: ""},
  twist: {
    linear:  {x: 0.3, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.5}
  }
}'
```

### 3. RC / PSDK 控制权切换

起飞成功后，飞控控制器会启动一个定时器，周期性检查：

- RC 是否连接
- RC 摇杆是否处于中立位
- 当前控制权是在 RC 还是 PSDK

逻辑如下：

1. 如果当前是 PSDK 在控，且检测到 RC 摇杆偏离中立位，驱动会释放 joystick authority 给 RC
2. 如果当前是 RC 在控，且摇杆持续回中达到 `rc_control_return_delay_s`，驱动会重新申请 joystick authority

这是一套很重要的安全机制，意味着人工遥控可以优先接管，而程序恢复接管需要等待 RC 保持中立一段时间。

## ROS 接口

### 输入话题

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/indooruav/desired_odometry` | `nav_msgs/Odometry` | 位置控制模式下的期望位姿输入 |
| `/indooruav/current_odometry` | `nav_msgs/Odometry` | 位置控制模式下的当前位姿输入 |
| `/PSDK/DjiFlightController/CommandInBodyFLU` | `geometry_msgs/TwistStamped` | 直接速度控制模式输入，仅在 `position_control.enabled=false` 时使用 |

### 服务

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/PSDK/DjiFlightController/TakeOffService` | `indooruav_dji_driver/PSDK_TakeOff` | 起飞服务 |
| `/PSDK/DjiFlightController/LandingService` | `indooruav_dji_driver/PSDK_Landing` | 降落服务 |
| `/PSDK/DjiCameraController/CameraShootPhoto` | `indooruav_dji_driver/PSDK_CameraShootPhoto` | 拍照服务 |
| `/PSDK/DjiGimbalController/GimbalPitchAngleInDegService` | `indooruav_dji_driver/PSDK_GimbalPitchAngleInDeg` | 云台俯仰角控制服务 |

### 当前没有对外发布的话题

尽管驱动内部订阅了大量 DJI 数据，但当前版本没有 ROS publisher。

## 服务说明与示例

### 1. 起飞服务

服务类型：

```text
indooruav_dji_driver/PSDK_TakeOff
```

请求与响应：

```text
bool takeoff
---
bool result
```

示例：

```bash
rosservice call /PSDK/DjiFlightController/TakeOffService "takeoff: true"
```

### 2. 降落服务

服务类型：

```text
indooruav_dji_driver/PSDK_Landing
```

请求与响应：

```text
bool landing
---
bool result
```

示例：

```bash
rosservice call /PSDK/DjiFlightController/LandingService "landing: true"
```

重要说明：当前实现调用的是 `DjiFlightController_StartForceLanding()`，所以这个服务的实际行为更接近“强制降落”，而不是普通自动降落。使用前请充分评估安全风险。

### 3. 拍照服务

服务类型：

```text
indooruav_dji_driver/PSDK_CameraShootPhoto
```

请求与响应：

```text
bool shoot_photo
float32 zoom_factor
---
bool result
```

行为说明：

- 只有 `shoot_photo=true` 才会执行拍照
- 当 `zoom_factor >= 1.0` 时，会先尝试设置光学变焦
- 目标倍率会被限制在 `[1.0, 6.9]`
- 如果当前倍率已经足够接近目标倍率，则不会重复设置

示例：

```bash
rosservice call /PSDK/DjiCameraController/CameraShootPhoto \
"shoot_photo: true
zoom_factor: 2.0"
```

### 4. 云台俯仰服务

服务类型：

```text
indooruav_dji_driver/PSDK_GimbalPitchAngleInDeg
```

请求与响应：

```text
float32 gimbal_pitch_angle_in_deg
---
bool result
```

行为说明：

- 输入角度会被限制在 `[-90, 0]`
- `0` 度表示水平
- 负值表示向下俯视
- 服务内部使用绝对角模式，并把 yaw 设置为当前机体 yaw

示例：

```bash
rosservice call /PSDK/DjiGimbalController/GimbalPitchAngleInDegService \
"gimbal_pitch_angle_in_deg: -45.0"
```

## 参数说明

参数文件为 [`config/mavic_3t_config.yaml`](/home/lxy/indooruav_ws/src/indooruav_dji_driver/config/mavic_3t_config.yaml)，参数根路径为：

```text
/indooruav_dji_driver/dji_mavic_3t
```

飞控控制器当前通过 `GetRequiredParam()` 读取这些参数，因此缺少任意一个必要参数都会在启动时抛出异常并退出。

### 话题与服务名

| 参数路径 | 默认值 | 说明 |
| --- | --- | --- |
| `/topics/command_in_body_flu` | `/PSDK/DjiFlightController/CommandInBodyFLU` | 直接速度控制模式话题 |
| `/topics/desired_odometry` | `/indooruav/desired_odometry` | 期望位姿输入 |
| `/topics/current_odometry` | `/indooruav/current_odometry` | 当前位姿输入 |
| `/services/takeoff` | `/PSDK/DjiFlightController/TakeOffService` | 起飞服务名 |
| `/services/landing` | `/PSDK/DjiFlightController/LandingService` | 降落服务名 |
| `/services/camera_shoot_photo` | `/PSDK/DjiCameraController/CameraShootPhoto` | 拍照服务名 |
| `/services/gimbal_pitch_angle_in_deg` | `/PSDK/DjiGimbalController/GimbalPitchAngleInDegService` | 云台俯仰服务名 |

### 通用与起飞相关

| 参数路径 | 默认值 | 说明 |
| --- | --- | --- |
| `/parameters/rc_value_detection_frequency_hz` | `10.0` | RC 中立检测频率 |
| `/parameters/rc_zero_deadband` | `0.02` | RC 摇杆中立死区 |
| `/parameters/rc_control_return_delay_s` | `5.0` | RC 回中后重新申请控制权的等待时间 |
| `/parameters/command_subscriber_queue_size` | `10` | 直接速度控制模式订阅队列长度 |
| `/parameters/takeoff/motor_started_timeout_cycles` | `20` | 起飞阶段轮询超时计数 |
| `/parameters/takeoff/in_air_timeout_cycles` | `110` | 起飞阶段轮询超时计数 |
| `/parameters/rid_info/latitude_rad` | `0.0` | RID 纬度，单位 rad |
| `/parameters/rid_info/longitude_rad` | `0.0` | RID 经度，单位 rad |
| `/parameters/rid_info/altitude_m` | `0` | RID 高度，单位 m |

### 位置控制相关

| 参数路径 | 默认值 | 说明 |
| --- | --- | --- |
| `/parameters/position_control/enabled` | `true` | 是否启用位置控制模式 |
| `/parameters/position_control/subscriber_queue_size` | `20` | `Odometry` 订阅队列长度 |
| `/parameters/position_control/sync_queue_size` | `50` | 同步器队列长度 |
| `/parameters/position_control/sync_max_interval_s` | `0.05` | 两路 `Odometry` 允许的最大时间差 |
| `/parameters/position_control/control_frequency_hz` | `30.0` | 控制器输出频率 |
| `/parameters/position_control/odometry_timeout_s` | `0.2` | 同步 `Odometry` 超时阈值 |
| `/parameters/position_control/send_zero_on_timeout` | `true` | 超时后是否发送一次零速度命令 |
| `/parameters/position_control/x/gain` | `0.8` | `x` 轴比例增益 |
| `/parameters/position_control/x/deadband_m` | `0.10` | `x` 轴死区 |
| `/parameters/position_control/x/output_limit_mps` | `1.0` | `x` 轴速度限幅 |
| `/parameters/position_control/y/gain` | `0.8` | `y` 轴比例增益 |
| `/parameters/position_control/y/deadband_m` | `0.10` | `y` 轴死区 |
| `/parameters/position_control/y/output_limit_mps` | `1.0` | `y` 轴速度限幅 |
| `/parameters/position_control/z/gain` | `0.8` | `z` 轴比例增益 |
| `/parameters/position_control/z/deadband_m` | `0.10` | `z` 轴死区 |
| `/parameters/position_control/z/output_limit_mps` | `0.8` | `z` 轴速度限幅 |
| `/parameters/position_control/yaw/gain` | `1.2` | `yaw` 误差比例增益 |
| `/parameters/position_control/yaw/deadband_rad` | `0.10` | `yaw` 误差死区 |
| `/parameters/position_control/yaw/output_limit_radps` | `0.5236` | `yaw` 角速度限幅 |

## 内部 DJI 订阅项

[`src/indooruav_dji_driver/mavic_3t_data_subscription_manager.cpp`](/home/lxy/indooruav_ws/src/indooruav_dji_driver/src/indooruav_dji_driver/mavic_3t_data_subscription_manager.cpp) 当前注册了以下内部 topic：

| DJI topic | 频率 | 用途 |
| --- | --- | --- |
| `QUATERNION` | `10 Hz` | 姿态参考、云台 yaw 参考 |
| `VELOCITY` | `10 Hz` | 内部保留 |
| `ANGULAR_RATE_FUSIONED` | `10 Hz` | 内部保留 |
| `RC` | `10 Hz` | 内部保留 |
| `GIMBAL_ANGLES` | `50 Hz` | 云台角度参考 |
| `STATUS_FLIGHT` | `10 Hz` | 起飞状态判断 |
| `STATUS_DISPLAYMODE` | `10 Hz` | 起飞状态判断 |
| `CONTROL_DEVICE` | `10 Hz` | 控制权判断 |
| `RC_WITH_FLAG_DATA` | `10 Hz` | RC 连接与中立判断 |
| `FLIGHT_ANOMALY` | `10 Hz` | 异常信息保留 |
| `POSITION_VO` | `10 Hz` | 视觉里程计位置保留 |
| `BATTERY_SINGLE_INFO_INDEX1` | `1 Hz` | 电池信息保留 |

电机启动错误码 topic 在代码里预留了回调，但当前没有真正订阅。

## 已知限制与注意事项

- 当前没有任何 ROS publisher，内部 DJI 订阅数据不会自动桥接到 ROS。
- `LandingService` 实际执行的是强制降落，不是普通自动降落。
- 位置控制器只使用两路 `Odometry` 的位姿，不使用其线速度信息。
- 位置控制默认开启；开启后不会订阅直接速度控制话题。
- 云台和相机挂载口当前写死为 `DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1`。
- 相机录像相关函数已经实现，但没有暴露成 ROS 接口。
- 项目包含 DJI 相关头文件和静态库，`package.xml` 当前许可证标记为 `Proprietary`，请按你的分发场景自行评估。

## 常见排障

### 节点启动即退出

优先检查以下几项：

- `dji_sdk_app_info_local.h` 是否存在
- 本地头文件中的占位符是否已替换为真实值
- 串口设备路径是否正确
- 当前用户是否有串口访问权限
- Payload 挂载位置是否满足启动检查要求

### 一直提示等待同步的 Odometry

如果看到类似“waiting for synchronized desired/current odometry”的警告，通常说明：

- 两路 `Odometry` 没有同时发布
- 时间戳不对齐，超过 `sync_max_interval_s`
- 参考系定义不一致，导致上层输入本身就不合理

### 位置控制没有输出

请检查：

- `position_control.enabled` 是否为 `true`
- `desired_odometry` 与 `current_odometry` 是否持续更新
- `odometry_timeout_s` 是否过小
- 飞机当前是否已经由 RC 接管控制权

### 直接速度控制不生效

请确认：

- `position_control.enabled=false`
- 正在向 `command_in_body_flu` 对应的话题发送 `geometry_msgs/TwistStamped`
- 起飞后控制权已由 PSDK 获取，且没有被 RC 抢回

## 后续建议

如果你准备继续扩展这个包，比较值得优先做的方向包括：

- 把内部 DJI 订阅数据桥接成标准 ROS topic
- 为录像、停止录像、变焦单独增加 ROS 接口
- 将普通降落与强制降落拆分为不同服务
- 增加状态诊断、自检和错误码文档
- 为位置控制器补充状态输出与调参说明
