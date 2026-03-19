# indooruav_dji_driver

`indooruav_dji_driver` 是一个基于 DJI Payload SDK 的 ROS1 驱动包，用来把 DJI Mavic 3T 的部分飞控、云台和相机能力封装成 ROS 接口，便于上层自主飞行、任务调度或实验程序调用。

当前实现以 `mavic_3t_driver_node` 为核心，完成了以下几类能力：

- 订阅 `geometry_msgs/TwistStamped` 的机体系 FLU 速度控制命令
- 提供起飞、强制降落、云台俯仰控制、相机拍照服务
- 初始化 DJI Payload SDK 运行环境
- 订阅 DJI 飞控内部状态数据，用于控制逻辑和状态判断

这个包适合做什么：

- 室内无人机实验中的底层控制适配层
- 将 DJI PSDK 接入 ROS1 系统
- 为上层路径规划、状态机、视觉任务提供统一入口

这个包目前不做什么：

- 不发布 IMU、姿态、位置、电池等 ROS 话题
- 不提供完整的状态估计桥接
- 不提供录像、变焦单独控制、回家等 ROS 服务
- 不包含仿真环境

## 1. 项目结构

```text
indooruav_dji_driver/
├── config/
│   └── mavic_3t_config.yaml          # ROS 参数
├── include/
│   ├── dependences/                  # DJI PSDK 平台适配层头文件
│   └── indooruav_dji_driver/         # 控制器头文件
├── launch/
│   └── mavic_3t_driver.launch        # 启动文件
├── node/
│   └── mavic_3t_driver.cpp           # 主节点入口
├── psdk_lib/                         # DJI Payload SDK 头文件与静态库
├── src/
│   ├── dependences/                  # UART / OSAL / Application 适配实现
│   └── indooruav_dji_driver/         # 飞控、云台、相机、数据订阅实现
└── srv/                              # 自定义 ROS Service 定义
```

## 2. 功能概览

### 飞控控制

- 节点订阅一个速度控制话题，将 ROS 指令直接映射到 DJI joystick command
- 起飞服务会等待电机启动、离地、起飞结束三个阶段完成后再返回成功
- 降落服务当前实际调用的是 **强制降落** 接口，而不是普通自动降落
- 起飞成功后，节点会启动 RC 摇杆检测逻辑，自动在 RC 和 PSDK 之间切换控制权

### 云台控制

- 提供云台俯仰角服务
- 云台模式初始化为 `DJI_GIMBAL_MODE_YAW_FOLLOW`
- 请求角度会被限制到 `[-90, 0]` 度

### 相机控制

- 提供拍照服务
- 可在拍照前附带设置一次光学变焦
- 当前代码中光学变焦限制为 `[1.0, 6.9]`

### DJI 内部数据订阅

节点启动后会订阅多类 DJI 飞控内部 topic，包括：

- Quaternion
- Velocity
- AngularRateFusioned
- RC
- GimbalAngles
- Flight Status
- Display Mode
- Control Device
- RCWithFlagData
- Flight Anomaly
- PositionVO
- BatterySingleInfoIndex1

注意：这些订阅当前只在节点内部使用，**不会发布为 ROS 话题**。

## 3. 依赖与运行环境

### 软件依赖

- Ubuntu + ROS1 Catkin 工作空间
- `roscpp`
- `geometry_msgs`
- `std_msgs`
- `message_generation` / `message_runtime`
- DJI Payload SDK 静态库和头文件

### 硬件与连接方式

当前代码使用 DJI Payload SDK 的 Linux 平台适配实现，并且默认编译为：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_ONLY_UART
```

也就是说，当前工程默认走 **UART 连接**。

串口设备名定义在 `include/dependences/hal_uart.h`：

- `LINUX_UART_DEV1 = "/dev/ttyTHS0"`
- `LINUX_UART_DEV2 = "/dev/ttyACM0"`

如果你的机载计算平台、转接板或实际接线不同，需要按实际环境修改这里的设备名。

### 支持的编译架构

`CMakeLists.txt` 会根据 `CMAKE_SYSTEM_PROCESSOR` 自动选择 `psdk_lib/lib` 下的静态库，目前支持：

- `x86_64`
- `aarch64`
- `armv7 / arm`

## 4. 编译前需要配置的内容

### 4.1 DJI 开发者信息

仓库中提交的：

- `include/dependences/dji_sdk_app_info.h`

现在只保留占位符，不再存放真实密钥。

真实凭据应保存在本地文件：

- `include/dependences/dji_sdk_app_info_local.h`

这个本地文件已经被 `.gitignore` 忽略，不会被提交到仓库。仓库里同时提供了模板：

- `include/dependences/dji_sdk_app_info_local.h.example`

本地文件中需要填写的字段包括：

- `USER_APP_NAME`
- `USER_APP_ID`
- `USER_APP_KEY`
- `USER_APP_LICENSE`
- `USER_DEVELOPER_ACCOUNT`
- `USER_BAUD_RATE`

如果本地文件不存在，程序会继续编译，但运行时会因为仍是占位符而报错退出。

### 4.2 运行时本地配置

- `include/dependences/dji_sdk_app_info_local.h`

仓库中只保留模板文件：

- `include/dependences/dji_sdk_app_info_local.h.example`

`Application` 初始化阶段会从本地头文件读取下面这几个宏：

- `USER_PSDK_ALIAS`
- `USER_PSDK_SERIAL_NUMBER`
- `USER_PSDK_LOG_PATH`

### 4.3 安装位置检查

启动时程序会检查 DJI 设备安装位置或适配器类型，不满足条件会直接报错退出。当前代码要求满足以下之一：

- 安装在 extension port
- 使用 E-Port V2 ribbon cable
- 使用 SkyPort V3

### 4.4 Payload 挂载口

云台和相机控制器当前都固定使用：

```c
DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1
```

如果你的设备实际不在 `payload port no1`，需要同步修改对应常量。

## 5. 编译方法

在 catkin 工作空间根目录执行：

```bash
catkin_make --pkg indooruav_dji_driver
source devel/setup.bash
```

如果你希望直接编译整个工作空间，也可以使用：

```bash
catkin_make
source devel/setup.bash
```

## 6. 启动方法

默认启动文件：

```bash
roslaunch indooruav_dji_driver mavic_3t_driver.launch
```

这个 launch 文件会：

1. 加载 `config/mavic_3t_config.yaml`
2. 启动节点 `mavic_3t_driver_node`

节点名为：

```text
/mavic_3t_driver
```

日志会输出到终端，同时 DJI 本地日志会写入：

```text
Logs/
```

## 7. ROS 接口说明

### 7.1 订阅话题

#### `/PSDK/DjiFlightController/CommandInBodyFLU`

- 类型：`geometry_msgs/TwistStamped`
- 作用：订阅 ROS 标准机体系 `FLU` 控制命令
- 默认参数路径：`/indooruav_dji_driver/dji_mavic_3t/topics/command_in_body_flu`

字段映射关系如下：

- `twist.linear.x` -> 机体系 FLU 下 `x` 方向速度，前方为正，单位 `m/s`
- `twist.linear.y` -> 机体系 FLU 下 `y` 方向速度，左方为正，单位 `m/s`
- `twist.linear.z` -> `z` 方向速度，向上为正，单位 `m/s`
- `twist.angular.z` -> 机体系 FLU 下偏航角速度，按 ROS 右手系约定，单位 `rad/s`

回调函数会在发送给 DJI 前做如下转换：

- `x_dji = x_flu`
- `y_dji = -y_flu`
- `z_dji = z_flu`
- `yaw_dji = -yaw_flu * 180 / pi`

也可以按坐标系理解为：

- ROS `FLU` 中 `+X` 为前、`+Y` 为左、`+Z` 为上
- DJI `FRU` 中 `+X` 为前、`+Y` 为右、`+Z` 为上
- 因此线速度只需要对 `y` 轴反号
- 当前代码里 `yaw` 也做了反号后再发送给 DJI

回调函数 `CallbackCommandInBodyFLU()` 当前实际会做这些事情：

1. 检查飞控是否已经初始化；如果 `initialized_ == false`，直接返回。
2. 从 `geometry_msgs/TwistStamped` 中读取：
   `linear.x`、`linear.y`、`linear.z`、`angular.z`
3. 按上面的 FLU -> FRU 规则装载到 `T_DjiFlightControllerJoystickCommand`，其中 `yaw` 会先从 `rad/s` 转成 `deg/s`
4. 调用 `DjiFlightController_ExecuteJoystickAction()` 将命令发送给 DJI 飞控
5. 如果下发失败，打印 warning 日志

当前未使用的字段有：

- `twist.angular.x`
- `twist.angular.y`
- `header`

飞控 joystick mode 仍然会初始化为：

- 水平控制：速度模式
- 垂直控制：速度模式
- 偏航控制：角速度模式
- 水平坐标系：DJI 机体系 `FRU`
- 稳定模式：开启

因此，现在已经完成了 FLU 指令到 DJI joystick command 的基础符号映射与发送。

按当前 PSDK joystick mode 定义，常见控制范围可按下面理解：

- `x / y`：约 `[-30, 30] m/s`
- `z`：约 `[-5, 5] m/s`
- `yaw` 下发给 DJI：约 `[-150, 150] deg/s`
- 对应订阅输入 `twist.angular.z`：约 `[-2.618, 2.618] rad/s`

示例：

```bash
rostopic pub -r 20 /PSDK/DjiFlightController/CommandInBodyFLU geometry_msgs/TwistStamped \
'{
  header: {stamp: now, frame_id: ""},
  twist: {
    linear:  {x: 0.3, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.5}
  }
}'
```

### 7.2 服务

#### `/PSDK/DjiFlightController/TakeOffService`

- 类型：`indooruav_dji_driver/PSDK_TakeOff`
- 请求：

```text
bool takeoff
```

- 响应：

```text
bool result
```

示例：

```bash
rosservice call /PSDK/DjiFlightController/TakeOffService "takeoff: true"
```

#### `/PSDK/DjiFlightController/LandingService`

- 类型：`indooruav_dji_driver/PSDK_Landing`
- 请求：

```text
bool landing
```

- 响应：

```text
bool result
```

示例：

```bash
rosservice call /PSDK/DjiFlightController/LandingService "landing: true"
```

重要说明：这个服务在当前实现中实际调用的是 `DjiFlightController_StartForceLanding()`，因此它的行为更接近 **强制降落**，请谨慎使用。

#### `/PSDK/DjiCameraController/CameraShootPhoto`

- 类型：`indooruav_dji_driver/PSDK_CameraShootPhoto`
- 请求：

```text
bool shoot_photo
float32 zoom_factor
```

- 响应：

```text
bool result
```

行为说明：

- 只有 `shoot_photo: true` 才会执行拍照
- 当 `zoom_factor >= 1.0` 时，会先尝试设置光学变焦再拍照
- 变焦倍率会被限制到 `[1.0, 6.9]`
- 当 `zoom_factor < 1.0` 时，保持当前倍率直接拍照

示例：

```bash
rosservice call /PSDK/DjiCameraController/CameraShootPhoto \
"shoot_photo: true
zoom_factor: 2.0"
```

#### `/PSDK/DjiGimbalController/GimbalPitchAngleInDegService`

- 类型：`indooruav_dji_driver/PSDK_GimbalPitchAngleInDeg`
- 请求：

```text
float32 gimbal_pitch_angle_in_deg
```

- 响应：

```text
bool result
```

行为说明：

- 输入角度会被限制到 `[-90, 0]`
- 0 度表示水平
- 负值表示向下俯视

示例：

```bash
rosservice call /PSDK/DjiGimbalController/GimbalPitchAngleInDegService \
"gimbal_pitch_angle_in_deg: -45.0"
```

## 8. 参数说明

默认参数文件：`config/mavic_3t_config.yaml`

参数根路径：

```text
/indooruav_dji_driver/dji_mavic_3t
```

当前代码实际会读取的参数如下：

| 参数路径 | 默认值 | 说明 |
| --- | --- | --- |
| `/topics/command_in_body_flu` | `/PSDK/DjiFlightController/CommandInBodyFLU` | FLU 速度控制命令订阅话题 |
| `/services/takeoff` | `/PSDK/DjiFlightController/TakeOffService` | 起飞服务名 |
| `/services/landing` | `/PSDK/DjiFlightController/LandingService` | 降落服务名 |
| `/services/camera_shoot_photo` | `/PSDK/DjiCameraController/CameraShootPhoto` | 拍照服务名 |
| `/services/gimbal_pitch_angle_in_deg` | `/PSDK/DjiGimbalController/GimbalPitchAngleInDegService` | 云台俯仰控制服务名 |
| `/parameters/rc_value_detection_frequency_hz` | `10.0` | RC 中立检测频率 |
| `/parameters/rc_zero_deadband` | `0.02` | RC 摇杆中立死区 |
| `/parameters/rid_info/latitude_rad` | `0.0` | RID 纬度，单位 rad |
| `/parameters/rid_info/longitude_rad` | `0.0` | RID 经度，单位 rad |
| `/parameters/rid_info/altitude_m` | `0` | RID 高度，单位 m |

说明：


## 9. 控制权切换逻辑

这是这个驱动里一个很重要的安全机制。

起飞成功后，节点会启动一个定时器，周期性检查：

- RC 是否连接
- RC 摇杆是否处于中立位
- 当前飞行控制权属于 RC 还是 PSDK

逻辑如下：

1. 如果当前控制权属于 PSDK，且检测到 RC 有非中立输入，则驱动立即释放 joystick authority 给 RC。
2. 如果当前控制权属于 RC，且摇杆已经保持中立连续 5 秒，则驱动会重新申请 joystick authority。

这意味着：

- 人工遥控可以随时“抢回”控制权
- 上层程序恢复接管前，需要等待 RC 摇杆回中并保持 5 秒

## 10. 已知限制与注意事项

- 当前没有任何 ROS publisher，DJI 内部订阅数据只在节点内部消费。
- `LandingService` 实际为强制降落，不是普通降落。
- 相机录像相关函数已写在代码里，但没有暴露为 ROS 服务。
- `StartLanding()` 和 `StartConfirmLanding()` 存在实现，但当前未对外使用。
- 配置文件中的 `application.*` 参数尚未接线到实际初始化流程。
- 串口权限处理依赖底层 `hal_uart.c`，请确保运行用户对目标串口设备有访问权限。
- 本项目包含 DJI 相关头文件和静态库，`package.xml` 许可证标记为 `Proprietary`，请根据你的分发场景自行评估。

## 11. 后续可改进方向

如果你准备继续完善这个包，比较值得优先做的方向有：

- 把 DJI 内部订阅数据桥接成标准 ROS topic
- 将录像、停止录像、变焦单独封装成服务或 action
- 为强制降落和普通降落拆分不同服务名，减少误用风险
- 增加状态诊断、错误码说明和运行自检

## 12. 相关文件

- 主节点入口：`node/mavic_3t_driver.cpp`
- 飞控控制：`src/indooruav_dji_driver/mavic_3t_flight_controller.cpp`
- 云台控制：`src/indooruav_dji_driver/mavic_3t_gimbal_controller.cpp`
- 相机控制：`src/indooruav_dji_driver/mavic_3t_camera_controller.cpp`
- DJI 数据订阅：`src/indooruav_dji_driver/mavic_3t_data_subscription_manager.cpp`
- 平台初始化：`src/dependences/application.cpp`
- 参数配置：`config/mavic_3t_config.yaml`
- 启动文件：`launch/mavic_3t_driver.launch`
