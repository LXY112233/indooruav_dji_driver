# indooruav_dji_driver

`indooruav_dji_driver` 是一个面向 ROS1 的 DJI Payload SDK 驱动包，当前主要服务于 `DJI Mavic 3T` 的机载接入场景。

当前版本的飞控接口已经收敛为单一的“位置闭环控制”设计：

- 节点只订阅两路 `nav_msgs/Odometry`
  - `desired_odometry`：期望位姿
  - `current_odometry`：当前位姿
- 节点对外提供飞控起飞与降落服务
  - `TakeOffService`
  - `LandingService`
- 驱动内部使用四通道 PID 结构分别计算 `x / y / z / yaw` 控制量
- 线速度先在地面/world `FLU` 坐标系中生成，再转换到机体 `FLU`
- 最终再映射到 DJI 飞控使用的机体 `FRU` joystick 命令
- 不再订阅机体系 `FLU` 速度指令话题，不再支持直接 `TwistStamped` 外部速度控制模式

除飞控外，包内仍保留云台俯仰与相机拍照服务，这部分能力与本次飞控接口收敛互不冲突。

## 核心设计

飞控控制链路如下：

1. 使用 `message_filters::Synchronizer` 对 `desired_odometry` 与 `current_odometry` 做近似时间同步
2. 从两路 `Odometry` 中提取位置与姿态 `yaw`
3. 分别计算 `x / y / z / yaw` 误差
4. 每个通道执行 `PID + deadband + output limit`
5. 将水平速度从地面/world `FLU` 转为机体 `FLU`
6. 将机体 `FLU` 转为 DJI 机体 `FRU` joystick 命令并下发

当前控制器只使用 `Odometry` 的位姿字段：

- `pose.pose.position`
- `pose.pose.orientation`

不会使用 `twist`、协方差等其他字段。

### 坐标约定

- `desired_odometry` 与 `current_odometry` 必须表达在同一个参考系中
- 当前实现假设这个参考系是统一的地面/world `FLU`
- 控制器先得到 world `FLU` 下的速度命令
- 再根据当前 yaw 把水平速度变换到机体 `FLU`
- 最后按 DJI 约定映射为机体 `FRU`

DJI 映射规则如下：

- `x_dji = x_body_flu`
- `y_dji = -y_body_flu`
- `z_dji = z_body_flu`
- `yaw_dji_deg_s = -yaw_rate_rad_s * 180 / pi`

### Odometry 超时保护

如果同步后的 `Odometry` 长时间未更新并超过 `odometry_timeout_s`：

- 节点会打印节流警告
- 控制器内部 PID 状态会被复位
- 当 `send_zero_on_timeout=true` 时，会额外发送一次零速度命令

### RC / PSDK 控制权切换

起飞成功后，驱动会周期性检查：

- RC 是否连接
- RC 摇杆是否回中
- 当前控制权是否在 PSDK

逻辑如下：

1. 当 PSDK 在控且 RC 摇杆偏离中立位时，驱动释放控制权给 RC
2. 当 RC 在控且摇杆持续回中达到 `rc_control_return_delay_s` 时，驱动重新申请 joystick authority

这套安全逻辑没有随本次接口调整而变化。

## ROS 接口

### 输入话题

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/indooruav/desired_odometry` | `nav_msgs/Odometry` | 期望位姿输入 |
| `/indooruav/current_odometry` | `nav_msgs/Odometry` | 当前位姿输入 |

### 飞控服务

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/PSDK/DjiFlightController/TakeOffService` | `indooruav_dji_driver/PSDK_TakeOff` | 起飞服务 |
| `/PSDK/DjiFlightController/LandingService` | `indooruav_dji_driver/PSDK_Landing` | 降落服务 |

`LandingService` 当前内部调用的是 `DjiFlightController_StartForceLanding()`，因此它的实际行为更接近“强制降落”。在真实飞机上使用前请充分评估安全性。

### 其他服务

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/PSDK/DjiCameraController/CameraShootPhoto` | `indooruav_dji_driver/PSDK_CameraShootPhoto` | 拍照服务 |
| `/PSDK/DjiGimbalController/GimbalPitchAngleInDegService` | `indooruav_dji_driver/PSDK_GimbalPitchAngleInDeg` | 云台俯仰角服务 |

### 当前没有对外发布的话题

虽然包内会订阅多类 DJI 内部数据，但当前版本仍然没有将这些数据桥接成 ROS publisher。

## 参数说明

参数文件位于 `config/mavic_3t_config.yaml`，根路径为：

```text
/indooruav_dji_driver/dji_mavic_3t
```

飞控控制器通过 `GetRequiredParam()` 读取参数，因此缺少任意必要参数都会在启动阶段抛出异常并退出。

### 话题与服务名

| 参数路径 | 默认值 | 说明 |
| --- | --- | --- |
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
| `/parameters/takeoff/motor_started_timeout_cycles` | `20` | 起飞阶段电机启动轮询超时计数 |
| `/parameters/takeoff/in_air_timeout_cycles` | `110` | 起飞阶段离地轮询超时计数 |
| `/parameters/rid_info/latitude_rad` | `0.0` | RID 纬度，单位 rad |
| `/parameters/rid_info/longitude_rad` | `0.0` | RID 经度，单位 rad |
| `/parameters/rid_info/altitude_m` | `0` | RID 高度，单位 m |

### 位置控制相关

| 参数路径 | 默认值 | 说明 |
| --- | --- | --- |
| `/parameters/position_control/subscriber_queue_size` | `20` | 两路 `Odometry` 订阅队列长度 |
| `/parameters/position_control/sync_queue_size` | `50` | 同步器队列长度 |
| `/parameters/position_control/sync_max_interval_s` | `0.05` | 两路 `Odometry` 允许的最大时间差 |
| `/parameters/position_control/control_frequency_hz` | `30.0` | PID 输出频率 |
| `/parameters/position_control/odometry_timeout_s` | `0.2` | 同步 `Odometry` 超时阈值 |
| `/parameters/position_control/send_zero_on_timeout` | `true` | 超时后是否额外发送一次零速度命令 |

### 各轴 PID 参数

| 参数路径 | 默认值 | 说明 |
| --- | --- | --- |
| `/parameters/position_control/x/kp` | `0.8` | `x` 轴比例增益 |
| `/parameters/position_control/x/ki` | `0.0` | `x` 轴积分增益 |
| `/parameters/position_control/x/kd` | `0.0` | `x` 轴微分增益 |
| `/parameters/position_control/x/deadband_m` | `0.10` | `x` 轴位置死区 |
| `/parameters/position_control/x/output_limit_mps` | `1.0` | `x` 轴速度限幅 |
| `/parameters/position_control/y/kp` | `0.8` | `y` 轴比例增益 |
| `/parameters/position_control/y/ki` | `0.0` | `y` 轴积分增益 |
| `/parameters/position_control/y/kd` | `0.0` | `y` 轴微分增益 |
| `/parameters/position_control/y/deadband_m` | `0.10` | `y` 轴位置死区 |
| `/parameters/position_control/y/output_limit_mps` | `1.0` | `y` 轴速度限幅 |
| `/parameters/position_control/z/kp` | `0.8` | `z` 轴比例增益 |
| `/parameters/position_control/z/ki` | `0.0` | `z` 轴积分增益 |
| `/parameters/position_control/z/kd` | `0.0` | `z` 轴微分增益 |
| `/parameters/position_control/z/deadband_m` | `0.10` | `z` 轴位置死区 |
| `/parameters/position_control/z/output_limit_mps` | `0.8` | `z` 轴速度限幅 |
| `/parameters/position_control/yaw/kp` | `1.2` | `yaw` 比例增益 |
| `/parameters/position_control/yaw/ki` | `0.0` | `yaw` 积分增益 |
| `/parameters/position_control/yaw/kd` | `0.0` | `yaw` 微分增益 |
| `/parameters/position_control/yaw/deadband_rad` | `0.10` | `yaw` 误差死区 |
| `/parameters/position_control/yaw/output_limit_radps` | `0.5236` | `yaw` 角速度限幅 |

默认配置把 `ki` 和 `kd` 设为 `0.0`，这样在未完成整机整定前保持更保守的初始行为；如果需要更完整的 PID 效果，请在实机或仿真中逐步整定。

## 编译前准备

### 1. 配置本地 DJI 凭据

先基于模板创建本地头文件：

```bash
cp include/dependences/dji_sdk_app_info_local.h.example    include/dependences/dji_sdk_app_info_local.h
```

然后填写 `include/dependences/dji_sdk_app_info_local.h` 中的真实内容。程序启动时会读取以下宏：

- `USER_APP_NAME`
- `USER_APP_ID`
- `USER_APP_KEY`
- `USER_APP_LICENSE`
- `USER_DEVELOPER_ACCOUNT`
- `USER_BAUD_RATE`
- `USER_PSDK_ALIAS`
- `USER_PSDK_SERIAL_NUMBER`
- `USER_PSDK_LOG_PATH`

### 2. 检查串口与波特率

需要至少确认以下两处配置：

- `include/dependences/hal_uart.h`
- `USER_BAUD_RATE`

### 3. 检查串口权限

运行用户必须具备目标串口设备的访问权限，否则 PSDK 初始化会失败。

### 4. 检查挂载与口位

当前工程默认使用 UART 连接，云台与相机控制固定使用 `DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1`。如果你的挂载方式不同，需要同步修改对应实现。

## 编译与启动

在 catkin 工作空间根目录执行：

```bash
catkin_make --pkg indooruav_dji_driver
source devel/setup.bash
```

启动：

```bash
roslaunch indooruav_dji_driver mavic_3t_driver.launch
```

默认节点名为：

```text
/mavic_3t_driver
```

## 使用示例

### 起飞

```bash
rosservice call /PSDK/DjiFlightController/TakeOffService "takeoff: true"
```

### 降落

```bash
rosservice call /PSDK/DjiFlightController/LandingService "landing: true"
```

### 发布期望位姿

```bash
rostopic pub -r 20 /indooruav/desired_odometry nav_msgs/Odometry '{
  header: {stamp: now, frame_id: "flu_world"},
  pose: {
    pose: {
      position: {x: 0.5, y: 0.0, z: 1.2},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}'
```

`current_odometry` 需要由外部定位/状态估计模块持续提供，并且必须与 `desired_odometry` 保持同一参考系。

仓库里还提供了一个简单的测试节点 `mavic_3t_driver_test_node`，它会：

- 订阅 `current_odometry`
- 周期发布 `desired_odometry`
- 调用起飞/降落服务完成一段简单航点任务

## 已知限制与注意事项

- 当前没有任何 ROS publisher，内部 DJI 数据不会自动桥接到 ROS。
- `LandingService` 实际执行的是强制降落，而不是普通自动降落。
- PID 控制器当前只使用两路 `Odometry` 的位姿，不使用线速度信息。
- `desired_odometry` 与 `current_odometry` 的时间同步质量会直接影响控制输出。
- 如果 `Odometry` 长时间中断，控制器会复位 PID 状态，并按参数决定是否发送一次零速度命令。
- 包内仍包含相机与云台服务，但它们不参与飞控位置闭环。

## 常见排障

### 节点启动即退出

优先检查：

- `dji_sdk_app_info_local.h` 是否存在且已填写真实值
- 串口设备路径是否正确
- 当前用户是否有串口访问权限
- Payload 挂载方式是否满足启动检查要求

### 一直提示等待同步 Odometry

通常说明：

- 两路 `Odometry` 没有同时发布
- 时间戳差异超过 `sync_max_interval_s`
- 两路消息参考系不一致

### 飞控没有持续输出速度命令

请重点检查：

- `desired_odometry` 与 `current_odometry` 是否持续更新
- `odometry_timeout_s` 是否过小
- 起飞后控制权是否仍然在 PSDK
- 是否被 RC 接管了 joystick authority
- PID 参数是否过小或死区设置过大
