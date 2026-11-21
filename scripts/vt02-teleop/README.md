# 轮腿 Windows 键鼠遥控

1. 按下文启用 VT‑02 并编译，烧录 `build/basic_framework.hex`。接通主控板和串口链路，先断开电机动力做通信验收。
2. 双击 `build/vt02-teleop/WheellegTeleop.exe`，选 USB 转串口的 COM 口，点「连接」。窗口的已发帧数会递增。插拔设备后可点「刷新」。
3. 让工具保持前台，松开控制键并停下鼠标，再按 `R` 允许键鼠控制。台架通信验收通过后，直接用键盘或移动鼠标控制底盘。

## 接线

| 链路 | 连接 |
| --- | --- |
| 电脑端使用 USB 转 TTL | USB‑TTL 的 `TX → 无线发送端 UART RX`，两端 `GND` 相连。发送端需支持从该 RX 转发 |
| 电脑端使用无线设备的 USB CDC | USB 连接电脑，配对无线设备后选择其透传 COM 口。以设备实际透传测试为准 |
| 车端 | 无线接收端 `UART TX → 主控 USART1_RX (PB7)`，两端 `GND` 相连。主控 `USART1_TX (PA9)` 可留空 |

串口使用 3.3 V TTL 信号、115200 波特、8N1、无流控。由各模块已确认的供电端供电。联调时也可用一根 USB‑TTL 直接接主控 `PB7` 和 `GND`，先验证主控收帧，再插入无线链路。参考 [PowerDebugger 无线配对与串口代理说明](https://docs.powerwriter.com/docs/next/powerdebugger_for_arm/manual/debugger-setting/) 和 [本工程 USART1 引脚](../../Src/usart.c#L138)。

## 操作

| 输入 | 作用 |
| --- | --- |
| `R` | 上电、急停或断链恢复后，先松开控制键并停下鼠标，单独按 `R` 允许键鼠控制 |
| `W/S` | 前进/后退，满指令 0.30 m/s |
| `A/D` 或左右移动鼠标 | 左右转向，上限 0.80 rad/s。有 `A/D` 输入时使用按键 |
| `Z/X/C` | 站立或行驶时设腿长 0.15/0.25/0.35 m |
| 移动输入归零或工具失焦 | 清 `vx/wz`，进入正常站立锁定。返回工具后先松开控制键 |
| `B` | 零力急停，车辆失去动力支撑。已采集到的急停指令短时重复发送，主控收到后锁存，腿长目标回 0.25 m |

仅 VT‑02 时，上电进入正常站立。首次键鼠控制前先按 `R`。使用 VT‑02 行驶时，如果连续 150 ms 收不到有效输入帧，主控清移动目标，保持站立控制和当前腿长目标。恢复通信后，需要中性输入和新的 `R` 按下沿。急停状态跨断链保持。已启动的起立过程由原有底盘状态机管理。鼠标横移按 0.04 rad/s 每计数换算，单帧鼠标转向最多保留 40 ms。以上限速和鼠标灵敏度均为首轮台架初值。

## 选择遥控输入并构建

在 [`application/robot_def.h`](../../application/robot_def.h) 选择遥控输入。无线串口透传和输出相同 `0x0304` 帧的 VT‑02 图传共用 USART1 接收模块。

| `WHEELLEG_REMOTE_MODE` | `WHEELLEG_VT02_FALLBACK` | 控制来源 |
| --- | --- | --- |
| `WHEELLEG_REMOTE_SBUS` 或 `WHEELLEG_REMOTE_DBUS` | `1` | 两路同时接收，遥控器在线优先，离线自动选 VT‑02 |
| `WHEELLEG_REMOTE_SBUS` 或 `WHEELLEG_REMOTE_DBUS` | `0` | 只用遥控器 |
| `WHEELLEG_REMOTE_VT02` | 任意 | 只用 VT‑02 |

双路模式下，遥控器的站立挡与急停也具有优先权。遥控器离线时，松开键鼠再按 `R`，重新授权电脑侧行驶。遥控器恢复在线后立即接管。两路均离线时清移动目标，保持站立锁定或已触发的零力。SW0 急停由 SW0 回上解除，掉线时继续保持。电脑侧获选时可用 `B` 急停，再用 `R` 解除，或在遥控器恢复后用 SW0 下拨再回上解除。

切换宏后从固件仓库根目录执行常规构建。电脑端使用 Windows 自带的 .NET Framework 4.x 编译器生成 EXE，运行时使用该框架。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/vt02-teleop/Build.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/project.ps1 -Action build
```

| 产物 | 位置 |
| --- | --- |
| Windows 工具 | `build/vt02-teleop/WheellegTeleop.exe` |
| 轮腿固件 | `build/basic_framework.hex`，同目录有 `.elf` 和 `.bin`。控制源由宏选择 |

## 台架验收

1. 动力断开后，用对应固件 `.elf` 在 EP 观察 `vt_link_frame_cnt`、`vt_link_crc_err_cnt` 和 `vt_link_online`。只接主控和串口，确认有效帧数递增且在线。Windows 的帧数表示本地串口写入次数。
2. 同时观察 `chassis_cmd_send.chassis_mode`、`vx`、`wz` 和 `leg_length_target`。核对上电站立、`R`、前后与转向符号、三挡腿长，以及操作停止和切窗口后的站立锁定。按 `B` 后，确认松键仍保持零力，再用 `R` 恢复。
3. 双路模式下同时保持键鼠和遥控器在线，核对遥控器站立挡仍锁定。关闭遥控器后，确认按住的键鼠指令未立即生效，松开并按 `R` 后再试。恢复遥控器，确认它立即接管。
4. VT‑02 行驶时断开串口或无线链路，确认主控超时清零。重连后检查仍需松键、停鼠标并重新按 `R`。双路模式还需核对 SW0 或 `B` 急停跨断链保持。记录掉线和恢复过程，再做带动力低速试验。

协议沿用参考工程的 `0x0304`，帧长 21 字节。序号为 `0x25`、只按下 `W` 时的示例帧如下。

```text
A5 0C 00 25 64 04 03 00 00 00 00 00 00 00 00 01 00 00 00 5F BE
```
