# 供电与 USB 过流问题排查记录

记录 MentorPi 的供电拓扑、USB 过流崩溃问题的分析与解决方案。蜂鸣器 5 声报警问题见 CLAUDE.md「Known Issues」。

## 供电拓扑（实测确认，2026-07-05）

STM32（RRCLite）和 Pi 5 之间有 **两根 USB 线**：

| 线 | 作用 | 说明 |
|----|------|------|
| USB 数据线 | 串口通信 | 板载 USB 转串口芯片（1a86 = 沁恒 CH34x），Pi 上枚举为 `/dev/ttyACM0`（by-id: `usb-1a86_USB_Single_Serial_5B21250490-if00`）。**这就是"串口线"**——没有独立的串口线。数据线 VBUS（5V 脚）会给 CH34x 芯片供电，属正常设计 |
| Type-C 共享供电线 | STM32 → Pi 供电 | 电池模式下由 STM32 板的 5V buck 给 Pi 供电。**插官方 PSU 时拔掉这根**，PSU 直插 Pi 的 Type-C |

**电源开关**：位于"电池 → 板载稳压器"之间，只控制电池给板子供电。
**管不住回灌**——回灌从 Pi 一侧进入（Type-C 共享线或数据线 VBUS），进入点在开关下游。

**待确认的隔离性实验**（约 1 分钟）：Pi 插官方 PSU，STM32 开关关掉，只留数据线：

- 板上 LED 不亮、`/imu/data_raw` 无数据 → VBUS 只喂 CH34x，隔离良好，数据线回灌无害
- 板子被"点亮"（LED 亮 / IMU 还在发数）→ VBUS 灌进主 5V 轨，STM32 板会从 Pi 的
  USB 口偷电（可达几百 mA），**直接挤占 Pi 的 USB 过流预算**，加剧下述问题

## 问题：USB 过流导致外设集体掉线（遥控一段时间后崩溃）

### 症状

遥控行驶一段时间后整个 stack 崩溃：STM32 串口报 `Errno 5`、雷达（ttyUSB0）、
Gemini 相机、手柄接收器**同一秒全部掉线**。

### 内核证据

```
usb usb4-port2: over-current change #N
usb usb2-port2: over-current change #N
```

6 小时内触发 24 次。每次跳闸整条 USB 总线断电重枚举，所有 USB 设备同时消失。

### 根因

Pi 5 对外设 USB 口有总电流限制（`usb_max_current_enable=1` 时 1.6A，
**与供电方式无关**）。外设叠加峰值就在门槛附近：

- Gemini 2L IR 投射器脉冲：峰值近 1A（最大电流户）
- MS200 雷达电机：~300mA（电机取电走 USB）
- 手柄接收器 + STM32 串口芯片：小电流

供电配置本身正确（`PSU_MAX_CURRENT=5000`、`usb_max_current_enable=1`、
输入 5.10V、`throttled=0x0`）——是外设峰值真的超了，**硬件问题**。

### 实测观察（2026-07-05）

- **满电电池**（STM32 buck → Type-C 共享线 → Pi）：不出现过流
- **官方 PSU 直插 Pi**：出现过（早前崩溃可能与电池亏电混杂，未做干净对照）

解释：系统处于临界状态，1.6A 限制两种供电下相同，跳到门槛哪一侧取决于输入
电压"硬度"（满电电池 buck 输出偏高且瞬态硬 → 同功率下电流略低）。另一个候选
因素是上面的数据线 VBUS 回灌（PSU 模式下 STM32 板偷电）——待隔离性实验确认。

### 解决方案

**根治：带独立供电的 USB 3.0 hub（约 ¥50-100）**，把 Gemini 2L 挂上去
（雷达也建议挪）。相机是最大电流户，挪走后 Pi 的 1.6A 预算立刻宽裕，
两种供电方式都不再跳闸。注意 Gemini 2L 必须走 USB 3.0 hub（USB 2.0 会
`color frame is not decoded`，见 CLAUDE.md）。

**台架持续调试推荐配置（不依赖电池续航）**：

```
官方 PSU ──→ Pi 5 (Type-C)
带电 USB3 hub ──→ Gemini 2L + 雷达
Pi 直插 ──→ STM32 数据线 + 手柄接收器（均为小电流）
电池（开关开）──→ 仅供 STM32 板待机 + 舵机保持
```

- Type-C 共享线拔掉（PSU 模式必须，避免反灌/地环流，见 CLAUDE.md 蜂鸣器问题）
- 电机不转时 STM32 板耗电极低，一块满电电池能撑很多天——电池焦虑的根源是
  之前 Pi + 相机 + 雷达全部吃电池，挪到 PSU + hub 后就没了
- 完全不用电池也可以，但 base_node 连不上 STM32 → 无 IMU/odom → EKF 不发
  odom→base_link TF → SLAM/定位模式全废，只剩相机预览。所以**完整 stack
  调试需要 STM32 有电**（电池待机即可；或用可调直流电源 8.4V 假电池替代）
- 跑动测试才切回纯电池模式（反正要无线）

### 验证方法

两种供电各跑一次遥控，同时观察：

```bash
journalctl -k -f | grep -i over-current
```

上 hub 后两种供电都应零跳闸。

### 软件加固（已部署，commit 169e251）

过流掉线曾暴露两个软件 bug（已修，勿回退）：

1. base_node 接收线程被 `TypeError` 杀死（watchdog 并发 `close()` 时 pyserial
   fd 置 None，`os.read(None)` 抛 TypeError 而非 OSError）→ 串口重连成功后
   **IMU 永久失联**，EKF 悄悄退化。现在 recv 线程接住该异常并存活。
2. shutdown 竞态时 publish 抛 `RCLError` 杀线程 → 已兜住。

效果：过流仍会发生（硬件问题软件治不了），但掉线后 base_node 自动重连并恢复
IMU，stack 不再整体崩溃，退化为短暂中断。

### 相关低电量报警（TODO）

`/battery` 话题（1Hz，STM32 电源输入电压）已有 Foxglove 表盘。计划在
supervisor 加低电量蜂鸣报警（2S lipo：7.0V 预警 / 6.6V 急促），手机 SPA
状态条加电压显示。
