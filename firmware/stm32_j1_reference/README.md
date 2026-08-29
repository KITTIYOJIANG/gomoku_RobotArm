# Jibot1 第一阶段安全调试固件

本目录基于众灵 `Jibot1-32+逆运动学-基础版-250508` 源码建立。目标芯片为
STM32F103C8（中容量、64 KiB Flash、20 KiB RAM），舵机总线和调试串口均为
115200 baud。

## 与原厂程序的关键区别

- 启动时不发送 `#255P1500T2000!`，也不执行 W25Q64 中保存的动作组。
- 启动只依次发送 `#000PRAD!` 到 `#005PRAD!` 读取真实位置。
- 任一舵机无响应或当前位置不在第一阶段窄限位内，立即进入 `FAULT`，不运动。
- 只接受安全层命令；原始多舵机 PWM 命令不会被转发。
- 动作、位置轮询、蜂鸣器均由非阻塞状态机处理；运动中仍可接收 `stop`。

注意：部分总线舵机可能已在自身 EEPROM 中启用了上电启动位置（`PCSD/PCSM`）。
本固件不会改写这些持久设置，因此“MCU 不发送启动动作”不能消除舵机自身上电
策略。首次烧录前必须悬空/支撑机械臂，并准备物理断电。

## 关节映射与第一阶段参数

| ID | 名称 | 第一阶段范围 | 测试幅度 | HOME | HOME中使用 |
|---:|---|---:|---:|---:|:---:|
| 000 | BASE / 底座旋转 | 1494–1534 | ±5 | 1514 | 是 |
| 001 | SHOULDER / 下臂 | 1502–1542 | ±5 | 1522 | 是 |
| 002 | ELBOW / 中臂 | 1520–1560 | ±5 | 1540 | 是 |
| 003 | UPPER_ARM / 上臂 | 1512–1552 | ±5 | 1532 | 是 |
| 004 | TOOL_ROLL / 泵嘴旋转 | 1485–1525 | ±5 | 1505 | 是 |
| 005 | SUCTION / 吸取执行器 | 不读取位置 | 不参与 | 不参与 | 否 |

这些只是初次通电用的临时窄窗口，不是最终机械限位。测试顺序被固件硬限制为：

```text
003 -> 002 -> 001 -> 004 -> 000
```

同一关节的流程为：读取当前位 → 当前位+5 → 当前位 → 当前位-5 → 当前位。
每步使用 800 ms，2 s 超时，到位容差为 2；每步成功后 PB5 蜂鸣 50 ms。

## 串口命令

命令不区分大小写，以换行结束；紧急停止也兼容原控制器的 `$DST!`。

```text
status
beep,3
test_joint,003
stop
$DST!
rescan
home
observe
pick_ready
park
```

- `stop`/`$DST!`：立即广播 `#255PDST!`。动作中停止会锁存故障，需 `rescan`。
- `beep,1` 到 `beep,5`：非阻塞蜂鸣，不发送任何舵机动作，可用于通信确认。
- `rescan`：不运动，重新读取并校验 6 个当前位置。
- `home`：仅在规定的关节测试全部完成后接受，按
  `003,002,001,000,004` 逐关节移动；005 不参与。
- `observe`、`pick_ready`、`park`：仅保留接口，返回 `ERR POSE_NOT_IMPLEMENTED`。
- 超出配置范围返回 `ERR JOINT_LIMIT`；超时或读数异常会发送停止命令并进入故障。

使用仓库内 Python CLI 运行测试时，按 `Ctrl+C` 会先在同一串口发送停止命令，再
退出程序；物理断电仍是最终安全手段。

典型日志：

```text
MOVE id=003 target=1505 time=800
STEP action=TEST step=POS id=003 target=1505 current=1505 elapsed=... result=OK
OK TEST_DONE id=003 verified_mask=0x08
```

## 构建

Keil 工程为 `Project/Jibot1-32.uvproj`，已改为中容量启动文件并定义
`SAFE_STAGE1`。也可使用 Arm GNU Toolchain：

```text
make
```

输出在 `build/`：

- `jibot1_stage1_safe.elf`
- `jibot1_stage1_safe.hex`
- `jibot1_stage1_safe.bin`
- `jibot1_stage1_safe.map`

构建完成不等于已在实机验证。未经操作者确认，不应自动烧录本固件。
