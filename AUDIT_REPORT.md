# J1 五子棋机械臂整合审计报告

审计日期：2026-07-20  
审计方式：只读目录扫描、源码检索、依赖文件检查和原工程内容指纹计算。审计期间未连接串口、未运行机械臂、未编译或烧录固件。

## 1. 目录识别

### 原始上位机

- 绝对路径：`D:\Projects\Embodied\RobotArms\gomoku_project`
- 选择依据：包含 `main.py`、`run.py`、`requirements.txt`、PyQt5 GUI、OpenCV 摄像头线程、`pupil-apriltags` 检测、`localization/` 棋盘定位、`vision/` 棋盘/棋子处理、相机与棋盘标定配置，以及上位机串口适配器。

### 原始下位机/机械臂控制工程

- 绝对路径：`D:\Projects\Embodied\RobotArms\gomoku_arm_controller`
- 固件参考子目录：`D:\Projects\Embodied\RobotArms\gomoku_arm_controller\firmware\jibot1_stage1`
- 选择依据：包含机械臂串口协议构造、串口控制、安全 GUI、舵机动作/姿态与 STM32F103C8 源码；固件中存在 `z_action.c`、`z_gpio.c`、`z_ps2.c`、Keil 工程和 USART 实现。

### 其他候选说明

- `gomoku_project/stm32_firmware` 存在但为空，不能作为可复制的固件工程。
- `gomoku_arm_controller/firmware/jibot1_stage1` 是当前工作区唯一完整 STM32 工程，但它是带 `SAFE_STAGE1` 宏的安全调试版本。其 README 和安全控制层明确表示：已编译配置只接受安全控制台命令，不转发原始多舵机 PWM。它与本项目给定的“实机当前接受原始 PWM”事实存在明显冲突。本整合版不修改固件，只将其作为可追溯参考复制，并在 `firmware/PROTOCOL_NOTES.md` 中区分“实机约定”和“参考固件编译配置”。

### 原工程基线

内容指纹排除了 `.git`、IDE 配置、缓存、虚拟环境和构建产物；用于完成后的只读复核。

| 工程 | 文件数 | SHA-256 聚合指纹 |
|---|---:|---|
| `gomoku_project` | 273 | `455835E4D74C7A122FE17850F20E8F7E993CF893AFBBE02F6433FDE38CA14DA8` |
| `gomoku_arm_controller` | 224 | `A6E07357837DA71C4AC5103BAFE6CB73BF1A93D83D0B18C86735F687073CC86A` |

两个 Git 工作区在本次整合开始前已经包含较多已修改和未跟踪文件；这些均视为用户现有内容。本次操作只读取和复制，不在原工程中写入。

## 2. 上位机功能清单

| 功能 | 结论 | 依据与处理 |
|---|---|---|
| 摄像头打开 | 需要适配 | `main.py`、`gomoku/camera_interface.py` 使用 OpenCV；旧入口在 GUI 导入前抢占摄像头，整合版改为用户点击后由 QThread 打开。 |
| 实时画面 | 需要适配 | 旧摄像头线程通过 OpenCV 独立窗口显示；复用采集/设备选择思路，输出改为 Qt Signal。 |
| AprilTag 检测 | 可直接复用 | `localization/apriltag_detector.py` 封装 `pupil-apriltags`，包含 ID、Hamming、margin 过滤。 |
| 棋盘四角定位 | 可直接复用 | `localization/board_localizer.py` 使用 RANSAC homography、重投影误差和几何校验。 |
| 棋盘网格绘制 | 需要适配 | `vision/grid_mapper.py` 和 `vision/standard_grid.py` 已有网格逻辑；整合版需把网格投影回实时画面。 |
| P77 目标点绘制 | 缺失 | 原工程有 `(7,7)` 星位/测试示例，但没有“固定机械臂 P77”安全目标覆盖层；整合版新增，仅显示固定 `(7,7)`。 |
| 相机标定 | 存在风险 | `calibration_tools/current_board_corners.json` 有实机四角；`camera_intrinsics.example.json` 的内参仍为空。禁止推测内参，整合版保留原配置并明确告警。 |
| COM 串口 | 需要适配 | 上位机适配器与下位机控制工程均使用 pyserial；旧代码有跨仓库绝对路径与多层包装，整合版复用协议格式并集中为单一串口所有者。 |
| 机械臂控制 | 需要适配 | `gomoku/stm32_controller.py` 和下位机 Python 控制层存在，但面向旧坐标映射/安全调试流程；V0.1 改为固定姿态动作库。 |
| GUI 框架 | 需要适配 | 原 GUI 使用 PyQt5，且摄像头/业务耦合；目标要求 PySide6 左右布局，因此保留交互经验、重建线程安全 GUI。 |
| 日志系统 | 缺失 | 未发现满足会话文件与 GUI 日志窗口要求的统一日志；整合版新增。 |

## 3. 下位机功能清单

| 功能 | 结论 | 证据/风险 |
|---|---|---|
| 串口接收 | 存在 | `src/z_usart.c` 可识别 `$`、`#`、`{`、`<` 开头并按终止符收包；`SAFE_STAGE1` 则由 `USER/safe_console.c` 接收换行命令。 |
| 舵机多设备协议 | 源码存在，当前参考编译禁用 | 旧路径接受 `{#000P...!#001P...!}` 并交给 `parse_action()`；`SAFE_STAGE1` 构建明确不转发它。 |
| `$DST!` 急停 | 存在 | 旧 `parse_group_cmd()` 和安全控制台均处理 `$DST!`；安全层转发广播停止 `#255PDST!`。 |
| 气泵控制 | 协议可表达，参考安全层禁用 ID 005 运动 | 给定实机约定使用 005=1500/2500；旧动作解析可写 005，`USER/servo_bus.c` 的 Stage-1 guard 将 005 设为只读。 |
| 蜂鸣器 | 存在 | PB5 蜂鸣器在 `z_gpio.c`；安全控制台支持 `beep,1..5`。本 V0.1 GUI 只显示禁用的后续功能按钮。 |
| 动作组 | 源码存在 | `z_action.c` 支持 W25Q64 动作组；`SAFE_STAGE1` 启动/编译路径不执行这些动作组。 |
| 到位回传 | 仅安全调试层存在 | Stage-1 通过舵机位置读取与容差判定输出 `OK TEST_DONE` 等；给定的实机原始 PWM 协议没有可依赖的通用到位 ACK。 |
| 当前舵机位置读取 | 存在但不能作为 V0.1 通用接口 | Stage-1 使用 `#nnnPRAD!` 读取 000–005；旧原始协议源码也包含读命令能力，但上位机固定动作流程不依赖它。 |
| 波特率 | 115200 | `USER/safe_main.c`、`src/z_usart.c` 和 Python 控制层一致。 |
| 使用的 UART | USART1 主机命令；USART3 舵机总线 | Stage-1 `safe_console` 使用 USART1，`servo_bus` 使用 USART3；源码也初始化 USART2，但整合上位机只面对 Windows COM 端口。 |

### 协议结论

V0.1 以用户给定且已实机确认的 ASCII 协议为权威：COM6、115200、原始单/多舵机命令和 `$DST!`。复制的 Stage-1 固件仅供阅读，不能被当作当前实机固件重编译或烧录；也不能据此改变 GUI 发送格式。

## 4. 依赖分析

| 依赖 | 原工程 | 整合版决策 |
|---|---|---|
| Python | 源码使用 `X | None` 等语法，要求 3.10+ | 明确要求 Python 3.10+，推荐 3.11/3.12。 |
| OpenCV | `opencv-python>=4.5.0` | 复用，整合版约束为 `>=4.8`。 |
| Qt | PyQt5>=5.15 | 目标明确要求 PySide6；不同时安装/导入两套 Qt。 |
| AprilTag | `pupil-apriltags==1.0.4.post11` | 原样保留单一后端，不引入其他 AprilTag 库。 |
| pyserial | `pyserial>=3.5` | 复用。 |
| numpy | `numpy>=1.19.0` | 复用，整合版使用与现代 Python 兼容的 `>=1.24`。 |
| pytest | 两工程均使用 | 作为测试依赖保留。 |
| ffmpeg | `camera_selector.py` 可选使用 | 仅用于按 DirectShow 名称解析摄像头；缺失时回退数字索引。 |

## 5. 风险与缓解

1. **参考固件协议冲突（高）**：Stage-1 编译配置拒绝原始 PWM，而实机约定要求原始 PWM。整合版不编译/烧录固件，运行前必须确认 COM6 所连设备确实运行兼容固件。
2. **无通用到位反馈（高）**：固定动作命令没有可靠 ACK；上位机只能用命令 T 值加等待余量。真实动作必须低速、清场、逐步验证。
3. **P77 碰撞路径（高）**：禁止 OBSERVE 与 P77_TOUCH 直接互转；序列和测试强制经过 CARRY_HIGH 与 P77_ABOVE。
4. **原工程绝对路径（中）**：`gomoku/stm32_controller.py` 和文档含跨仓库绝对路径；整合版不复用这些路径，所有运行资源从项目根目录解析。
5. **旧摄像头生命周期（中）**：旧 `main.py` 启动时打开摄像头，旧 GUI/CameraThread 混用 OpenCV 窗口。整合版由独立 QThread 在用户点击后管理设备。
6. **Qt 框架迁移（中）**：原工程 PyQt5，目标 PySide6；业务定位模块可直接复用，GUI 信号/控件需重建。
7. **标定不完整（高）**：AprilTag 物理尺寸和相机内参为 `null`，只能进行 2D homography；不得声称获得毫米级/机械臂坐标标定。
8. **定位遮挡（中）**：机械臂运动时可能遮挡 AprilTag。动作期间冻结放棋触发，结束后请求重新定位。
9. **串口竞争（高）**：必须只有一个 `SerialArmController` 持有 COM；普通写入互斥，急停不排队等待动作延时。
10. **动作 PWM 数据漂移（高）**：所有动作只存在于 `config/arm_actions.json`；GUI、序列和 Python 不保存 PWM 副本。
11. **原工作区已有脏文件（审计）**：不能用简单 `git status` 把既有改动归因于整合。完成时使用相同排除规则重算聚合指纹。

## 6. 审计决策

- 直接复用：AprilTag detector/localizer/temporal pipeline、相机设备选择、AprilTag 布局和棋盘网格方向配置。
- 最小适配：把定位包复制到 `app/vision/localization`；由新的 QThread 摄像头 worker 调用；使用新的 overlay 显示网格/P77。
- 新建：PySide6 GUI、串口单一所有者、固定动作 JSON、状态机、安全序列、动作 worker、dry-run、会话日志和验收测试。
- 只读参考：Stage-1 固件工程。不得编译、烧录或用于推导新的不兼容协议。
