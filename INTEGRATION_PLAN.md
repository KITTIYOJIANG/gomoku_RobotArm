# J1 五子棋机械臂 V0.1 整合计划

## 范围

本版本实现固定点 P77 的人工触发安全取放、AprilTag 稳定棋盘跟踪、四角/网格/P77 实时覆盖、复用的棋子识别和串口/dry-run 控制。任意棋盘坐标到 PWM、AI 自动落子、吸取检测和自动重试仍不在 V0.1 范围内。

## 实施顺序

1. 完成只读扫描、候选识别、功能/依赖/风险审计，并保存原工程聚合指纹。
2. 创建独立目录 `J1_Gomoku_Integrated`，不修改原工程。
3. 复制上位机 AprilTag 定位包、摄像头选择逻辑、布局/网格/标定配置；复制固件阅读和构建所需源码，排除缓存、构建产物和用户 IDE 状态。
4. 用 `config/arm_actions.json` 建立唯一 PWM 权威源；Python 仅加载、解析、验证和打印命令。
5. 实现单一串口控制器、状态机、不可并发的动作 worker 和可中断等待。
6. 实现 `return_to_observe`、`pick_piece`、`place_to_p77`、`run_full_cycle`，通过结构化步骤强制安全中间点。
7. 用 CameraWorker 调用复用的 AprilTag pipeline，每 N 帧检测一次；BoardTracker 在每帧提供同一组平滑四角/homography，显示棋盘边界、15×15 网格、P77 和 TL/TR/BR/BL 红点。
8. 实现 PySide6 左 65% 画面、右 35% 控制/状态/日志 GUI；启动不连接、不运动。
9. 实现 `--dry-run`、可选测试画面、会话日志、错误处理和安全关闭。
10. 运行编译检查、导入检查、单元测试及 offscreen dry-run GUI smoke test。
11. 重算原工程指纹，确认未发生本次写入；输出仅由用户执行的实机测试步骤。

## 安全门槛

- 串口连接后状态只能是 `UNKNOWN`，必须由用户点击“回观察位”。
- `pick_piece` 仅允许 `OBSERVE_IDLE`；完成后必须是 `OBSERVE_HOLD`。
- `place_to_p77` 仅允许 `OBSERVE_HOLD` 且 Board Locked/P77 已显示。
- 去程必须 `CARRY_HIGH_P77_HOLD -> P77_ABOVE_HOLD -> P77_TOUCH_HOLD`，六个 P77 路径动作的 000 全部为 1560。
- 回程必须 `P77_TOUCH_RELEASE -> P77_ABOVE_IDLE -> CARRY_HIGH_P77_IDLE -> OBSERVE_IDLE`。
- 普通动作只在 ArmSequenceWorker 执行；GUI 主线程不 sleep。
- 急停由 GUI 线程直接调用串口控制器写 `$DST!` 并同时取消 worker 剩余步骤。
- 任何启动路径都不得自动打开 COM6 或发送动作。

## 验证策略

- JSON/正则测试验证每条动作 ASCII、舵机 ID、PWM、时间和气泵状态。
- 状态机测试验证前置状态、急停锁存和并发拒绝。
- 安全序列测试验证 P77 前后中间点及运输期间 005=2500。
- fake serial/dry-run 测试验证命令顺序，不访问硬件。
- offscreen GUI smoke test只创建并关闭窗口，不连接摄像头或 COM。

## 明确不执行

- 不连接 COM6。
- 不运行任何真实取料、放棋、HOME 或手动姿态。
- 不编译或烧录 STM32。
- 不修改、移动或清理两个原工程中的任何文件。
