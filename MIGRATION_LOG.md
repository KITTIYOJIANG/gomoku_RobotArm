# 迁移记录

## 2026-07-20：只读审计

- 扫描工作区一级、二级目录和项目标记。
- 识别 `gomoku_project` 为上位机，`gomoku_arm_controller` 为机械臂控制/下位机工程。
- 记录两个原工程在迁移前已经存在的 Git 修改/未跟踪内容；未清理、覆盖或格式化。
- 计算排除缓存/构建产物后的原工程聚合 SHA-256 指纹，见 `AUDIT_REPORT.md`。
- 发现 `SAFE_STAGE1` 参考固件与给定实机原始 PWM 协议冲突；决定只读复制并记录，不编译、不烧录、不改变上位机协议。

## 2026-07-20：直接复制

- `gomoku_project/localization/*.py` -> `app/vision/localization/*.py`（内容不变）。
- `gomoku_project/gomoku/camera_selector.py` -> `app/vision/camera_selector.py`（内容不变）。
- `gomoku_project/config/apriltag_board.json` -> `config/apriltag_board.json`（内容不变）。
- `gomoku_project/config/vision_grid.json` -> `config/vision_grid.json`（内容不变）。
- `gomoku_project/config/camera_intrinsics.example.json` -> `config/camera_intrinsics.example.json`（内容不变）。
- `gomoku_project/calibration_tools/current_board_corners.json` -> `calibration/legacy_board_corners.json`（重命名复制，内容不变）。
- `gomoku_arm_controller/firmware/jibot1_stage1` 的 143 个构建/阅读文件 -> `firmware/stm32_j1_reference`（内容不变，按排除规则选择）。

## 2026-07-20：整合版适配

- 所有新增文件均只写入 `J1_Gomoku_Integrated`；新代码运行时不依赖原工程绝对路径。
- GUI 从 PyQt5 迁移到 PySide6；定位算法本身保持原样，通过新的 BoardLocator/CameraWorker 调用。
- 串口从多层/跨仓库适配器收敛为 `SerialArmController` 单一所有者；普通写入用锁串行化，急停直接写 `$DST!`。
- 11 个动作 PWM 收敛到 `config/arm_actions.json` 唯一权威源；`actions.py` 只解析、验证和打印。
- 新增 `ArmStateMachine`、`ArmSequenceWorker`、固定 P77 序列、可中断等待和动作并发拒绝。
- 新增实时棋盘边界/15×15 网格/P77 覆盖、Board Locked/Lost、FPS 和独立相机 QThread。
- 新增左 65% 视觉、右 35% 控制/状态/日志的 PySide6 GUI；未实现能力明确禁用并标注“后续版本”。
- 新增 dry-run 模拟串口、测试画面、会话日志、启动/安装脚本、协议说明和 README。
- 在当前 Anaconda Python 3.12 中，PySide6 6.11.1 出现 QtCore DLL 过程不匹配；已将项目依赖固定为经验证可导入的 PySide6 6.8.3。

## 2026-07-20：验证结果

- `python -m compileall -q app tests`：通过。
- `python -m pytest -q`：30 passed。
- offscreen `python -m app.main --dry-run --test-pattern --smoke-test`：退出码 0；测试画面 CameraWorker 正常启动和释放，未连接 COM。
- 实际 ArmSequenceWorker dry-run（初始版本）发送顺序：`SOURCE_TOUCH_IDLE -> SOURCE_TOUCH_HOLD -> OBSERVE_HOLD -> CARRY_HIGH_HOLD -> P77_ABOVE_HOLD -> P77_TOUCH_HOLD -> P77_TOUCH_RELEASE -> P77_ABOVE_IDLE -> CARRY_HIGH_IDLE -> OBSERVE_IDLE`；已由下方最新标定记录取代。
- 原上位机完成后指纹：`455835E4D74C7A122FE17850F20E8F7E993CF893AFBBE02F6433FDE38CA14DA8`，与基线一致。
- 原下位机完成后指纹：`A6E07357837DA71C4AC5103BAFE6CB73BF1A93D83D0B18C86735F687073CC86A`，与基线一致。
- 10 个复制的 localization 文件和 143 个固件参考文件与对应来源逐文件 SHA-256 一致。
- 固件参考目录未发现 `.git`、IDE、缓存、虚拟环境或 build/Debug/Release/Objects/Listings 目录。

## 2026-07-20：P77 最新标定与稳定四角追加

- `config/arm_actions.json` 是唯一 PWM 权威源。
- 将 `P77_ABOVE_HOLD`、`P77_ABOVE_IDLE`、`P77_TOUCH_HOLD`、`P77_TOUCH_RELEASE` 的 000 从 1580 改为 1560，其他通道不变。
- 将旧 `CARRY_HIGH_HOLD/IDLE` 替换为语义明确的 `CARRY_HIGH_P77_HOLD/IDLE`，000=1560，其他通道不变；未保留 LEGACY 1580 动作。
- 完整流程显式从 `OBSERVE_IDLE` 开始，所有 P77 去程/触碰/回程动作均引用 1560 版本。
- 新增 `BoardTracker`：角点固定为 TL/TR/BR/BL，指数平滑；非检测帧保持 LOCKED，机械臂遮挡或短暂漏检保持 FROZEN，达到连续失败阈值或超时后才 LOST。
- `overlay.py` 新增显示缓冲区四角红点、白色轮廓、标签和可选坐标；四角、边界、网格、P77 共享同一 smoothed corners/homography。
- CameraWorker 显式分离 detection frame 与 display frame；红点不会进入 AprilTag 或棋子识别输入。
- 直接复用原 `gomoku_project/vision/stone_detector.py`，放棋/完整流程重新定位后自动重新识别，也可由视觉调试按钮触发。
- 新增 Corner Status 和 Piece Status；四角显示复选框默认开启。
- 自动测试增至 42 项并全部通过；完整 ArmSequenceWorker dry-run 的实际顺序为 `OBSERVE_IDLE -> SOURCE_TOUCH_IDLE -> SOURCE_TOUCH_HOLD -> OBSERVE_HOLD -> CARRY_HIGH_P77_HOLD -> P77_ABOVE_HOLD -> P77_TOUCH_HOLD -> P77_TOUCH_RELEASE -> P77_ABOVE_IDLE -> CARRY_HIGH_P77_IDLE -> OBSERVE_IDLE`，P77 路径全部含 `#000P1560`，无 `#000P1580`。

## 排除项

- 未复制 `.git`、`.idea`、`.vscode`、`__pycache__`、`.pytest_cache`、`.venv`、`venv`、build、dist、Debug、Release、Objects、Listings 或大型日志。
- 未移动或删除任何原始文件。
- 未编译、烧录或执行 STM32 固件。
- 未连接 COM6，未发送真实机械臂动作。
