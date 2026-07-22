# 文件来源映射

所有路径均相对于 `D:\Projects\Embodied\RobotArms`。`直接复制` 表示内容未改；`适配` 表示在整合目录内新建包装层或转换框架，原文件不变。

| 整合版文件/目录 | 原始来源 | 处理方式 |
|---|---|---|
| `app/vision/localization/*.py` | `gomoku_project/localization/*.py` | 直接复制完整定位包（10 个文件），保持原有相对导入、Tag ID、homography、几何阈值和时序稳定逻辑。 |
| `app/vision/camera_selector.py` | `gomoku_project/gomoku/camera_selector.py` | 直接复制；保留 DirectShow 设备名优先与数字索引回退。 |
| `config/apriltag_board.json` | `gomoku_project/config/apriltag_board.json` | 直接复制；ID 15/16/17/18、角色、坐标和阈值不变。 |
| `config/vision_grid.json` | `gomoku_project/config/vision_grid.json` | 直接复制；15×15、row-down/col-right 坐标方向不变。 |
| `config/camera_intrinsics.example.json` | `gomoku_project/config/camera_intrinsics.example.json` | 直接复制；保留内参缺失状态，不伪造标定数据。 |
| `calibration/legacy_board_corners.json` | `gomoku_project/calibration_tools/current_board_corners.json` | 复制并重命名；保留旧固定相机棋盘四角，仅作为标定参考。 |
| `firmware/stm32_j1_reference/` | `gomoku_arm_controller/firmware/jibot1_stage1/` | 选择性原样复制构建/阅读文件；排除 build、Debug、Release、CMSIS 文档、IDE 用户状态和 HTML。固件源码不修改。 |
| `app/vision/board_locator.py` | 上述 `localization` 包 | 新建适配层；按配置节流检测并保留原 pipeline。 |
| `app/vision/camera_worker.py` | `gomoku_project/gomoku/camera_interface.py`、`camera_selector.py` | 新建 PySide6 QThread 适配；复用摄像头选择策略，去除 OpenCV 独立窗口和 GUI 启动抢占。 |
| `app/vision/overlay.py` | `localization/visualization.py`、`vision_grid.json` | 新建实时覆盖层；复用定位结果并增加网格/P77，不改变坐标方向。 |
| `app/vision/board_tracker.py` | 原 temporal localization + 追加稳定显示要求 | 新建 TL/TR/BR/BL 平滑缓存和 LOCKED/FROZEN/LOST 状态。 |
| `app/vision/stone_detector.py` | `gomoku_project/vision/stone_detector.py` | 直接复制，内容和 SHA-256 不变。 |
| `app/vision/piece_recognizer.py` | 上述 stone detector + 稳定 homography | 新建 detection-frame-only 适配层。 |
| `app/vision/calibration.py` | `localization/camera_intrinsics.py`、原配置 | 新建只读配置加载器；只在有效内参存在时启用去畸变。 |
| `app/arm/controller.py` | `gomoku_arm_controller/arm_controller/serial_protocol.py`、`stm32_controller.py`、`safety_gui.py` | 新建最小串口适配；保留 ASCII 格式、pyserial、115200 和写锁思路，不复用旧跨仓库路径。 |
| `app/arm/actions.py` | 用户给定的 11 个实机确认动作 | 新建加载/验证层；PWM 本体仅在 `config/arm_actions.json`。 |
| `app/arm/state.py` | 用户给定 V0.1 状态规则 | 新建显式状态机。 |
| `app/arm/sequences.py` | 用户给定 P77 安全序列 | 新建结构化动作/等待步骤；禁止直接 OBSERVE↔P77_TOUCH。 |
| `app/arm/worker.py` | 用户给定线程与急停要求 | 新建唯一动作队列 QThread，可中断等待。 |
| `app/gui/*.py`、`app/main_window.py` | `gomoku_project/gomoku/gui_qt.py` 的交互目标 + 用户界面要求 | 使用 PySide6 重建；不直接复制 PyQt5 控件代码。 |
| `app/config.py`、`app/logging_config.py` | 原工程配置模式 + 用户要求 | 新建相对路径配置和会话日志。 |
| `tests/*.py` | 用户验收条件 | 新建 dry-run/静态验收测试。 |
| `README.md`、审计/迁移/协议文档、脚本 | 本次整合审计与目标要求 | 新建。 |

## 固件复制范围

复制 143 个文件，包括 `CMSIS/CM3`、`Libraries/inc`、`Libraries/src`、`Libraries/Startup`、`Project`（排除 `.uvgui*`）、`src`、`Startup`、`USER`、`Makefile`、链接脚本和原 README。未复制 `.git`、缓存、虚拟环境、build/Debug/Release、Objects/Listings 或 CMSIS HTML 文档。

固件目录内的每个文件均保持原相对路径；例如整合版 `firmware/stm32_j1_reference/USER/safe_main.c` 一一对应原始 `gomoku_arm_controller/firmware/jibot1_stage1/USER/safe_main.c`。完成校验的 143 个目标文件均与对应来源 SHA-256 一致。

## 直接复用 Python 文件逐项映射

| 整合版文件 | 原始来源 | 处理方式 |
|---|---|---|
| `app/vision/localization/__init__.py` | `gomoku_project/localization/__init__.py` | 直接复制 |
| `app/vision/localization/apriltag_detector.py` | `gomoku_project/localization/apriltag_detector.py` | 直接复制 |
| `app/vision/localization/board_localizer.py` | `gomoku_project/localization/board_localizer.py` | 直接复制 |
| `app/vision/localization/camera_intrinsics.py` | `gomoku_project/localization/camera_intrinsics.py` | 直接复制 |
| `app/vision/localization/geometry.py` | `gomoku_project/localization/geometry.py` | 直接复制 |
| `app/vision/localization/layout.py` | `gomoku_project/localization/layout.py` | 直接复制 |
| `app/vision/localization/models.py` | `gomoku_project/localization/models.py` | 直接复制 |
| `app/vision/localization/pipeline.py` | `gomoku_project/localization/pipeline.py` | 直接复制 |
| `app/vision/localization/temporal_localizer.py` | `gomoku_project/localization/temporal_localizer.py` | 直接复制 |
| `app/vision/localization/visualization.py` | `gomoku_project/localization/visualization.py` | 直接复制 |
| `app/vision/camera_selector.py` | `gomoku_project/gomoku/camera_selector.py` | 直接复制 |

## 新建文件逐项映射

| 文件 | 来源/依据 |
|---|---|
| `app/__init__.py` | 整合版包元数据 |
| `app/main.py` | 用户启动、dry-run 和 smoke-test 要求 |
| `app/main_window.py` | 用户状态、安全、GUI 和线程协调要求 |
| `app/config.py` | 用户集中配置和相对路径要求 |
| `app/logging_config.py` | 用户 GUI+会话文件日志要求 |
| `app/arm/__init__.py` | 整合版包导出 |
| `app/arm/actions.py` | 用户动作验证要求；数据从唯一 JSON 加载 |
| `app/arm/controller.py` | 原串口格式/写锁思路 + 用户单一串口所有者要求 |
| `app/arm/state.py` | 用户列出的 V0.1 状态和转换 |
| `app/arm/sequences.py` | 用户固定 P77 硬安全序列 |
| `app/arm/worker.py` | 用户独立动作 worker、取消和非阻塞 GUI 要求 |
| `app/vision/board_locator.py` | 复用 localization 包的节流适配层 |
| `app/vision/camera_worker.py` | 原 CameraThread/selector + 用户 QThread 要求 |
| `app/vision/overlay.py` | 原定位结果/网格方向 + 用户 P77 覆盖要求 |
| `app/vision/board_tracker.py` | 追加的角点平滑、冻结和丢失判定要求 |
| `app/vision/piece_recognizer.py` | 原棋子检测器 + 重新识别要求 |
| `app/vision/calibration.py` | 原内参格式 + 禁止伪造标定要求 |
| `app/gui/__init__.py` | 整合版 GUI 包 |
| `app/gui/camera_panel.py` | 用户左侧 65% 实时视觉要求 |
| `app/gui/control_panel.py` | 用户右侧连接、动作、扩展、手动、安全区要求 |
| `app/gui/status_panel.py` | 用户 Camera/COM/Board/Arm/Action 状态要求 |
| `app/gui/log_panel.py` | 用户 GUI 日志窗口要求 |
| `config/app_config.json` | 用户 COM、波特率、相机、等待、P77、检测频率、日志配置 |
| `config/arm_actions.json` | 用户给定的 11 个不可修改动作（唯一 PWM 权威源） |
| `config/vision_config.json` | 原视觉配置的相对路径索引，不复制 Tag/网格数据 |
| `tests/test_action_commands.py` | 动作命令验收条件 |
| `tests/test_state_machine.py` | 状态机验收条件 |
| `tests/test_safe_sequences.py` | P77 硬安全条件 |
| `tests/test_dry_run_sequences.py` | 无硬件 TX 顺序条件 |
| `tests/test_imports.py` | 核心模块导入条件 |
| `tests/test_board_tracker_overlay.py` | 四角顺序、平滑、冻结、丢失、显示缓冲隔离和棋子输入验收 |
| `tests/test_worker_cancellation.py` | 急停/取消不得推进后续动作验收 |
| `scripts/install_dependencies.bat` | 用户依赖安装脚本要求 |
| `scripts/run_gui.bat` | 用户启动/错误保留要求 |
| `firmware/PROTOCOL_NOTES.md` | STM32 源码审计 + 用户实机协议事实 |
| `AUDIT_REPORT.md` | 本次只读审计 |
| `INTEGRATION_PLAN.md` | 本次阶段计划 |
| `MIGRATION_LOG.md` | 本次迁移事实 |
| `SOURCE_MAP.md` | 本文件 |
| `README.md` | 安装、运行、安全和实机交接要求 |
| `requirements.txt` | 原依赖 + PySide6 目标框架；只保留单一 AprilTag 后端 |
| `pytest.ini`、`.gitignore`、`assets/.gitkeep`、`models/.gitkeep`、`logs/.gitkeep` | 整合版测试/工作目录维护 |
