# J1 Gomoku Integrated V0.1

固定点 P77 五子棋机械臂整合开发版。左侧显示摄像头、稳定棋盘四角红点、15×15 网格和固定 P77；右侧提供连接、视觉调试、状态、安全动作、手动姿态、急停和日志。

## Development

Returning to the project? Start with [START_HERE.md](START_HERE.md), then run:

```powershell
python tools\project_doctor.py
python tools\resume_project.py
python tools\smoke_test.py
```

Maintenance references: [Architecture](docs/ARCHITECTURE.md), [Hardware Setup](docs/HARDWARE_SETUP.md), [Calibration](docs/CALIBRATION.md), and [Development Workflow](docs/DEVELOPMENT_WORKFLOW.md).

## 重要安全边界

- 程序启动不会自动连接摄像头或 COM，也不会自动运动。
- 本版本只支持固定 `(7,7)` / P77，不计算任意棋盘坐标到 PWM。
- 严禁从 OBSERVE 直接到 P77_TOUCH，或从 P77_TOUCH 直接回 OBSERVE。
- 最新 P77 标定要求六个 P77 安全路径姿态的 000 通道全部为 1560；完整流程不保留或引用 1580 旧动作。
- 真实运动没有可靠通用到位反馈；当前使用动作 T 值加等待余量。
- 工作区内复制的 `SAFE_STAGE1` 固件与给定实机原始 PWM 协议有冲突。不要编译、烧录或用它替换当前实机固件；详见 `firmware/PROTOCOL_NOTES.md`。
- 最终安全手段是物理断电；软件急停不能替代清场、支撑和断电准备。

## 环境与安装

要求 64 位 Python 3.10+（推荐 3.11/3.12）。在项目根目录运行：

```powershell
python -m pip install -r requirements.txt
```

也可双击 `scripts\install_dependencies.bat`。AprilTag 只使用原工程已有的 `pupil-apriltags==1.0.4.post11`，不要同时安装多个同类后端。

## 启动

项目根目录：

```text
D:\Projects\Embodied\RobotArms\J1_Gomoku_Integrated
```

普通 GUI（仍然不会自动连接）：

```powershell
cd D:\Projects\Embodied\RobotArms\J1_Gomoku_Integrated
python -m app.main
```

开发用相机测试画面（仍不会自动连接 COM）：

```powershell
python -m app.main --test-pattern
```

批处理入口会切换到项目根目录，并在错误时保留窗口：

```powershell
scripts\run_gui.bat
```

实机 GUI 命令是 `python -m app.main`。界面不再提供 DRY RUN；命令本身不连接硬件，只有用户在 GUI 中选择端口并点击连接后，动作按钮才会向下位机发送。

## 相机与 AprilTag

- 默认优先设备名：`USB 2.0 Camera`；无法解析名称时回退索引 0。
- 可用环境变量 `GOMOKU_CAMERA_ID` 指定数字索引；设备名解析需要系统存在 ffmpeg，否则自动回退。
- AprilTag ID 和角色保持原配置：15=TL、16=TR、17=BR、18=BL。
- 默认每 4 帧运行一次 AprilTag 检测。
- 必须连续满足原定位包的稳定帧规则，GUI 才显示 `BOARD LOCKED` 和 P77。
- BoardTracker 以固定 `TL, TR, BR, BL` 顺序缓存并平滑四角。非检测帧继续显示同一组红点、网格和 P77。
- 机械臂遮挡或短暂漏检时显示 `BOARD LOCKED · FROZEN` / `4/4 FROZEN`；只有连续失败或超时后才变为 `0/4 LOST`。
- “视觉调试”默认勾选“显示棋盘四角红点”，可选显示像素坐标。所有覆盖层先画在 display frame，再整体交给 QLabel 缩放。
- 棋子识别复用原上位机 `stone_detector.py`，只读取未绘制红点的 detection frame；放棋或完整流程后在重新锁定棋盘时自动重新识别。
- 相机内参示例仍是 `CAMERA_INTRINSICS_MISSING`，本项目没有伪造内参。当前仅使用原 2D homography。

## COM6 与状态

1. 端口下拉默认 COM6，波特率固定 115200。
2. 点击“连接 COM6”后状态只会变成 `UNKNOWN`。
3. 确认机械臂周围无障碍，再点击“回观察位”；成功后变成 `OBSERVE_IDLE`。
4. `OBSERVE_IDLE` 允许取料，不允许下棋。
5. 取料完成后为 `OBSERVE_HOLD`，显示“已取料，等待下棋”；只有 Board Locked/P77 显示时允许下棋。
6. ERROR 或 ESTOP 不自动恢复。用户只能通过气泵关闭、HOME/手动姿态或回观察位进行显式恢复。

## 当前动作

- 回观察位：只发送 `OBSERVE_IDLE`。
- 取料：`SOURCE_TOUCH_IDLE -> SOURCE_TOUCH_HOLD -> 等待700ms -> OBSERVE_HOLD`。
- 下棋到 P77：`CARRY_HIGH_P77_HOLD -> P77_ABOVE_HOLD -> P77_TOUCH_HOLD -> P77_TOUCH_RELEASE -> 等待700ms -> P77_ABOVE_IDLE -> CARRY_HIGH_P77_IDLE -> OBSERVE_IDLE`。
- 完整固定点流程：`OBSERVE_IDLE -> SOURCE_TOUCH_IDLE -> SOURCE_TOUCH_HOLD -> 等待700ms -> OBSERVE_HOLD -> CARRY_HIGH_P77_HOLD -> P77_ABOVE_HOLD -> P77_TOUCH_HOLD -> P77_TOUCH_RELEASE -> 等待700ms -> P77_ABOVE_IDLE -> CARRY_HIGH_P77_IDLE -> OBSERVE_IDLE -> 重新定位 -> 重新识别棋子`。
- `CARRY_HIGH_P77_*`、`P77_ABOVE_*`、`P77_TOUCH_*` 的 000 均为 1560；PWM 只维护在 `config/arm_actions.json`。
- 11 个单姿态动作：执行前均弹出障碍确认。
- 急停：立即发送 `$DST!`，取消未执行步骤并锁存 ESTOP。
- 气泵关闭：发送 `#005P1500T0500!`；若已知状态可能失效，会进入 ERROR。

## 日志

GUI 右侧显示日志，同时写入：

```text
logs/session_YYYYMMDD_HHMMSS.log
```

日志包括连接、定位、状态、步骤、实际 TX 字符串、等待、完成、失败、急停和串口错误。

## 测试

```powershell
python -m pytest -q
python -m compileall -q app tests
```

无显示器 GUI smoke test（PowerShell）：

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -m app.main --test-pattern --smoke-test
```

此 smoke test 只启动生成的测试画面 worker，不打开真实摄像头，也不连接 COM。

## 实机测试顺序（必须由操作者执行）

1. 不放棋盘/棋子，机械臂悬空或可靠支撑；准备物理断电，清空工作空间。
2. 确认 COM6 控制器运行兼容 `{#...}`/`$DST!` 的既有固件，不是复制目录中的 SAFE_STAGE1 构建。
3. 启动普通 GUI，但暂不连接 COM；只连接相机，确认 Tag 15–18、网格方向和 P77 圆点无误。
4. 清空机械臂工作区，确认物理断电手段可用，再准备连接下位机。
5. 连接 COM6 后状态应为 UNKNOWN；先测试急停，确认控制器停止响应正确，再由用户恢复。
6. 在无棋盘/无吸取负载条件下，逐个确认 HOME/OBSERVE/CARRY_HIGH_P77/P77_ABOVE 姿态；核对日志中的 000=1560，不要先测试 P77_TOUCH。
7. 放置棋盘后，低风险验证 `OBSERVE_HOLD -> CARRY_HIGH_P77_HOLD -> P77_ABOVE_HOLD` 的净空，并确认 ABOVE 到 TOUCH 期间底座不横向旋转。
8. 最后在操作者随时可断电的条件下分别测试“取料”和“下棋到 P77”；不要先用完整流程。
9. 每一步核对 005 在运输时为 2500、释放后为 1500，并保留会话日志。

## 尚未实现

Stage 7 已增加独立的 Rapid Calibration 页面：以 Stage 5 的 225 点 ABOVE
解析结果为只读 baseline，用 QUICK 5 / STANDARD 9 direct anchor 的 ΔPWM 双线性
correction field 生成 deployment candidate，并支持局部 direct anchor、验证、commit
和 rollback。Stage 7 的可见控制在显式连接 COM 后直接走下位机；只调整空间关节 `000..004`，`005` 气泵锁定。

Stage 7 committed deployment 尚未自动替换普通 Stage 5 点击悬停和 Stage 6 descent
的 ABOVE source。仍未实现相机-机械臂自动外参标定、自动落子、五子棋 AI、吸取
成功检测/重试和视觉闭环 correction。详细边界、测试和实机步骤见
`docs/STAGE7_OFFLINE_DELIVERY_REPORT.md`。

## 原工程与追溯

- 上位机：`D:\Projects\Embodied\RobotArms\gomoku_project`
- 下位机/机械臂控制：`D:\Projects\Embodied\RobotArms\gomoku_arm_controller`
- 整合版：`D:\Projects\Embodied\RobotArms\J1_Gomoku_Integrated`
- 审计、来源和变更分别见 `AUDIT_REPORT.md`、`SOURCE_MAP.md`、`MIGRATION_LOG.md`。
