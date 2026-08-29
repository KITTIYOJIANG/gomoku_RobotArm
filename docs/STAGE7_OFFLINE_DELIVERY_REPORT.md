# Stage 7 快速部署标定交付报告

报告日期：2026-08-25

状态：**OFFLINE PASS / HARDWARE NOT VERIFIED**

本轮没有连接真实串口，没有驱动机械臂，也没有改写 Stage 5 baseline、
Stage 6 下降标定、P77 动作表或固件。

## 1. 修改文件

新增：

- `app/stage7/__init__.py`
- `app/stage7/baseline.py`
- `app/stage7/settings.py`
- `app/stage7/session.py`
- `app/stage7/coordinator.py`
- `app/gui/rapid_calibration_panel.py`
- `config/stage7_rapid_calibration.json`
- `tests/test_stage7_backend.py`
- `tests/test_stage7_gui.py`
- `tests/test_stage7_live_jog.py`
- `docs/STAGE7_OFFLINE_DELIVERY_REPORT.md`

修改：

- `app/stage5/pwm_interpolator.py`：抽出无 pose 语义的双线性数值核心。
- `app/arm/controller.py`：增加只允许 `000..004` 的单关节 PWM API。
- `app/arm/state.py`：增加 Stage 7 ABOVE 与安全返回状态路径。
- `app/gui/control_panel.py`：加入默认收起的 Stage 7 页面。
- `app/main_window.py`：接入 session、GUI、安全移动、live jog 与审计日志。
- `scripts/run_gui.bat`：优先使用本机已安装 PySide6 的 Anaconda Python。
- `README.md`：补充 Stage 7 使用和边界。

## 2. 架构

```text
stage5_board_calibration.json (read-only + SHA-256)
        │
        ├─ 30 direct anchors
        └─ Stage 5 bilinear resolver → 195 points
                       │
                       ▼
             BaselineSnapshot (225)
                       │
     new direct anchor PWM - baseline PWM
                       │
                       ▼
          2-D per-joint ΔPWM correction field
                       │
                       ▼
     baseline + interpolated delta = candidate 225
                       │
              verify / local anchor
                       │
                       ▼
       session JSON + current deployment JSON
```

相机像素/homography 不进入 Stage 7 数据结构。棋盘 `(row,col)` 到 ABOVE PWM
与相机的 image pixel 到棋盘坐标保持解耦。

## 3. Baseline 数据来源

权威源：`calibration/stage5_board_calibration.json`。

- 15×15，`row=0` 在上，`col=0` 在左；
- 30 个 `direct_anchor`；
- 195 个 `bilinear_interpolation`；
- 共 225 个可解析 ABOVE pose；
- 空间关节只有 `000..004`；`005` 是气泵，禁止进入 correction field；
- Stage 7 加载前后及 session 操作时检查 SHA-256。

扁平编号是 `index = row * 15 + col`，canonical ID 使用三位格式：
`P000, P014, P112, P210, P224`。GUI 同时显示 `(row,col)`。

## 4. QUICK 5 算法

直接点：`(0,0), (0,14), (7,7), (14,0), (14,14)`。

先计算每个直接点相对 baseline 的 ΔPWM。四条边的中点 ΔPWM 由相邻角点
平均产生为显式 `VIRTUAL:Pxxx`，中心使用真实 direct ΔPWM。这样得到一个
3×3 correction node grid，再分成四个连续的 piecewise-bilinear 区域。

它严格通过四角和中心；虚拟点在 provenance 中不会伪装成 direct anchor。

## 5. STANDARD 9 算法

直接 3×3 节点：

```text
P000  P007  P014
P105  P112  P119
P210  P217  P224
```

节点行列均为 `0,7,14`。四个区域分别使用真实四角 direct ΔPWM 做双线性
插值。QUICK session 补齐 P007/P105/P119/P217 后自动原 session 升级为
`STANDARD_9`，不要求重开。

任意局部 anchor 使用所在标准 cell 内的 separable tent residual（分片双线性）
修正邻域，并在 cell 边界衰减为 0。最后再次强制覆盖所有 direct 点，保证
direct PWM 永远严格等于用户保存值。

## 6. ΔPWM 公式

对每个空间关节独立计算：

```text
delta_anchor[j] = new_anchor_pwm[j] - baseline_anchor_pwm[j]

delta(u,v)[j] =
    (1-u)(1-v) q11[j] + u(1-v) q12[j]
  + (1-u)v q21[j]     + uv q22[j]

candidate_pwm[j] = baseline_pwm[j] + round(delta(u,v)[j])
```

双线性核心由 Stage 5 和 Stage 7 共用，Stage 7 插值的是 correction field，
不是重新对机械臂求 IK，也不是重新插值 absolute pose。

## 7. GUI 操作

1. 展开 `Stage 7 · Rapid Calibration`。
2. `Load Baseline`，选择 QUICK 或 STANDARD，点 `New Calibration Session`。
3. 点击 15×15 grid 或输入 P index 选择点。
4. `Move ABOVE Current Point`；dry-run 只预览，live 使用 carry-high 安全路径。
5. 用 J0–J4 的 `-/+` 调整，默认 Step=5。
6. `Save As Direct Anchor`；该按钮本身不触发运动。
7. 必需 anchor 完成后点 `Recalculate 225`。
8. 任意点 Move ABOVE；正确则 `Verify Point`，偏差则 live tune 后保存为 local direct。
9. 再次 Recalculate，完成基本验证后 `Commit Calibration`。
10. `Rollback to Baseline` 会写当前 deployment 为 baseline 状态，但保留 session 审计。

快捷键：上下箭头为当前 Step；Ctrl+上下为 ±1；Shift+上下为 ±20；Enter
保存 anchor。

## 8. Live PWM 串口路径

```text
RapidCalibrationPanel.jog_requested
→ MainWindow.stage7_jog
→ RapidCalibrationCoordinator.queue_live_jog
→ 80 ms coalescing timer
→ SerialArmController.send_joint_pwm
→ SerialArmController.write (RLock)
```

示例：J2=1265 后 `+5` 只发送：

```text
#002P1270T0200!
```

不发送多关节 action，不携带 J0/J1/J3/J4，不访问泵 `005`。

## 9. Safety

- Stage 7 默认 dry-run；顶层 `--dry-run` 和 force gate 不能被页面静默关闭。
- 关闭 dry-run 必须二次确认，GUI 明确显示 `LIVE HARDWARE · COMx`。
- 未连接、dry-run、ESTOP、arm busy、未建 session、未先到当前 ABOVE 时只改 GUI，
  显示 `NOT SENT`。
- live jog 只允许 `000..004`，`005` 气泵和 `006/007` 被 controller API 拒绝。
- 人工标定范围使用既有 `550..2450`；超界 clamp，并保存 requested/applied。
- 同一关节快速点击只保留最新值；总 pending 上限为 5 个空间关节，每 80 ms
  最多发送一条。
- Move ABOVE 只允许从 `OBSERVE_IDLE`、泵关闭状态开始；路径为
  `CARRY_HIGH_LIFTED_IDLE → TARGET_ABOVE_IDLE`。
- 到达 ABOVE 后状态保持 `HOVERING`，必须通过
  `TARGET_ABOVE → CARRY_HIGH_LIFTED → OBSERVE_IDLE` 安全返回，不能直接横切。
- Save Anchor、Recalculate、Verify、Commit 都不触发下降、吸取、释放或落子。
- dry-run 结果不能标成实机 VERIFIED；Verify 需要真实连接和成功 live Move ABOVE。

## 10. Session JSON 示例

```json
{
  "schema_version": 1,
  "calibration_session_id": "2026-08-25_150000_quick_5",
  "baseline": {
    "path": "calibration/stage5_board_calibration.json",
    "sha256": "...",
    "resolved_point_count": 225,
    "direct_anchor_count": 30
  },
  "mode": "QUICK_5",
  "anchors": {
    "P112": {
      "point_id": "P112",
      "board_row": 7,
      "board_col": 7,
      "baseline_pwm": {"000": 1560, "001": 1170, "002": 990, "003": 1170, "004": 1500},
      "new_pwm": {"000": 1560, "001": 1175, "002": 989, "003": 1170, "004": 1500},
      "delta_pwm": {"000": 0, "001": 5, "002": -1, "003": 0, "004": 0},
      "source": "DIRECT",
      "timestamp": "2026-08-25T15:00:00+08:00",
      "calibration_session_id": "2026-08-25_150000_quick_5"
    }
  },
  "generated_points": {},
  "verified_points": [],
  "candidate_revision": 0,
  "candidate_stale": true,
  "created_at": "2026-08-25T15:00:00+08:00",
  "updated_at": "2026-08-25T15:00:00+08:00"
}
```

## 11. 测试结果

Stage 7 专项：`15 passed`。

覆盖：零 delta、常量 delta、双线性数学、direct 精确覆盖、provenance、clamp、
session round-trip、225 grid、编号映射、dry-run 不发送、mock 单关节发送、快速
点击合并以及 ABOVE→安全返回状态路径。

GUI offscreen smoke test：通过；无自动 camera/COM 连接，无真实串口发送。

全仓回归：`162 passed, 5 failed`（最终复跑值；5 个为 Stage 6 报告已记录的既有
baseline 不一致：P77_TOUCH_RELEASE 泵值、旧 pick/full-cycle 顺序、旧 carry action
名称期望）。Stage 7 没有修改这些稳定行为来迎合旧断言。

## 12. 尚未完成

- **HARDWARE NOT VERIFIED**：live 单关节协议只完成 mock 与 dry-run 验证。
- 未实现相机自动末端识别、AprilTag 机械臂自动标定、camera homography。
- 未实现 IK 重建、自动下降/落子标定、视觉闭环 joint correction。
- Stage 7 committed deployment 目前由 Stage 7 页面使用；尚未自动替换普通 Stage 5
  点击悬停或 Stage 6 descent 的 ABOVE source。
- GUI 当前创建新 session；后端支持 load existing session，但本轮没有做任意文件选择器。

## 13. 实机测试步骤

1. 先用 `--dry-run` 启动，创建 QUICK session，确认 +/- 只改 GUI、日志为 NOT SENT。
2. 机械臂可靠支撑，工作区清空，准备物理断电；确认真实固件兼容单舵机 PWM 协议。
3. 普通模式启动并连接 COM；先测试急停和恢复，再回 `OBSERVE_IDLE`，确认泵关闭。
4. 展开 Stage 7，创建 QUICK session；明确取消 DRY RUN，确认状态为 LIVE HARDWARE。
5. 首点只用 P112，点 Move ABOVE，观察 carry-high 路径；任何异常立即急停/断电。
6. 成功到 ABOVE 后只测试 J0 ±1，再 J1 ±1；确认每次只有对应关节微动。
7. 点“回观察位”，核对先回 carry-high 再 OBSERVE，禁止直接横切。
8. 再按 P000/P014/P210/P224 完成 QUICK；每点先 Move、微调、Save、再安全返回。
9. Recalculate 后只选一个近中心低风险点验证；不要直接测试边界或下降。
10. P137 局部修正流程完成后复验邻点，再 Commit；保存 session JSON 和完整日志。
11. 本轮仍不得测试 Stage 6 下降、吸取、释放或自动落子。
