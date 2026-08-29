# Hardware Setup

This document records only hardware facts present in code, configuration, protocol notes, or calibration data. Exact mechanical mounting dimensions and the deployed firmware revision are not stored in the repository.

## Required Hardware

- J1 robot arm using host-addressed servo IDs `000..007`.
- STM32-compatible controller running the existing ASCII PWM protocol.
- Windows PC with a USB serial connection.
- USB camera; preferred configured name `USB 2.0 Camera`.
- 15×15 Gomoku board with AprilTags 15, 16, 17 and 18 visible around the playable area.
- Stable robot/board/camera fixtures and an immediately reachable physical power cutoff.

No committed deployment photograph is available. The `logs/tour_frames` images in the audited working tree are untracked runtime frames, not a durable setup reference.

## Robot Connection

```text
Windows PC
  -> USB serial / selectable COM port
  -> deployed STM32-compatible controller
  -> servo bus
  -> J1 servos and suction channel
```

Do not assume the controller currently attached to the configured port runs the copied `firmware/stm32_j1_reference` image. The reference build enables `SAFE_STAGE1`, rejects raw multi-servo PWM, and is not a drop-in replacement for the deployed controller.

## Serial Configuration

| Setting | Repository value |
| --- | --- |
| Default port hint | `COM6` |
| Port selection | GUI dropdown; COM numbers may change |
| Baud rate | `115200` (fixed by host validation) |
| Encoding | ASCII |
| Read timeout | `0.1 s` in `SerialArmController.connect()` |
| Write timeout | `1.0 s` from `config/app_config.json` |
| Auto-connect | No |
| Emergency stop | `$DST!` |
| Pump off | `#005P1500T0500!` |

A multi-servo target is formatted as `{#000P....!#001P....!...}`. The order inside braces is a synchronized target, not a movement sequence. Ordered motion is sent as separate commands by `ArmSequenceWorker`, with waits derived from command `T` plus a configured margin. There is no reliable general arrival ACK.

## Servo IDs

| ID | Host role | Notes |
| --- | --- | --- |
| `000` | Base/spatial axis | Included in stable action PWM and Stage 5/6 calibration. |
| `001` | Shoulder/spatial axis | Raised in runtime carry-high actions. |
| `002` | Elbow/spatial axis | Included in candidate planar kinematics. |
| `003` | Upper arm/wrist spatial axis | Included in candidate planar kinematics. |
| `004` | Auxiliary wrist/tool axis | Preserved by Stage 6; exact physical axis is not proven in repository evidence. |
| `005` | Suction | 2500=hold/vacuum, 1500=off/release under the host convention. |
| `006`, `007` | No established host role | Stable actions hold both at 1500. |

The copied `SAFE_STAGE1` reference documents only IDs `000..005` for its guarded bring-up path. That does not supersede the deployed host action table, which contains `000..007` exactly once per action.

## Camera Setup

Repository configuration:

- Preferred device name: `USB 2.0 Camera`.
- Numeric fallback index: `0`; override with environment variable `GOMOKU_CAMERA_ID`.
- Requested resolution/rate: `1280×720 @ 30 FPS`.
- AprilTag detection: every 4 frames by default.
- Minimum tags for localization: 3.
- Camera intrinsics: no real intrinsics file is committed; the example explicitly says missing.
- Localization: normalized 2D homography, not metric camera pose.

Mount the camera so the board and at least three configured tags remain visible over the robot's working cycle. The repository does not establish whether the camera is overhead, its rotation, distance, or fixed transform. Before any motion, connect only the camera and verify the overlay orientation against the board.

## Board Orientation

The configured image/board orientation is:

```text
Tag 15 / TL                         Tag 16 / TR
P000 = (row 0, col 0)        P014 = (row 0, col 14)

                 legacy P77 = (7,7) = flat P112

P210 = (row 14, col 0)      P224 = (row 14, col 14)
Tag 18 / BL                         Tag 17 / BR
```

Rows grow top-to-bottom; columns grow left-to-right. Confirm this overlay physically; do not rotate or mirror the mapping merely to make a click appear plausible.

## Physical Deployment Checklist

- [ ] Robot base fixed and arm workspace clear.
- [ ] Board fixed in the verified orientation.
- [ ] Camera fixed; tags and the whole playable area visible.
- [ ] USB/serial cable connected to the intended controller.
- [ ] Controller/servo power understood and physical cutoff ready.
- [ ] Start command contains `--dry-run`; Stage 5 dry-run checkbox is visibly checked.
- [ ] Camera-only board orientation verified before connecting serial.
- [ ] Exact COM port and deployed firmware compatibility confirmed.
- [ ] For a later authorized hardware test: no piece/load, one point, one cycle, operator ready to cut power.
