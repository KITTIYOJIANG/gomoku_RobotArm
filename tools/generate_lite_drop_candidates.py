from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.arm.actions import ActionLibrary
from app.calibration_lite.drop_v1 import LiteDropStore
from app.calibration_lite.p77_point import load_p77_point
from app.integrated_v1.movel import MoveLPlanner
from app.integrated_v1.points import all_points
from app.stage6.kinematics import ArmKinematics, KinematicsConfig
from app.stage7.baseline import point_id as stage7_point_id


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate Lite Calibration V1 DROP candidates offline."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "calibration_lite.json",
    )
    return parser


def _load_saved_above() -> dict[tuple[int, int], dict[str, int]]:
    deployment_path = PROJECT_ROOT / "calibration" / "stage7_current_deployment.json"
    deployment = json.loads(deployment_path.read_text(encoding="utf-8"))
    session_path = Path(str(deployment.get("session_path") or ""))
    if not session_path.is_absolute():
        session_path = PROJECT_ROOT / session_path
    session = json.loads(session_path.read_text(encoding="utf-8"))
    points = session.get("generated_points") or {}
    if len(points) != 225:
        raise RuntimeError("committed Lite ABOVE deployment must contain 225 points")
    result: dict[tuple[int, int], dict[str, int]] = {}
    for point in all_points():
        record = points[stage7_point_id(point.row, point.col)]
        pwm = record.get("new_pwm") or record.get("final_above_pwm") or record.get("pwm")
        if not isinstance(pwm, dict):
            raise RuntimeError(f"missing saved ABOVE PWM for {point.point_id}")
        result[point.as_tuple()] = {f"{int(key):03d}": int(value) for key, value in pwm.items()}
    p77 = load_p77_point(PROJECT_ROOT / "calibration" / "p77.json")
    if p77 is not None:
        result[(7, 7)] = p77
    return result


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    settings = json.loads(args.config.read_text(encoding="utf-8"))
    drop = settings["drop_v1"]
    if drop.get("auto_hardware_batch", False):
        raise RuntimeError("auto_hardware_batch must remain false")
    library = ActionLibrary()
    store = LiteDropStore(PROJECT_ROOT / drop["profile_path"], library=library)
    store.load_or_initialize(
        _load_saved_above(),
        source={
            "above_policy": "deployment_session_plus_p77_and_five_golden_overlays",
            "deployment": "calibration/stage7_current_deployment.json",
            "session": "deployment.session_path",
            "p77": "calibration/p77.json",
            "execution": "OFFLINE_ONLY_NO_CONTROLLER_IMPORT",
        },
    )
    planner = MoveLPlanner(
        store,
        kinematics=ArmKinematics(
            KinematicsConfig.load(PROJECT_ROOT / drop["kinematics_path"])
        ),
        target_descent_mm=float(drop["target_descent_mm"]),
        step_mm=float(drop["waypoint_step_mm"]),
        max_waypoint_joint_delta_pwm=int(drop["max_waypoint_joint_delta_pwm"]),
    )
    summary = planner.generate_all(persist=True)
    print(json.dumps(summary.to_dict(), ensure_ascii=False, sort_keys=True))
    print(f"profile={store.path}")
    print("hardware_execution=NO")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
