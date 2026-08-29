from __future__ import annotations

import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
REQUIRED_DISTRIBUTIONS = (
    "opencv-python",
    "numpy",
    "PySide6",
    "pyserial",
    "pupil-apriltags",
    "pytest",
)


def emit(text: str = "") -> None:
    line = str(text) + "\n"
    try:
        sys.stdout.write(line)
    except UnicodeEncodeError:
        stream = getattr(sys.stdout, "buffer", None)
        if stream is not None:
            stream.write(line.encode(getattr(sys.stdout, "encoding", None) or "utf-8", "replace"))


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("top-level JSON value is not an object")
    return value


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def git(*args: str) -> tuple[int, str]:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            check=False,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)
    return result.returncode, (result.stdout or result.stderr).strip()


class Doctor:
    def __init__(self) -> None:
        self.critical = 0
        self.warnings = 0

    def line(self, label: str, status: str, detail: str = "") -> None:
        if status == "FAIL":
            self.critical += 1
        elif status == "WARN":
            self.warnings += 1
        suffix = f" — {detail}" if detail else ""
        emit(f"{label:<28} {status}{suffix}")

    def run(self) -> int:
        emit("=" * 64)
        emit("GOMOKU ROBOT PROJECT DOCTOR")
        emit("NON-DESTRUCTIVE: no camera open, serial open, or motion command")
        emit("=" * 64)

        state = self.repository_checks()
        self.environment_checks()
        self.calibration_checks(state)
        self.lite_v1_checks()
        self.integrated_v1_checks()
        self.safety_checks(state)
        self.vision_serial_checks()

        emit()
        if state:
            emit(f"Current Stage ............. {state.get('current_stage', 'UNKNOWN')}")
            emit(f"Status .................... {state.get('status', 'UNKNOWN')}")
            emit("NEXT TASK:")
            emit(str(state.get("next_task", "UNKNOWN")))
        emit()
        if self.critical:
            emit(f"Overall: NOT READY — {self.critical} critical problem(s), {self.warnings} warning(s)")
            return 1
        emit(f"Overall: READY FOR SAFE DEVELOPMENT — {self.warnings} warning(s)")
        return 0

    def repository_checks(self) -> dict[str, Any] | None:
        emit("\nRepository")
        code, root = git("rev-parse", "--show-toplevel")
        expected = str(ROOT.resolve()).casefold()
        if code == 0 and str(Path(root).resolve()).casefold() == expected:
            self.line("Git repository", "PASS", root)
        else:
            self.line("Git repository", "FAIL", root or "not found")

        for relative in ("START_HERE.md", "project_state.json", "docs/ARCHITECTURE.md"):
            path = ROOT / relative
            self.line(relative, "PASS" if path.is_file() else "FAIL")

        state_path = ROOT / "project_state.json"
        try:
            state = load_json(state_path)
            for key in ("project", "current_stage", "status", "last_stable", "next_task"):
                if key not in state:
                    raise ValueError(f"missing key {key}")
            self.line("Project state JSON", "PASS", f"updated {state.get('last_updated', 'UNKNOWN')}")
        except Exception as exc:
            self.line("Project state JSON", "FAIL", str(exc))
            state = None

        code, branch = git("branch", "--show-current")
        self.line("Current branch", "INFO" if code == 0 else "WARN", branch or "UNKNOWN")
        code, status = git("status", "--porcelain=v1", "--untracked-files=all")
        if code != 0:
            self.line("Working tree", "WARN", "git status failed")
        elif status:
            count = len(status.splitlines())
            self.line("Working tree", "WARN", f"dirty ({count} entries); preserve existing work")
        else:
            self.line("Working tree", "PASS", "clean")
        return state

    def environment_checks(self) -> None:
        emit("\nEnvironment")
        version = sys.version_info
        detail = f"{version.major}.{version.minor}.{version.micro} at {Path(sys.executable)}"
        supported = (version.major, version.minor) in {(3, 10), (3, 11), (3, 12)}
        self.line("Python", "PASS" if supported else "WARN", detail)

        missing: list[str] = []
        versions: list[str] = []
        for name in REQUIRED_DISTRIBUTIONS:
            try:
                versions.append(f"{name}={importlib.metadata.version(name)}")
            except importlib.metadata.PackageNotFoundError:
                missing.append(name)
        if missing:
            self.line("Required packages", "WARN", "missing: " + ", ".join(missing))
        else:
            self.line("Required packages", "PASS", ", ".join(versions))

    def calibration_checks(self, state: dict[str, Any] | None) -> None:
        emit("\nCalibration")
        baseline_info = (state or {}).get("baseline_calibration") or {}
        baseline_path = ROOT / str(baseline_info.get("path", "calibration/stage5_board_calibration.json"))
        try:
            baseline = load_json(baseline_path)
            self.line("Baseline file", "PASS", str(baseline_path.relative_to(ROOT)))
        except Exception as exc:
            self.line("Baseline file", "FAIL", str(exc))
            return

        expected_hash = str(baseline_info.get("sha256", "UNKNOWN")).upper()
        actual_hash = sha256(baseline_path)
        self.line(
            "Baseline SHA-256",
            "PASS" if expected_hash == actual_hash else "FAIL",
            actual_hash,
        )

        board_size = int(baseline.get("board_size", 0))
        anchors = baseline.get("anchors")
        expected_anchors = int(baseline_info.get("direct_anchor_count", 30))
        valid_anchors = isinstance(anchors, dict) and len(anchors) == expected_anchors
        self.line(
            "Direct anchors",
            "PASS" if valid_anchors else "FAIL",
            f"{len(anchors) if isinstance(anchors, dict) else 0}; board {board_size}x{board_size}",
        )

        pwm_errors: list[str] = []
        if isinstance(anchors, dict):
            for key, record in anchors.items():
                pwm = record.get("pwm") if isinstance(record, dict) else None
                if not isinstance(pwm, dict) or set(pwm) != {"000", "001", "002", "003", "004"}:
                    pwm_errors.append(f"{key}: keys")
                    continue
                if any(type(value) is not int or not 500 <= value <= 2500 for value in pwm.values()):
                    pwm_errors.append(f"{key}: values")
        self.line(
            "Baseline PWM parse",
            "PASS" if not pwm_errors else "FAIL",
            "30 x joints 000..004" if not pwm_errors else ", ".join(pwm_errors[:5]),
        )

        stage6_path = ROOT / "calibration" / "stage6_descent_calibration.json"
        try:
            stage6 = load_json(stage6_path)
            snapshots = stage6.get("above_fk_snapshot") or {}
            profiles = stage6.get("profiles") or {}
            batch = stage6.get("last_batch_generation") or {}
            rejected = batch.get("rejected") or {}
            counts_ok = len(snapshots) == 225 and len(profiles) == 153 and len(rejected) == 72
            self.line(
                "225-point provenance",
                "PASS" if counts_ok else "WARN",
                f"snapshot={len(snapshots)}, profiles={len(profiles)}, rejected={len(rejected)}",
            )
            source_hash = str((stage6.get("source_above") or {}).get("sha256", "")).upper()
            self.line(
                "Stage 6 source hash",
                "PASS" if source_hash == actual_hash else "FAIL",
                source_hash or "missing",
            )
        except Exception as exc:
            self.line("Stage 6 calibration", "WARN", str(exc))

        action_info = (state or {}).get("stable_action_table") or {}
        action_path = ROOT / str(action_info.get("path", "config/arm_actions.json"))
        try:
            actions = load_json(action_path)
            action_count = len(actions.get("actions") or {})
            hash_ok = sha256(action_path) == str(action_info.get("sha256", "")).upper()
            self.line(
                "Stable action table",
                "PASS" if action_count == 11 and hash_ok else "FAIL",
                f"actions={action_count}, sha256={sha256(action_path)}",
            )
        except Exception as exc:
            self.line("Stable action table", "FAIL", str(exc))

    def safety_checks(self, state: dict[str, Any] | None) -> None:
        emit("\nSafety")
        try:
            app_config = load_json(ROOT / "config" / "app_config.json")
            stage5 = app_config.get("stage5") or {}
            default = bool(stage5.get("default_dry_run", True))
            forced = bool(stage5.get("force_dry_run", False))
            self.line(
                "Stage 5 dry-run",
                "PASS" if default else "WARN",
                f"default={default}, forced={forced}",
            )
        except Exception as exc:
            self.line("Stage 5 dry-run", "FAIL", str(exc))

        try:
            stage6 = load_json(ROOT / "config" / "stage6_descent.json")
            forced = bool(stage6.get("force_dry_run", False))
            self.line("Stage 6 force dry-run", "PASS" if forced else "WARN", str(forced))
        except Exception as exc:
            self.line("Stage 6 force dry-run", "FAIL", str(exc))

        stage7_path = ROOT / "config" / "stage7_rapid_calibration.json"
        if stage7_path.exists():
            try:
                stage7 = load_json(stage7_path)
                default = bool(stage7.get("default_dry_run", True))
                forced = bool(stage7.get("force_dry_run", False))
                self.line(
                    "Stage 7 operator path",
                    "PASS" if not default and not forced else "WARN",
                    f"live_default={not default and not forced}; explicit COM connection required",
                )
            except Exception as exc:
                self.line("Stage 7 operator path", "WARN", str(exc))

        project_forced = bool(((state or {}).get("safe_defaults") or {}).get("project_wide_force_dry_run", False))
        self.line(
            "Project-wide force lock",
            "PASS" if project_forced else "WARN",
            "not globally forced; operator GUI is live, keep COM disconnected until checks pass",
        )

    def lite_v1_checks(self) -> None:
        emit("\nLite Calibration V1")
        try:
            from app.calibration_lite.drop_v1 import LiteDropStore
            from app.integrated_v1.golden import SPATIAL_KEYS, assert_golden_above

            settings = load_json(ROOT / "config" / "calibration_lite.json")
            drop = settings.get("drop_v1") or {}
            no_batch_motion = drop.get("auto_hardware_batch") is False
            self.line(
                "Lite hardware batch gate",
                "PASS" if no_batch_motion else "FAIL",
                "auto_hardware_batch=false",
            )
            kinematics = load_json(ROOT / str(drop["kinematics_path"]))
            j5_locked = int(kinematics.get("pump_joint_id", -1)) == 5
            spatial_only = set(kinematics.get("joints") or {}) == set(SPATIAL_KEYS)
            self.line(
                "Lite J5 kinematics lock",
                "PASS" if j5_locked and spatial_only else "FAIL",
                "pump=005; FK/IK joints=000..004",
            )
            profile_path = ROOT / str(drop["profile_path"])
            store = LiteDropStore(profile_path)
            data = store.load()
            assert_golden_above(data["above"]["points"])
            records = (data.get("drop") or {}).get("points") or {}
            stats = store.statistics()
            verification_levels = {
                str(record.get("verification_level", "NOT VERIFIED"))
                for record in records.values()
            }
            valid_levels = verification_levels <= {
                "NOT VERIFIED",
                "OFFLINE VERIFIED",
                "HARDWARE VERIFIED",
            }
            no_j5_pose = all(
                "005" not in (record.get(field) or {})
                for record in records.values()
                for field in ("above_pwm", "drop_auto_pwm", "drop_final_pwm")
            )
            ok = len(records) == 225 and valid_levels and no_j5_pose
            self.line(
                "Lite DROP profile",
                "PASS" if ok else "FAIL",
                f"generated={stats['Generated']}, unreachable={stats['Unreachable']}, "
                f"verified={stats['Verified']}",
            )
        except Exception as exc:
            self.line("Lite Calibration V1", "FAIL", str(exc))

    def integrated_v1_checks(self) -> None:
        emit("\nIntegrated V1")
        try:
            from app.integrated_v1.golden import GOLDEN_ABOVE, assert_golden_above
            from app.integrated_v1.profile import CalibrationProfileManager

            expected = {
                "P33": [1589, 1136, 1101, 1084, 1500],
                "P311": [1432, 1199, 1157, 1042, 1500],
                "P77": [1500, 1230, 870, 1230, 1500],
                "P113": [1630, 1264, 588, 1424, 1500],
                "P1111": [1382, 1258, 639, 1410, 1500],
            }
            actual = {anchor.legacy_id: list(anchor.pwm) for anchor in GOLDEN_ABOVE.values()}
            self.line("Golden ABOVE constants", "PASS" if actual == expected else "FAIL", f"{len(actual)} protected anchors")

            settings = load_json(ROOT / "config" / "integrated_v1.json")
            profile_path = ROOT / str(settings["profile_path"])
            if not profile_path.exists():
                self.line("V1 startup route", "INFO", "FIRST_SETUP; no saved V1 profile")
            else:
                manager = CalibrationProfileManager(profile_path=profile_path)
                manager.load()
                assert_golden_above(manager.data["above"]["points"])
                status = manager.status()
                self.line("V1 profile integrity", "PASS", f"route={status.route}; valid={status.valid}")

            safety = settings.get("safety") or {}
            no_batch_motion = safety.get("generate_all_hardware_execution") is False
            protected = safety.get("protect_golden_above") is True
            self.line(
                "V1 batch/Golden guards",
                "PASS" if no_batch_motion and protected else "FAIL",
                f"batch_hardware={not no_batch_motion}; protect_golden={protected}",
            )
        except Exception as exc:
            self.line("Integrated V1", "FAIL", str(exc))

    def vision_serial_checks(self) -> None:
        emit("\nVision / Serial")
        try:
            app_config = load_json(ROOT / "config" / "app_config.json")
            camera = app_config.get("camera") or {}
            self.line(
                "Camera config",
                "PASS",
                f"{camera.get('preferred_name', 'UNKNOWN')} / index {camera.get('index', 'UNKNOWN')} / "
                f"{camera.get('width', '?')}x{camera.get('height', '?')}@{camera.get('fps', '?')}",
            )
            serial = app_config.get("serial") or {}
            self.line(
                "Serial config",
                "INFO",
                f"port hint={serial.get('default_port', 'UNKNOWN')}, baud={serial.get('baudrate', 'UNKNOWN')}; not opened",
            )
        except Exception as exc:
            self.line("App configuration", "FAIL", str(exc))

        real_intrinsics = ROOT / "config" / "camera_intrinsics.json"
        fallback = ROOT / "config" / "camera_intrinsics.example.json"
        if real_intrinsics.is_file():
            self.line("Camera intrinsics", "PASS", str(real_intrinsics.relative_to(ROOT)))
        elif fallback.is_file():
            self.line("Camera intrinsics", "WARN", "real file missing; 2D homography fallback only")
        else:
            self.line("Camera intrinsics", "WARN", "no real or example file")


def main() -> int:
    return Doctor().run()


if __name__ == "__main__":
    raise SystemExit(main())
