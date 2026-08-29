from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping

from app.arm.actions import Action, ActionLibrary, ServoTarget
from app.arm.ordered_motion import (
    BOARD_SAFE_RETURN_PHASE_TIME_MS,
    j1_first_sequence,
    j1_last_sequence,
)
from app.arm.sequences import ActionStep, SequenceDefinition, WaitStep, pick_piece
from app.integrated_v1.golden import (
    GOLDEN_ABOVE,
    SPATIAL_KEYS,
    assert_golden_above,
    golden_for,
    normalize_spatial,
)
from app.integrated_v1.movel import DropStatus
from app.integrated_v1.points import BOARD_SIZE, PointRef, all_points, parse_point_id
from app.integrated_v1.profile import ProfileError
from app.stage6.kinematics import KinematicsConfig


PRODUCT = "J1 Gomoku Robot Lite Calibration V1"

# Runtime-only DROP calibration staging lift for J1.  The SAFE ABOVE pose is
# (J0/J2/J3/J4 = exact ABOVE, J1 = exact ABOVE J1 + 30) and is never persisted.
SAFE_ABOVE_J1_LIFT_PWM = 30


def _now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


class LiteDropStore:
    """Independent Lite DROP store; ABOVE is read-only after synchronization.

    Only J0..J4 are accepted. J5 never enters the file's kinematic pose fields.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        library: ActionLibrary | None = None,
        joint_limits: Mapping[int | str, tuple[int, int]] | None = None,
    ) -> None:
        self.path = Path(path)
        self.library = library or ActionLibrary()
        if joint_limits is None:
            configured = KinematicsConfig.load().joints
            self.joint_limits = {
                f"{joint:03d}": (int(item.pwm_min), int(item.pwm_max))
                for joint, item in configured.items()
            }
        else:
            self.joint_limits = {
                f"{int(joint):03d}": (int(bounds[0]), int(bounds[1]))
                for joint, bounds in joint_limits.items()
            }
        if set(self.joint_limits) != set(SPATIAL_KEYS):
            raise ProfileError("Lite DROP joint limits must contain J0..J4 only")
        self.data: dict[str, Any] | None = None

    def load(self) -> dict[str, Any]:
        raw = json.loads(self.path.read_text(encoding="utf-8"))
        if not isinstance(raw, dict) or raw.get("product") != PRODUCT:
            raise ProfileError("not a Lite Calibration V1 DROP file")
        if int(raw.get("schema_version", 0)) != 1:
            raise ProfileError("unsupported Lite DROP schema")
        points = (raw.get("above") or {}).get("points") or {}
        if len(points) != BOARD_SIZE * BOARD_SIZE:
            raise ProfileError("Lite DROP file must contain 225 ABOVE points")
        assert_golden_above(points)
        self.data = raw
        return raw

    def load_or_initialize(
        self,
        above_by_coord: Mapping[tuple[int, int], Mapping[str | int, int]],
        *,
        source: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self.path.is_file():
            self.load()
        else:
            now = _now()
            self.data = {
                "schema_version": 1,
                "product": PRODUCT,
                "created_at": now,
                "updated_at": now,
                "source": dict(source or {}),
                "safety": {
                    "j5_pump_joint": "005",
                    "j5_in_kinematics": False,
                    "auto_hardware_batch": False,
                    "golden_above_mutable": False,
                },
                "above": {"points": {}},
                "drop": {"points": {}, "last_generate_all": None},
                "verification": {"verified_drop_points": []},
                "history": [],
            }
        self.sync_above(above_by_coord, source=source)
        return self._require_data()

    def sync_above(
        self,
        above_by_coord: Mapping[tuple[int, int], Mapping[str | int, int]],
        *,
        source: Mapping[str, Any] | None = None,
    ) -> int:
        data = self._require_data()
        if len(above_by_coord) != BOARD_SIZE * BOARD_SIZE:
            raise ProfileError("Lite ABOVE snapshot must contain exactly 225 points")
        previous = (data.get("above") or {}).get("points") or {}
        updated: dict[str, dict[str, Any]] = {}
        changed = 0
        for point in all_points():
            try:
                pwm = self._validate_pwm(above_by_coord[point.as_tuple()])
            except KeyError as exc:
                raise ProfileError(f"missing ABOVE {point.point_id}") from exc
            golden = golden_for(point.row, point.col)
            if golden is not None:
                pwm = golden.pwm_map
            record = {
                "point_id": point.point_id,
                "board": [point.row, point.col],
                "final_above_pwm": pwm,
                "source": "golden_direct_anchor" if golden else "lite_saved_above",
                "protected": golden is not None,
                "verified": golden is not None,
                "verification_level": (
                    "HARDWARE VERIFIED" if golden else "NOT VERIFIED"
                ),
            }
            old = previous.get(point.point_id) or {}
            if old.get("final_above_pwm") != pwm:
                changed += 1
                self._invalidate_drop(point, reason="ABOVE_SOURCE_CHANGED")
            updated[point.point_id] = record
        data["above"] = {"points": updated}
        if source is not None:
            data["source"] = dict(source)
        data["updated_at"] = _now()
        data["history"].append(
            {
                "timestamp": _now(),
                "event": "LITE_ABOVE_SYNCHRONIZED",
                "changed_points": changed,
                "golden_preserved": len(GOLDEN_ABOVE),
            }
        )
        assert_golden_above(updated)
        return changed

    def save(self) -> Path:
        data = self._require_data()
        assert_golden_above(data["above"]["points"])
        data["updated_at"] = _now()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.path)
        return self.path

    def above_record(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        return dict(self._require_data()["above"]["points"][point.point_id])

    def above_pwm(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> dict[str, int]:
        return self._validate_pwm(self.above_record(point_id)["final_above_pwm"])

    def drop_record(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> dict[str, Any] | None:
        point = parse_point_id(point_id)
        record = self._require_data()["drop"]["points"].get(point.point_id)
        return None if record is None else record

    def set_drop_record(
        self,
        point_id: str | PointRef | tuple[int, int],
        record: Mapping[str, Any],
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        stored = dict(record)
        if stored.get("above_pwm") is not None:
            stored["above_pwm"] = self._validate_pwm(stored["above_pwm"])
        for field in ("drop_auto_pwm", "drop_final_pwm"):
            if stored.get(field) is not None:
                stored[field] = self._validate_pwm(stored[field])
        correction = stored.get("drop_correction_pwm")
        if correction is not None:
            stored["drop_correction_pwm"] = self._validate_correction(correction)
        for waypoint in stored.get("waypoints") or []:
            waypoint["pwm"] = self._validate_pwm(waypoint["pwm"])
        stored["point_id"] = point.point_id
        stored["board"] = [point.row, point.col]
        self._require_data()["drop"]["points"][point.point_id] = stored
        return stored

    def save_correction(
        self,
        point_id: str | PointRef | tuple[int, int],
        correction: Mapping[str | int, int],
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        record = self.drop_record(point)
        if record is None or record.get("drop_auto_pwm") is None:
            raise ProfileError(f"generate DROP before correcting {point.point_id}")
        delta = self._validate_correction(correction)
        auto = self._validate_pwm(record["drop_auto_pwm"])
        final = self._validate_pwm(
            {joint: auto[joint] + delta[joint] for joint in SPATIAL_KEYS}
        )
        record["drop_correction_pwm"] = delta
        record["drop_final_pwm"] = final
        record["status"] = (
            DropStatus.MANUAL_CORRECTED.value
            if any(delta.values())
            else DropStatus.PENDING_VERIFY.value
        )
        record["verified"] = False
        record["verification_level"] = "NOT VERIFIED"
        record["verification_time"] = None
        ids = self._verified_ids()
        ids.discard(point.point_id)
        self._write_verified_ids(ids)
        self._require_data()["history"].append(
            {
                "timestamp": _now(),
                "event": "LITE_DROP_CORRECTION_SAVED",
                "point_id": point.point_id,
                "correction": delta,
            }
        )
        return record

    def apply_p77_manual_movel_calibration(
        self,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Apply the confirmed P77 manual endpoint as the MoveL correction overlay."""
        if payload.get("point_id") != "P77" or payload.get("board") != [7, 7]:
            raise ProfileError("manual MoveL calibration must remain scoped to P77")
        if payload.get("source") != "manual_movel_tuning":
            raise ProfileError("P77 manual MoveL calibration source is invalid")
        if not bool(payload.get("operator_confirmed", False)):
            raise ProfileError("P77 manual MoveL calibration requires operator confirmation")
        if bool(payload.get("hardware_verified", False)):
            raise ProfileError("P77 manual MoveL calibration cannot self-assert hardware verification")

        steps = list(payload.get("steps") or [])
        drop_index = int(payload.get("drop_step_index", -1))
        if drop_index <= 0 or len(steps) != drop_index + 1:
            raise ProfileError("P77 manual MoveL steps must be contiguous through DROP")
        normalized_steps: list[dict[str, Any]] = []
        for index, step in enumerate(steps):
            if int(step.get("step_index", -1)) != index:
                raise ProfileError("P77 manual MoveL step indices must be contiguous")
            if not bool(step.get("operator_confirmed", False)):
                raise ProfileError(f"P77 manual MoveL Step{index} is not confirmed")
            if bool(step.get("hardware_verified", False)):
                raise ProfileError("manual MoveL steps cannot self-assert hardware verification")
            normalized_steps.append(
                {
                    "step_index": index,
                    "final_pwm": self._validate_pwm(
                        {
                            f"{joint:03d}": step["final_pwm"][f"J{joint}"]
                            for joint in range(5)
                        }
                    ),
                    "operator_confirmed": True,
                    "hardware_verified": False,
                }
            )
        if normalized_steps[0]["final_pwm"] != self.above_pwm("P77"):
            raise ProfileError("P77 manual MoveL Step0 must equal Golden ABOVE")
        final = self._validate_pwm(
            {
                f"{joint:03d}": payload["final_pwm"][f"J{joint}"]
                for joint in range(5)
            }
        )
        if normalized_steps[-1]["final_pwm"] != final:
            raise ProfileError("P77 manual MoveL DROP must match its final saved step")

        record = self.drop_record("P77")
        if record is None or record.get("drop_auto_pwm") is None:
            raise ProfileError("generate the P77 MoveL candidate before applying manual calibration")
        auto = self._validate_pwm(record["drop_auto_pwm"])
        correction = {
            joint: int(final[joint]) - int(auto[joint]) for joint in SPATIAL_KEYS
        }
        record = self.save_correction("P77", correction)
        record["manual_movel_calibration"] = {
            "source": "manual_movel_tuning",
            "drop_step_index": drop_index,
            "final_pwm": final,
            "steps": normalized_steps,
            "operator_confirmed": True,
            "hardware_verified": False,
            "saved_at": str(payload.get("saved_at") or _now()),
        }
        self._require_data()["history"].append(
            {
                "timestamp": _now(),
                "event": "P77_MANUAL_MOVEL_APPLIED",
                "point_id": "P77",
                "drop_step_index": drop_index,
                "final_pwm": final,
                "hardware_verified": False,
            }
        )
        return record

    def mark_verified(
        self,
        point_id: str | PointRef | tuple[int, int],
        *,
        hardware_confirmed: bool,
        notes: str = "",
    ) -> dict[str, Any]:
        point = parse_point_id(point_id)
        record = self.drop_record(point)
        if record is None or record.get("status") in {
            DropStatus.NOT_GENERATED.value,
            DropStatus.MOVE_L_UNREACHABLE.value,
            DropStatus.INVALID.value,
        }:
            raise ProfileError(f"{point.point_id} has no executable DROP candidate")
        level = "HARDWARE VERIFIED" if hardware_confirmed else "OFFLINE VERIFIED"
        record["status"] = DropStatus.VERIFIED.value
        record["verified"] = True
        record["verification_level"] = level
        record["verification_time"] = _now()
        record["notes"] = str(notes)
        ids = self._verified_ids()
        ids.add(point.point_id)
        self._write_verified_ids(ids)
        self._require_data()["history"].append(
            {
                "timestamp": _now(),
                "event": "LITE_DROP_VERIFIED",
                "point_id": point.point_id,
                "verification_level": level,
                "notes": str(notes),
            }
        )
        return record

    def statistics(self) -> dict[str, int]:
        records = self._require_data()["drop"]["points"].values()
        counts = {status.value: 0 for status in DropStatus}
        for record in records:
            key = str(record.get("status", DropStatus.NOT_GENERATED.value))
            counts[key] = counts.get(key, 0) + 1
        return {
            "Total": BOARD_SIZE * BOARD_SIZE,
            "Generated": sum(
                value
                for key, value in counts.items()
                if key not in {
                    DropStatus.NOT_GENERATED.value,
                    DropStatus.MOVE_L_UNREACHABLE.value,
                    DropStatus.INVALID.value,
                }
            ),
            "Verified": counts.get(DropStatus.VERIFIED.value, 0),
            "Unreachable": counts.get(DropStatus.MOVE_L_UNREACHABLE.value, 0),
            "Invalid": counts.get(DropStatus.INVALID.value, 0),
        }

    def _validate_pwm(self, pwm: Mapping[str | int, int]) -> dict[str, int]:
        values = normalize_spatial(pwm)
        for joint, value in values.items():
            lower, upper = self.joint_limits[joint]
            if not lower <= value <= upper:
                raise ProfileError(
                    f"joint {joint} PWM {value} outside {lower}..{upper}"
                )
        return values

    def joint_bounds(self, joint: int | str) -> tuple[int, int]:
        key = f"{int(joint):03d}"
        try:
            return self.joint_limits[key]
        except KeyError as exc:
            raise ProfileError("only J0..J4 have motion limits") from exc

    @staticmethod
    def _validate_correction(
        correction: Mapping[str | int, int]
    ) -> dict[str, int]:
        normalized = {f"{int(key):03d}": int(value) for key, value in correction.items()}
        missing = [joint for joint in SPATIAL_KEYS if joint not in normalized]
        if missing:
            raise ProfileError(f"missing DROP correction joints: {missing}")
        return {joint: normalized[joint] for joint in SPATIAL_KEYS}

    def _invalidate_drop(self, point: PointRef, *, reason: str) -> None:
        data = self._require_data()
        record = (data.get("drop") or {}).get("points", {}).get(point.point_id)
        if record is None:
            return
        record["status"] = DropStatus.NOT_GENERATED.value
        record["verified"] = False
        record["verification_level"] = "NOT VERIFIED"
        record["stale_reason"] = reason
        ids = self._verified_ids()
        ids.discard(point.point_id)
        self._write_verified_ids(ids)

    def _verified_ids(self) -> set[str]:
        return set(self._require_data()["verification"].get("verified_drop_points") or [])

    def _write_verified_ids(self, ids: set[str] | None = None) -> None:
        if ids is None:
            ids = self._verified_ids()
        self._require_data()["verification"]["verified_drop_points"] = sorted(ids)

    def _require_data(self) -> dict[str, Any]:
        if self.data is None:
            raise ProfileError("initialize the Lite DROP store first")
        return self.data


class LiteDropSequenceBuilder:
    """Build guarded Lite sequences; owns no serial port or worker."""

    def __init__(
        self,
        *,
        actions: ActionLibrary,
        store: LiteDropStore,
        move_time_ms: int = 1000,
        vacuum_build_ms: int = 700,
        release_ms: int = 700,
        pump_hold_pwm: int = 2500,
        pump_off_pwm: int = 1500,
    ) -> None:
        self.actions = actions
        self.store = store
        self.move_time_ms = int(move_time_ms)
        self.vacuum_build_ms = int(vacuum_build_ms)
        self.release_ms = int(release_ms)
        self.pump_hold_pwm = int(pump_hold_pwm)
        self.pump_off_pwm = int(pump_off_pwm)

    def build_move_above(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> SequenceDefinition:
        return self.move_observation_to_above(point_id, pump_state="OFF")

    def move_observation_to_above(
        self,
        point_id: str | PointRef | tuple[int, int],
        pump_state: str,
    ) -> SequenceDefinition:
        """Move from the saved OBSERVE pose to ABOVE.

        Rule 1: the coarse transit holds J1 at the OBSERVE value while
        J0/J2/J3/J4 reach ABOVE first (J1 LAST).  Rule 3: a transient SAFE
        ABOVE staging pose (J1 = exact ABOVE J1 + 30) is inserted before the
        exact ABOVE.  The +30 pose is runtime-only and is never saved.
        """

        point = parse_point_id(point_id)
        normalized_pump_state = str(pump_state).strip().upper()
        if normalized_pump_state == "OFF":
            hold = False
            pump_pwm = self.pump_off_pwm
            observe_name = "OBSERVE_IDLE"
        elif normalized_pump_state == "HOLD":
            hold = True
            pump_pwm = self.pump_hold_pwm
            observe_name = "OBSERVE_HOLD"
        else:
            raise ProfileError("pump_state must be OFF or HOLD")

        name = self._above_name(point, hold=hold)
        self.actions.register_runtime(
            self._pose_action(name, self.store.above_pwm(point), pump_pwm=pump_pwm)
        )
        runtime_prefix = (
            f"LITE_DROP_MOVE_ABOVE_{point.point_id}_{normalized_pump_state}"
        )
        sequence = SequenceDefinition(
            name=f"MANUAL:LITE_DROP_MOVE_ABOVE:{point.point_id}",
            display_name=f"Lite move ABOVE {point.point_id}",
            steps=(ActionStep(name),),
        )
        sequence = j1_last_sequence(
            self.actions,
            sequence,
            initial_action=self.actions.get(observe_name),
            runtime_prefix=runtime_prefix,
        )
        sequence = self._stage_safe_above(point, sequence, runtime_prefix)
        self._assert_pump(sequence, pump_pwm)
        return sequence

    def build_move_drop(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> SequenceDefinition:
        point, drop = self._require_drop(point_id)
        self._register_drop_actions(point, drop)
        prefix = self._prefix(point)
        names = [
            f"{prefix}_WP_{int(item['index']):02d}_IDLE"
            for item in drop["waypoints"]
        ]
        if drop["drop_final_pwm"] != drop["drop_auto_pwm"]:
            names.append(f"{prefix}_DROP_FINAL_IDLE")
        sequence = SequenceDefinition(
            name=f"MANUAL:LITE_DROP_MOVE:{point.point_id}",
            display_name=f"Lite MoveL DROP {point.point_id}",
            steps=tuple(ActionStep(name) for name in names),
        )
        if len(names) < 2:
            raise ProfileError("MoveL DROP requires ABOVE plus at least one Z waypoint")
        self._assert_pump(sequence, self.pump_off_pwm)
        return sequence

    def build_move_waypoint(
        self,
        point_id: str | PointRef | tuple[int, int],
        target_index: int,
        *,
        from_final_correction: bool = False,
    ) -> SequenceDefinition:
        """Move to one adjacent saved Cartesian waypoint for guided testing."""
        point, drop = self._require_drop(point_id)
        self._register_drop_actions(point, drop)
        index = int(target_index)
        waypoints = list(drop["waypoints"])
        if not 0 <= index < len(waypoints):
            raise ProfileError(
                f"waypoint {index} outside 0..{max(0, len(waypoints) - 1)}"
            )
        if int(waypoints[index]["index"]) != index:
            raise ProfileError("saved MoveL waypoint indices must be contiguous")
        prefix = self._prefix(point)
        actions = [f"{prefix}_WP_{index:02d}_IDLE"]
        if from_final_correction:
            last = len(waypoints) - 1
            if index != last - 1:
                raise ProfileError(
                    "leaving corrected DROP must first return through the last auto waypoint"
                )
            actions.insert(0, f"{prefix}_WP_{last:02d}_IDLE")
        sequence = SequenceDefinition(
            name=f"MANUAL:LITE_DROP_WAYPOINT_{index:02d}:{point.point_id}",
            display_name=f"Lite waypoint WP{index:02d} {point.point_id}",
            steps=tuple(ActionStep(action) for action in actions),
        )
        self._assert_pump(sequence, self.pump_off_pwm)
        return sequence

    def build_retract_from_waypoint(
        self,
        point_id: str | PointRef | tuple[int, int],
        current_index: int,
    ) -> SequenceDefinition:
        """Return from an intermediate auto waypoint using only saved reverse points."""
        point, drop = self._require_drop(point_id)
        self._register_drop_actions(point, drop)
        index = int(current_index)
        waypoint_count = len(drop["waypoints"])
        if not 1 <= index < waypoint_count:
            raise ProfileError(
                f"intermediate retract index {index} outside 1..{waypoint_count - 1}"
            )
        prefix = self._prefix(point)
        names = [
            f"{prefix}_WP_{target:02d}_IDLE"
            for target in range(index - 1, -1, -1)
        ]
        sequence = SequenceDefinition(
            name=f"MANUAL:LITE_DROP_WAYPOINT_RETURN:{point.point_id}",
            display_name=f"Lite waypoint return to ABOVE {point.point_id}",
            steps=tuple(ActionStep(name) for name in names),
        )
        if not names[-1].endswith("_WP_00_IDLE"):
            raise ProfileError("waypoint return must finish at exact ABOVE")
        self._assert_pump(sequence, self.pump_off_pwm)
        return sequence

    def build_retract(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> SequenceDefinition:
        point, drop = self._require_drop(point_id)
        self._register_drop_actions(point, drop)
        names = self._reverse_names(point, drop)
        sequence = SequenceDefinition(
            name=f"MANUAL:LITE_DROP_RETRACT:{point.point_id}",
            display_name=f"Lite vertical retract {point.point_id}",
            steps=tuple(ActionStep(name) for name in names),
        )
        if not names or not names[-1].endswith("_WP_00_IDLE"):
            raise ProfileError("retract must finish at the exact saved ABOVE waypoint")
        self._assert_pump(sequence, self.pump_off_pwm)
        return sequence

    def build_test_place(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> SequenceDefinition:
        point, drop = self._require_drop(point_id)
        self._register_drop_actions(point, drop)
        prefix = self._prefix(point)
        steps: list[ActionStep | WaitStep] = list(
            pick_piece(self.vacuum_build_ms).steps
        )
        steps.append(ActionStep("CARRY_HIGH_P77_HOLD"))
        steps.extend(
            ActionStep(f"{prefix}_WP_{int(item['index']):02d}_HOLD")
            for item in drop["waypoints"]
        )
        if drop["drop_final_pwm"] != drop["drop_auto_pwm"]:
            steps.append(ActionStep(f"{prefix}_DROP_FINAL_HOLD"))
        steps.append(ActionStep(f"{prefix}_DROP_FINAL_RELEASE"))
        steps.append(WaitStep("VACUUM RELEASE", self.release_ms))
        steps.extend(ActionStep(name) for name in self._reverse_names(point, drop))
        sequence = SequenceDefinition(
            name=f"MANUAL:LITE_DROP_TEST_PLACE:{point.point_id}",
            display_name=f"Lite Test PLACE {point.point_id}",
            steps=tuple(steps),
        )
        names = sequence.action_names
        carry = names.index("CARRY_HIGH_P77_HOLD")
        above = names.index(f"{prefix}_WP_00_HOLD")
        release = names.index(f"{prefix}_DROP_FINAL_RELEASE")
        final_above = len(names) - 1
        if not carry < above < release < final_above or not names[-1].endswith(
            "_WP_00_IDLE"
        ):
            raise ProfileError("unsafe Lite Test PLACE ordering")
        expected_descent = tuple(
            f"{prefix}_WP_{int(item['index']):02d}_HOLD"
            for item in drop["waypoints"]
        )
        actual_descent = tuple(name for name in names if "_WP_" in name and name.endswith("_HOLD"))
        if actual_descent != expected_descent:
            raise ProfileError("Test PLACE must use every saved descent waypoint in order")
        expected_reverse = tuple(self._reverse_names(point, drop))
        if tuple(names[release + 1 :]) != expected_reverse:
            raise ProfileError("Test PLACE must use the exact saved reverse path")
        self._assert_test_place_pump(sequence)
        return sequence

    def build_safe_return_from_above(
        self,
        point_id: str | PointRef | tuple[int, int],
        *,
        pump_state: str = "OFF",
    ) -> SequenceDefinition:
        """Return directly from ABOVE to OBSERVE with J1 commanded first.

        Leaving the board runs the reverse of the entry policy in exactly two
        coarse phases: keep J0/J2/J3/J4 at ABOVE while J1 moves to OBSERVE,
        then keep J1 at OBSERVE while J0/J2/J3/J4 move to OBSERVE.

        DROP must already have been retracted to ABOVE by the existing reverse
        path before this sequence is used.
        """
        point = parse_point_id(point_id)
        normalized_pump_state = str(pump_state).strip().upper()
        if normalized_pump_state == "OFF":
            hold = False
            pump_pwm = self.pump_off_pwm
            observe_name = "OBSERVE_IDLE"
            runtime_suffix = ""
        elif normalized_pump_state == "HOLD":
            hold = True
            pump_pwm = self.pump_hold_pwm
            observe_name = "OBSERVE_HOLD"
            runtime_suffix = "_HOLD"
        else:
            raise ProfileError("pump_state must be OFF or HOLD")
        above = self.store.above_pwm(point)
        initial = self._pose_action(
            self._above_name(point, hold=hold), above, pump_pwm=pump_pwm
        )
        sequence = SequenceDefinition(
            name=f"MANUAL:LITE_DROP_SAFE_RETURN:{point.point_id}",
            display_name=f"Lite safe return from {point.point_id} ABOVE",
            steps=(ActionStep(observe_name),),
        )
        sequence = j1_first_sequence(
            self.actions,
            sequence,
            initial_action=initial,
            runtime_prefix=(
                f"LITE_DROP_SAFE_RETURN_{point.point_id}{runtime_suffix}"
            ),
            phase_time_ms=BOARD_SAFE_RETURN_PHASE_TIME_MS,
        )
        self._assert_pump(sequence, pump_pwm)
        return sequence

    def build_safe_return_from_drop(
        self,
        point_id: str | PointRef | tuple[int, int],
        *,
        pump_state: str = "OFF",
    ) -> SequenceDefinition:
        """Retract by the saved reverse path, then return with J1 FIRST."""
        point, drop = self._require_drop(point_id)
        normalized_pump_state = str(pump_state).strip().upper()
        if normalized_pump_state not in {"OFF", "HOLD"}:
            raise ProfileError("pump_state must be OFF or HOLD")
        hold = normalized_pump_state == "HOLD"
        pump_pwm = self.pump_hold_pwm if hold else self.pump_off_pwm
        self._register_drop_actions(point, drop)
        reverse_names = self._reverse_names(point, drop, hold=hold)
        if not reverse_names or not reverse_names[-1].endswith(
            f"_WP_00_{'HOLD' if hold else 'IDLE'}"
        ):
            raise ProfileError("safe return must retract to exact ABOVE first")
        return_from_above = self.build_safe_return_from_above(
            point,
            pump_state=normalized_pump_state,
        )
        sequence = SequenceDefinition(
            name=f"MANUAL:LITE_DROP_SAFE_RETURN:{point.point_id}",
            display_name=f"Lite DROP reverse + safe return {point.point_id}",
            steps=(
                *(ActionStep(name) for name in reverse_names),
                *return_from_above.steps,
            ),
        )
        self._assert_pump(sequence, pump_pwm)
        return sequence

    def apply_joint_target(
        self,
        point_id: str | PointRef | tuple[int, int],
        joint: int,
        correction: int,
    ) -> int:
        if not 0 <= int(joint) <= 4:
            raise ProfileError("only J0..J4 may be corrected")
        _point, drop = self._require_drop(point_id)
        key = f"{int(joint):03d}"
        target = int(drop["drop_auto_pwm"][key]) + int(correction)
        candidate = dict(drop["drop_auto_pwm"])
        candidate[key] = target
        self.store._validate_pwm(candidate)
        return target

    def _require_drop(
        self, point_id: str | PointRef | tuple[int, int]
    ) -> tuple[PointRef, dict[str, Any]]:
        point = parse_point_id(point_id)
        drop = self.store.drop_record(point)
        if drop is None:
            raise ProfileError(f"generate DROP before using {point.point_id}")
        if drop.get("status") in {
            DropStatus.NOT_GENERATED.value,
            DropStatus.MOVE_L_UNREACHABLE.value,
            DropStatus.INVALID.value,
        }:
            raise ProfileError(
                f"DROP {drop.get('status')}: {drop.get('reason') or '-'}"
            )
        if not drop.get("waypoints") or drop.get("drop_final_pwm") is None:
            raise ProfileError(f"{point.point_id} DROP record is incomplete")
        return point, drop

    def _register_drop_actions(
        self, point: PointRef, drop: Mapping[str, Any]
    ) -> None:
        prefix = self._prefix(point)
        for item in drop.get("waypoints") or []:
            index = int(item["index"])
            pwm = item["pwm"]
            self.actions.register_runtime(
                self._pose_action(
                    f"{prefix}_WP_{index:02d}_IDLE", pwm, pump_pwm=self.pump_off_pwm
                )
            )
            self.actions.register_runtime(
                self._pose_action(
                    f"{prefix}_WP_{index:02d}_HOLD", pwm, pump_pwm=self.pump_hold_pwm
                )
            )
        self.actions.register_runtime(
            self._pose_action(
                f"{prefix}_DROP_FINAL_IDLE",
                drop["drop_final_pwm"],
                pump_pwm=self.pump_off_pwm,
            )
        )
        self.actions.register_runtime(
            self._pose_action(
                f"{prefix}_DROP_FINAL_HOLD",
                drop["drop_final_pwm"],
                pump_pwm=self.pump_hold_pwm,
            )
        )
        self.actions.register_runtime(
            self._pose_action(
                f"{prefix}_DROP_FINAL_RELEASE",
                drop["drop_final_pwm"],
                pump_pwm=self.pump_off_pwm,
            )
        )

    def _reverse_names(
        self,
        point: PointRef,
        drop: Mapping[str, Any],
        *,
        hold: bool = False,
    ) -> list[str]:
        prefix = self._prefix(point)
        suffix = "HOLD" if hold else "IDLE"
        names: list[str] = []
        if drop["drop_final_pwm"] != drop["drop_auto_pwm"]:
            last = int(drop["waypoints"][-1]["index"])
            names.append(f"{prefix}_WP_{last:02d}_{suffix}")
        names.extend(
            f"{prefix}_WP_{int(index):02d}_{suffix}"
            for index in drop.get("reverse_ascent_indices") or []
        )
        return names

    def _pose_action(
        self, name: str, pwm: Mapping[str | int, int], *, pump_pwm: int
    ) -> Action:
        spatial = self.store._validate_pwm(pwm)
        values = [spatial[key] for key in SPATIAL_KEYS]
        values.extend((int(pump_pwm), 1500, 1500))
        targets = tuple(
            ServoTarget(index, value, self.move_time_ms)
            for index, value in enumerate(values)
        )
        command = "{" + "".join(
            f"#{target.servo_id:03d}P{target.pwm:04d}T{target.time_ms:04d}!"
            for target in targets
        ) + "}"
        return Action(name=name, command=command, targets=targets)

    def _assert_pump(self, sequence: SequenceDefinition, expected: int) -> None:
        for name in sequence.action_names:
            if self.actions.get(name).target(5).pwm != int(expected):
                raise ProfileError(f"{name} violates the locked J5 pump state")

    def _assert_test_place_pump(self, sequence: SequenceDefinition) -> None:
        holding = False
        released = False
        for name in sequence.action_names:
            if name == "SOURCE_TOUCH_HOLD":
                holding = True
            if name.endswith("_DROP_FINAL_RELEASE"):
                released = True
            expected = self.pump_hold_pwm if holding and not released else self.pump_off_pwm
            if self.actions.get(name).target(5).pwm != expected:
                raise ProfileError(f"{name} violates Test PLACE J5 pump semantics")

    @staticmethod
    def _prefix(point: PointRef) -> str:
        return f"LITE_DROP_P{point.row:02d}_{point.col:02d}"

    def _above_name(self, point: PointRef, *, hold: bool) -> str:
        return f"{self._prefix(point)}_ABOVE_{'HOLD' if hold else 'IDLE'}"

    def _stage_safe_above(
        self,
        point: PointRef,
        sequence: SequenceDefinition,
        runtime_prefix: str,
    ) -> SequenceDefinition:
        """Insert the transient SAFE ABOVE staging pose before the exact ABOVE.

        SAFE ABOVE keeps J0/J2/J3/J4 at the exact ABOVE values and raises J1 by
        ``SAFE_ABOVE_J1_LIFT_PWM``.  It is a runtime safety staging pose only;
        it never overwrites, modifies, or saves a new Golden ABOVE.
        """
        if golden_for(point.row, point.col) is None:
            return sequence
        steps = list(sequence.steps)
        if not steps or not isinstance(steps[-1], ActionStep):
            raise ProfileError("move ABOVE must end at the exact ABOVE pose")
        exact = self.actions.get(steps[-1].action_name)
        exact_pwm = {int(target.servo_id): int(target.pwm) for target in exact.targets}
        safe_spatial = {f"{joint:03d}": int(exact_pwm[joint]) for joint in range(5)}
        safe_spatial["001"] = (
            int(safe_spatial["001"]) + SAFE_ABOVE_J1_LIFT_PWM
        )
        safe_name = f"{runtime_prefix}_SAFE_ABOVE"
        self.actions.register_runtime(
            self._pose_action(safe_name, safe_spatial, pump_pwm=int(exact_pwm[5]))
        )
        steps.insert(len(steps) - 1, ActionStep(safe_name))
        return SequenceDefinition(
            name=sequence.name,
            display_name=sequence.display_name,
            steps=tuple(steps),
            requires_board=sequence.requires_board,
        )
