from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Mapping

from app.arm.actions import Action, ActionLibrary, ServoTarget
from app.arm.ordered_motion import (
    BOARD_SAFE_RETURN_PHASE_TIME_MS,
    j1_first_sequence,
)
from app.arm.sequences import ActionStep, SequenceDefinition, WaitStep, pick_piece
from app.integrated_v1.profile import ProfileError


PRODUCT = "J1 Gomoku Lite P77 Manual MoveL Tuner"

P77_GOLDEN_ABOVE = {
    "J0": 1500,
    "J1": 1230,
    "J2": 870,
    "J3": 1230,
    "J4": 1500,
}

SPATIAL_JOINTS = tuple(P77_GOLDEN_ABOVE)

SAFE_ABOVE_J1_LIFT_PWM = 30

# 当前 Manual MoveL：ABOVE -> FINAL DROP
MANUAL_DESCENT_STEP_COUNT = 1
FINAL_DROP_STEP_INDEX = MANUAL_DESCENT_STEP_COUNT

FINAL_DROP_J1_REFERENCE_PWM = 1050


def _now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


class P77ManualMoveLStore:
    """Independent P77-only manual step store; Golden ABOVE is immutable."""

    def __init__(
        self,
        path: str | Path,
        *,
        joint_limits: Mapping[int | str, tuple[int, int]] | None = None,
    ) -> None:
        self.path = Path(path)
        self.joint_limits = {
            f"J{int(joint)}": (int(bounds[0]), int(bounds[1]))
            for joint, bounds in (joint_limits or {joint: (550, 2450) for joint in range(5)}).items()
        }
        if set(self.joint_limits) != set(SPATIAL_JOINTS):
            raise ProfileError("P77 manual MoveL limits must contain J0..J4 only")
        self.data: dict[str, Any] | None = None

    def load_or_initialize(self) -> dict[str, Any]:
        if self.path.is_file():
            return self.load()
        now = _now()
        self.data = {
            "schema_version": 1,
            "product": PRODUCT,
            "point_id": "P77",
            "board": [7, 7],
            "created_at": now,
            "updated_at": now,
            "golden_above_pwm": dict(P77_GOLDEN_ABOVE),
            "steps": [self._new_step(0, P77_GOLDEN_ABOVE)],
            "drop_candidate": None,
        }
        return self._require_data()

    def load(self) -> dict[str, Any]:
        raw = json.loads(self.path.read_text(encoding="utf-8"))
        if raw.get("product") != PRODUCT or int(raw.get("schema_version", 0)) != 1:
            raise ProfileError("not a P77 Manual MoveL Tuner file")
        if raw.get("point_id") != "P77" or raw.get("board") != [7, 7]:
            raise ProfileError("manual MoveL data must remain scoped to P77 (7,7)")
        if self._normalize_pwm(raw.get("golden_above_pwm") or {}) != P77_GOLDEN_ABOVE:
            raise ProfileError("P77 Golden ABOVE was modified")
        steps = raw.get("steps") or []
        if not steps or [int(item.get("step_index", -1)) for item in steps] != list(
            range(len(steps))
        ):
            raise ProfileError("manual MoveL step indices must be contiguous from Step0")
        if len(steps) > MANUAL_DESCENT_STEP_COUNT + 1:
            raise ProfileError("P77 Manual MoveL currently supports ABOVE -> FINAL DROP only")
        for index, item in enumerate(steps):
            item["final_pwm"] = self.validate_candidate(index, item.get("final_pwm") or {}, steps=steps)
            reference = item.get("auto_pwm")
            if reference is None:
                reference = P77_GOLDEN_ABOVE if index == 0 else steps[index - 1]["final_pwm"]
            item["auto_pwm"] = self._normalize_pwm(reference)
            item["correction_pwm"] = self._delta(
                item["final_pwm"], item["auto_pwm"]
            )
            if bool(item.get("hardware_verified", False)):
                raise ProfileError("manual MoveL steps cannot self-assert HARDWARE VERIFIED")
            item["hardware_verified"] = False
            item["operator_confirmed"] = bool(item.get("operator_confirmed", False))
        if steps[0]["final_pwm"] != P77_GOLDEN_ABOVE:
            raise ProfileError("Step0 must equal immutable P77 Golden ABOVE")
        drop = raw.get("drop_candidate")
        if drop is not None:
            if int(drop.get("step_index", -1)) != FINAL_DROP_STEP_INDEX:
                raise ProfileError("P77 Manual MoveL DROP must be Step3")
            drop["final_pwm"] = self._normalize_pwm(drop.get("final_pwm") or {})
            if drop.get("source") != "manual_movel_tuning":
                raise ProfileError("P77 DROP candidate has an invalid source")
            if bool(drop.get("hardware_verified", False)):
                raise ProfileError("P77 DROP candidate cannot self-assert HARDWARE VERIFIED")
            drop["hardware_verified"] = False
        self.data = raw
        return raw

    def save(self) -> Path:
        data = self._require_data()
        if self.step(0)["final_pwm"] != P77_GOLDEN_ABOVE:
            raise ProfileError("refusing to save modified P77 Golden ABOVE")
        data["updated_at"] = _now()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.path)
        return self.path

    def step_count(self) -> int:
        return len(self._require_data()["steps"])

    def step(self, index: int) -> dict[str, Any]:
        step_index = int(index)
        if not 0 <= step_index <= FINAL_DROP_STEP_INDEX:
            raise ProfileError("manual MoveL step must be Step0..Step3")
        try:
            return deepcopy(self._require_data()["steps"][step_index])
        except IndexError as exc:
            raise ProfileError(f"manual MoveL Step{step_index} does not exist") from exc

    def validate_candidate(
        self,
        index: int,
        pwm: Mapping[str | int, int],
        *,
        steps: list[dict[str, Any]] | None = None,
    ) -> dict[str, int]:
        step_index = int(index)
        if not 0 <= step_index <= FINAL_DROP_STEP_INDEX:
            raise ProfileError("manual MoveL step must be Step0..Step3")
        values = self._normalize_pwm(pwm)
        for joint, value in values.items():
            lower, upper = self.joint_limits[joint]
            if not lower <= value <= upper:
                raise ProfileError(f"{joint} PWM {value} outside {lower}..{upper}")
        if step_index == 0:
            if values != P77_GOLDEN_ABOVE:
                raise ProfileError("Step0 is immutable P77 Golden ABOVE")
            return values
        records = steps if steps is not None else self._require_data()["steps"]
        if step_index - 1 >= len(records):
            raise ProfileError(f"Step{step_index} requires saved Step{step_index - 1}")
        return values

    def save_step(
        self,
        index: int,
        final_pwm: Mapping[str | int, int],
        *,
        auto_pwm: Mapping[str | int, int] | None = None,
        correction_pwm: Mapping[str | int, int] | None = None,
    ) -> dict[str, Any]:
        step_index = int(index)
        steps = self._require_data()["steps"]
        if not 0 <= step_index < len(steps):
            raise ProfileError("create the next manual step before saving it")
        values = self.validate_candidate(step_index, final_pwm)
        existing = steps[step_index]
        reference = auto_pwm or existing.get("auto_pwm")
        if reference is None:
            reference = (
                P77_GOLDEN_ABOVE
                if step_index == 0
                else steps[step_index - 1]["final_pwm"]
            )
        reference_pwm = self._normalize_pwm(reference)
        record = self._new_step(
            step_index,
            values,
            reference_pwm=reference_pwm,
        )
        calculated_correction = self._delta(values, reference_pwm)
        if correction_pwm is not None:
            requested_correction = self._normalize_delta(correction_pwm)
            if requested_correction != calculated_correction:
                raise ProfileError(
                    "manual MoveL correction must equal final_pwm - auto_pwm"
                )
        record["correction_pwm"] = calculated_correction
        steps[step_index] = record
        self._require_data()["drop_candidate"] = None
        return deepcopy(record)

    def confirm_step(self, index: int) -> dict[str, Any]:
        step_index = int(index)
        record = self._require_data()["steps"][step_index]
        record["operator_confirmed"] = True
        record["confirmed_at"] = _now()
        record["hardware_verified"] = False
        return deepcopy(record)

    def create_next_step(self, index: int) -> dict[str, Any]:
        step_index = int(index)
        steps = self._require_data()["steps"]
        if not 0 <= step_index < len(steps):
            raise ProfileError(f"Step{step_index} does not exist")
        if not bool(steps[step_index].get("operator_confirmed", False)):
            raise ProfileError("Confirm the saved current step before creating Next Step")
        if step_index >= FINAL_DROP_STEP_INDEX:
            raise ProfileError("P77 Manual MoveL ends at Step1 FINAL DROP")
        next_index = step_index + 1
        if next_index < len(steps):
            return deepcopy(steps[next_index])
        inherited = self._normalize_pwm(steps[step_index]["final_pwm"])
        record = self._new_step(
            next_index,
            inherited,
            reference_pwm=inherited,
        )
        record["inherited_from_step"] = step_index
        record["direction_suggestion"] = {
            "J1": "small decrease",
            "J2": "small decrease",
            "J3": "small increase",
            "note": "operator-tunable suggestion; not a kinematic truth",
        }
        if next_index == FINAL_DROP_STEP_INDEX:
            record["final_drop_reference"] = {
                "J1_approx_pwm": FINAL_DROP_J1_REFERENCE_PWM,
                "note": "hardware observation reference only; not verified evidence",
            }
        steps.append(record)
        return deepcopy(record)

    def set_as_drop(self, index: int) -> dict[str, Any]:
        step_index = int(index)
        if step_index != FINAL_DROP_STEP_INDEX:
            raise ProfileError("only Step3 can be set as P77 FINAL DROP")
        step = self._require_data()["steps"][step_index]
        if not bool(step.get("operator_confirmed", False)):
            raise ProfileError("Confirm Step before setting it as P77 DROP")
        for index in range(step_index):
            if not bool(self._require_data()["steps"][index].get("operator_confirmed", False)):
                raise ProfileError(
                    f"Confirm saved Step{index} before setting the P77 DROP path"
                )
        drop = {
            "step_index": step_index,
            "final_pwm": self._normalize_pwm(step["final_pwm"]),
            "source": "manual_movel_tuning",
            "operator_confirmed": True,
            "hardware_verified": False,
            "saved_at": _now(),
        }
        self._require_data()["drop_candidate"] = drop
        return deepcopy(drop)

    def drop_candidate(self) -> dict[str, Any] | None:
        value = self._require_data().get("drop_candidate")
        return None if value is None else deepcopy(value)

    def calibration_payload(self) -> dict[str, Any]:
        drop = self.drop_candidate()
        if drop is None:
            raise ProfileError("Set As P77 DROP before applying Manual MoveL calibration")
        drop_index = int(drop["step_index"])
        if drop_index != FINAL_DROP_STEP_INDEX:
            raise ProfileError("P77 Manual MoveL DROP must be Step3")
        steps: list[dict[str, Any]] = []
        for index in range(drop_index + 1):
            step = self.step(index)
            if not bool(step.get("operator_confirmed", False)):
                raise ProfileError(
                    f"Confirm saved Step{index} before using the P77 full flow"
                )
            steps.append(
                {
                    "step_index": index,
                    "final_pwm": self._normalize_pwm(step["final_pwm"]),
                    "operator_confirmed": True,
                    "hardware_verified": False,
                }
            )
        if steps[-1]["final_pwm"] != drop["final_pwm"]:
            raise ProfileError("P77 DROP must match the final saved manual step")
        return {
            "point_id": "P77",
            "board": [7, 7],
            "source": "manual_movel_tuning",
            "drop_step_index": drop_index,
            "final_pwm": self._normalize_pwm(drop["final_pwm"]),
            "steps": steps,
            "operator_confirmed": True,
            "hardware_verified": False,
            "saved_at": str(drop["saved_at"]),
        }

    def mark_movel_applied(self) -> dict[str, Any]:
        drop = self._require_data().get("drop_candidate")
        if drop is None:
            raise ProfileError("P77 DROP candidate is not set")
        drop["movel_profile_applied"] = True
        drop["movel_profile_applied_at"] = _now()
        return deepcopy(drop)

    @staticmethod
    def _new_step(
        index: int,
        pwm: Mapping[str | int, int],
        *,
        reference_pwm: Mapping[str | int, int] | None = None,
    ) -> dict[str, Any]:
        final = P77ManualMoveLStore._normalize_pwm(pwm)
        reference = P77ManualMoveLStore._normalize_pwm(reference_pwm or pwm)
        return {
            "step_index": int(index),
            "auto_pwm": reference,
            "correction_pwm": P77ManualMoveLStore._delta(final, reference),
            "final_pwm": final,
            "hardware_verified": False,
            "operator_confirmed": False,
        }

    @staticmethod
    def _delta(
        final_pwm: Mapping[str | int, int],
        reference_pwm: Mapping[str | int, int],
    ) -> dict[str, int]:
        final = P77ManualMoveLStore._normalize_pwm(final_pwm)
        reference = P77ManualMoveLStore._normalize_pwm(reference_pwm)
        return {
            joint: int(final[joint]) - int(reference[joint])
            for joint in SPATIAL_JOINTS
        }

    @staticmethod
    def _normalize_pwm(pwm: Mapping[str | int, int]) -> dict[str, int]:
        normalized: dict[str, int] = {}
        for joint in range(5):
            candidates = (f"J{joint}", f"{joint:03d}", joint)
            for key in candidates:
                if key in pwm:
                    normalized[f"J{joint}"] = int(pwm[key])
                    break
            else:
                raise ProfileError(f"manual MoveL PWM missing J{joint}")
        return normalized

    @staticmethod
    def _normalize_delta(pwm: Mapping[str | int, int]) -> dict[str, int]:
        return P77ManualMoveLStore._normalize_pwm(pwm)

    def _require_data(self) -> dict[str, Any]:
        if self.data is None:
            raise ProfileError("initialize the P77 manual MoveL store first")
        return self.data


class P77ManualMoveLSequenceBuilder:
    """Build P77 manual actions that command J0..J4 and omit locked J5."""

    def __init__(
        self,
        *,
        actions: ActionLibrary,
        store: P77ManualMoveLStore,
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

    def build_enter_above(self, observation_pwm: Mapping[str | int, int]) -> SequenceDefinition:
        observe = self.store._normalize_pwm(observation_pwm)
        above = self.store.step(0)["final_pwm"]
        phase_a = dict(above)
        phase_a["J1"] = observe["J1"]
        safe_above = dict(above)
        safe_above["J1"] += SAFE_ABOVE_J1_LIFT_PWM
        names = (
            "P77_MANUAL_MOVEL_ENTER_J1_HELD",
            "P77_MANUAL_MOVEL_SAFE_ABOVE",
            "P77_MANUAL_MOVEL_STEP_00",
        )
        for name, pwm in zip(names, (phase_a, safe_above, above)):
            self._register(name, pwm)
        return SequenceDefinition(
            name="MANUAL:P77_MOVEL:MOVE:0",
            display_name="P77 Manual MoveL enter immutable Step0 ABOVE",
            steps=tuple(ActionStep(name) for name in names),
        )

    def build_move_candidate(
        self,
        index: int,
        final_pwm: Mapping[str | int, int],
    ) -> SequenceDefinition:
        step_index = int(index)
        pwm = self.store.validate_candidate(step_index, final_pwm)
        name = f"P77_MANUAL_MOVEL_STEP_{step_index:02d}_CANDIDATE"
        self._register(name, pwm)
        return SequenceDefinition(
            name=f"MANUAL:P77_MOVEL:MOVE:{step_index}",
            display_name=f"P77 Manual MoveL Step {step_index}",
            steps=(ActionStep(name),),
        )

    def build_return_previous(self, current_index: int) -> SequenceDefinition:
        current = int(current_index)
        if current <= 0:
            raise ProfileError("already at P77 ABOVE Step0")
        target = current - 1
        name = self._saved_step_action(target)
        return SequenceDefinition(
            name=f"MANUAL:P77_MOVEL:RETURN_PREVIOUS:{target}",
            display_name=f"P77 Manual MoveL return to Step {target}",
            steps=(ActionStep(name),),
        )

    def build_return_above(self, current_index: int) -> SequenceDefinition:
        current = int(current_index)
        if current <= 0:
            raise ProfileError("already at P77 ABOVE Step0")
        names = tuple(self._saved_step_action(index) for index in range(current - 1, -1, -1))
        return SequenceDefinition(
            name="MANUAL:P77_MOVEL:RETURN_ABOVE:0",
            display_name="P77 Manual MoveL exact saved reverse to ABOVE",
            steps=tuple(ActionStep(name) for name in names),
        )

    def build_return_observation(self) -> SequenceDefinition:
        """Return from exact P77 Step0 ABOVE using the J1-FIRST board-exit policy."""
        name = "P77_MANUAL_RETURN_OBSERVATION_START"
        self._register_full_pose(
            name,
            self.store.step(0)["final_pwm"],
            pump_pwm=self.pump_off_pwm,
        )
        return j1_first_sequence(
            self.actions,
            SequenceDefinition(
                name="MANUAL:P77_MOVEL:RETURN_OBSERVATION",
                display_name="P77 Manual MoveL return ABOVE to Observation",
                steps=(ActionStep("OBSERVE_IDLE"),),
            ),
            initial_action=self.actions.get(name),
            runtime_prefix="P77_MANUAL_RETURN_OBSERVATION",
            phase_time_ms=BOARD_SAFE_RETURN_PHASE_TIME_MS,
        )

    def build_full_pick_place(
        self,
        *,
        start_from_above: bool = False,
    ) -> SequenceDefinition:
        """Run the P77 pick/place calibration using only confirmed manual steps."""
        payload = self.store.calibration_payload()
        path = list(payload["steps"])
        above = dict(path[0]["final_pwm"])
        observe = self.store._normalize_pwm(
            {
                f"J{joint}": self.actions.get("OBSERVE_HOLD").target(joint).pwm
                for joint in range(5)
            }
        )
        phase_a = dict(above)
        phase_a["J1"] = observe["J1"]
        safe_above = dict(above)
        safe_above["J1"] += SAFE_ABOVE_J1_LIFT_PWM

        entry_names = (
            "P77_MANUAL_FULL_ENTER_J1_HELD_HOLD",
            "P77_MANUAL_FULL_SAFE_ABOVE_HOLD",
            "P77_MANUAL_FULL_STEP_00_HOLD",
        )
        for name, pwm in zip(entry_names, (phase_a, safe_above, above)):
            self._register_full_pose(name, pwm, pump_pwm=self.pump_hold_pwm)

        descent_names: list[str] = []
        for step in path[1:]:
            index = int(step["step_index"])
            name = f"P77_MANUAL_FULL_STEP_{index:02d}_HOLD"
            self._register_full_pose(
                name,
                step["final_pwm"],
                pump_pwm=self.pump_hold_pwm,
            )
            descent_names.append(name)

        final_pwm = path[-1]["final_pwm"]
        release_name = "P77_MANUAL_FULL_DROP_RELEASE"
        self._register_full_pose(
            release_name,
            final_pwm,
            pump_pwm=self.pump_off_pwm,
        )

        reverse_names: list[str] = []
        for step in reversed(path[:-1]):
            index = int(step["step_index"])
            name = f"P77_MANUAL_FULL_STEP_{index:02d}_IDLE"
            self._register_full_pose(
                name,
                step["final_pwm"],
                pump_pwm=self.pump_off_pwm,
            )
            reverse_names.append(name)

        return_start = self.actions.get(reverse_names[-1])
        return_sequence = j1_first_sequence(
            self.actions,
            SequenceDefinition(
                name="MANUAL:P77_MOVEL:FULL_CYCLE_RETURN",
                display_name="P77 Manual MoveL full-cycle return to Observation",
                steps=(ActionStep("OBSERVE_IDLE"),),
            ),
            initial_action=return_start,
            runtime_prefix="P77_MANUAL_FULL_RETURN",
            phase_time_ms=BOARD_SAFE_RETURN_PHASE_TIME_MS,
        )
        steps: list[ActionStep | WaitStep] = []
        if start_from_above:
            steps.extend(return_sequence.steps)
        steps.extend(pick_piece(self.vacuum_build_ms).steps)
        steps.extend(ActionStep(name) for name in entry_names)
        steps.extend(ActionStep(name) for name in descent_names)
        steps.append(ActionStep(release_name))
        steps.append(WaitStep("VACUUM RELEASE", self.release_ms))
        steps.extend(ActionStep(name) for name in reverse_names)
        steps.extend(return_sequence.steps)
        sequence = SequenceDefinition(
            name="MANUAL:P77_MOVEL:FULL_CYCLE",
            display_name="P77 manual calibrated pick and place full flow",
            steps=tuple(steps),
            requires_board=True,
        )
        self._assert_full_cycle(sequence, payload)
        return sequence

    def _saved_step_action(self, index: int) -> str:
        step = self.store.step(index)
        name = f"P77_MANUAL_MOVEL_STEP_{int(index):02d}_SAVED"
        self._register(name, step["final_pwm"])
        return name

    def _register(self, name: str, pwm: Mapping[str | int, int]) -> None:
        values = self.store._normalize_pwm(pwm)
        targets = tuple(
            ServoTarget(joint, values[f"J{joint}"], self.move_time_ms)
            for joint in range(5)
        )
        command = "{" + "".join(
            f"#{target.servo_id:03d}P{target.pwm:04d}T{target.time_ms:04d}!"
            for target in targets
        ) + "}"
        self.actions.register_runtime(Action(name=name, command=command, targets=targets))

    def _register_full_pose(
        self,
        name: str,
        pwm: Mapping[str | int, int],
        *,
        pump_pwm: int,
    ) -> None:
        spatial = self.store._normalize_pwm(pwm)
        values = [spatial[f"J{joint}"] for joint in range(5)]
        values.extend((int(pump_pwm), 1500, 1500))
        targets = tuple(
            ServoTarget(joint, value, self.move_time_ms)
            for joint, value in enumerate(values)
        )
        command = "{" + "".join(
            f"#{target.servo_id:03d}P{target.pwm:04d}T{target.time_ms:04d}!"
            for target in targets
        ) + "}"
        self.actions.register_runtime(
            Action(name=name, command=command, targets=targets)
        )

    def _assert_full_cycle(
        self,
        sequence: SequenceDefinition,
        payload: Mapping[str, Any],
    ) -> None:
        names = sequence.action_names
        drop_index = int(payload["drop_step_index"])
        expected_descent = tuple(
            f"P77_MANUAL_FULL_STEP_{index:02d}_HOLD"
            for index in range(1, drop_index + 1)
        )
        actual_descent = tuple(
            name
            for name in names
            if name.startswith("P77_MANUAL_FULL_STEP_") and name.endswith("_HOLD")
            and name != "P77_MANUAL_FULL_STEP_00_HOLD"
        )
        if actual_descent != expected_descent:
            raise ProfileError("P77 full flow must use every saved manual step in order")
        expected_reverse = tuple(
            f"P77_MANUAL_FULL_STEP_{index:02d}_IDLE"
            for index in range(drop_index - 1, -1, -1)
        )
        release = names.index("P77_MANUAL_FULL_DROP_RELEASE")
        if tuple(names[release + 1 : release + 1 + len(expected_reverse)]) != expected_reverse:
            raise ProfileError("P77 full flow must reverse through every saved manual step")
        holding = False
        released = False
        for name in names:
            if name == "SOURCE_TOUCH_HOLD":
                holding = True
            if name == "P77_MANUAL_FULL_DROP_RELEASE":
                released = True
            expected = (
                self.pump_hold_pwm
                if holding and not released
                else self.pump_off_pwm
            )
            if self.actions.get(name).target(5).pwm != expected:
                raise ProfileError(f"{name} violates P77 full-flow pump semantics")
