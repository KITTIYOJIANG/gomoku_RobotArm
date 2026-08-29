from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from datetime import datetime
import logging
from pathlib import Path
from typing import Any, Mapping

from app.arm.actions import ActionLibrary
from app.arm.controller import SerialArmController
from app.stage5.safety import derive_calibration_limits

from .baseline import BaselineSnapshot, SPATIAL_KEYS
from .pick_poses import PickPoseStore
from .session import CalibrationMode, RapidCalibrationSession, SessionError
from .settings import Stage7Settings


LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class JogResult:
    joint_id: int
    requested_pwm: int
    applied_pwm: int
    status: str
    queued: bool
    sent: bool
    command: str | None = None


class RapidCalibrationCoordinator:
    """Stage 7 application service and coalescing live-jog queue."""

    def __init__(
        self,
        *,
        controller: SerialArmController,
        library: ActionLibrary,
        settings: Stage7Settings | None = None,
    ) -> None:
        self.controller = controller
        self.library = library
        self.settings = settings or Stage7Settings.load()
        self.limits = derive_calibration_limits(library)
        self.baseline = BaselineSnapshot(
            self.settings.baseline_path,
            board_size=self.settings.board_size,
            library=library,
        )
        self.pick_poses = PickPoseStore(self.settings.pick_pose_path, library)
        self.session: RapidCalibrationSession | None = None
        self.dry_run = bool(
            self.settings.default_dry_run
            or self.settings.force_dry_run
            or self.controller.dry_run
        )
        self._pending: OrderedDict[int, tuple[int, int]] = OrderedDict()
        self.last_jog_result: JogResult | None = None

    def reload_baseline(self) -> BaselineSnapshot:
        self.baseline = BaselineSnapshot(
            self.settings.baseline_path,
            board_size=self.settings.board_size,
            library=self.library,
        )
        return self.baseline

    def set_dry_run(self, enabled: bool) -> bool:
        requested = bool(enabled)
        if not requested and (self.settings.force_dry_run or self.controller.dry_run):
            self.dry_run = True
            return False
        self.dry_run = requested
        if self.dry_run:
            self._pending.clear()
        return True

    def new_session(
        self,
        mode: CalibrationMode | str,
        *,
        session_id: str | None = None,
        path: str | Path | None = None,
    ) -> RapidCalibrationSession:
        selected_mode = CalibrationMode(mode)
        identifier = session_id or datetime.now().strftime(
            f"%Y-%m-%d_%H%M%S_{selected_mode.value.lower()}"
        )
        destination = Path(path) if path is not None else self.settings.sessions_dir / f"{identifier}.json"
        session = RapidCalibrationSession(
            baseline=self.baseline,
            mode=selected_mode,
            session_id=identifier,
            path=destination,
            limits=self.limits,
        )
        session.history.append(
            {
                "timestamp": session.created_at,
                "event": "SESSION_CREATED",
                "mode": selected_mode.value,
            }
        )
        session.save()
        self.session = session
        LOGGER.info(
            "[STAGE7][SESSION_CREATED] id=%s mode=%s path=%s baseline_sha=%s",
            identifier,
            selected_mode.value,
            destination,
            self.baseline.source_sha256,
        )
        return session

    def load_session(self, path: str | Path) -> RapidCalibrationSession:
        self.session = RapidCalibrationSession.load(
            path,
            baseline=self.baseline,
            limits=self.limits,
        )
        return self.session

    def require_session(self) -> RapidCalibrationSession:
        if self.session is None:
            raise SessionError("create a calibration session first")
        return self.session

    def point_pwm(self, row: int, col: int) -> dict[str, int]:
        if self.session is None:
            return dict(self.baseline.get(row, col).pwm)
        return self.session.point_pwm(row, col)

    def save_anchor(
        self, row: int, col: int, pwm: Mapping[int | str, int]
    ) -> dict[str, Any]:
        session = self.require_session()
        anchor = session.save_anchor(row, col, pwm)
        session.save()
        LOGGER.info(
            "[STAGE7][ANCHOR] point=%s delta=%s clamped=%s",
            anchor["point_id"],
            anchor["delta_pwm"],
            int(anchor["clamped"]),
        )
        return anchor

    def recalculate(self) -> dict[str, dict[str, Any]]:
        session = self.require_session()
        result = session.recalculate()
        session.save()
        LOGGER.info(
            "[STAGE7][RECALCULATE] revision=%s anchors=%s points=%s",
            session.candidate_revision,
            len(session.anchors),
            len(result),
        )
        return result

    def verify(self, row: int, col: int) -> dict[str, Any]:
        session = self.require_session()
        result = session.verify(row, col)
        session.save()
        LOGGER.info("[STAGE7][VERIFIED] point=%s", result["point_id"])
        return result

    def commit(self) -> Path:
        return self.require_session().commit(self.settings.current_deployment_path)

    def rollback(self) -> Path:
        return self.require_session().commit_rollback(
            self.settings.current_deployment_path
        )

    @property
    def live_status(self) -> str:
        if self.settings.force_dry_run or self.controller.dry_run or self.dry_run:
            return "DRY RUN / NOT SENT"
        if not self.controller.is_connected:
            return "SERIAL NOT CONNECTED / NOT SENT"
        return f"LIVE HARDWARE · {self.controller.port or 'CONNECTED'}"

    def queue_live_jog(
        self,
        joint_id: int | str,
        requested_pwm: int,
        *,
        time_ms: int | None = None,
    ) -> JogResult:
        jid = int(str(joint_id))
        if jid not in range(5):
            raise ValueError("Stage 7 live jog permits J0..J4 only; J5 is the pump")
        requested = int(requested_pwm)
        applied = max(
            int(self.limits.joint_min[jid]),
            min(int(self.limits.joint_max[jid]), requested),
        )
        if self.settings.force_dry_run or self.controller.dry_run or self.dry_run:
            result = JogResult(jid, requested, applied, "DRY RUN / NOT SENT", False, False)
        elif not self.controller.is_connected:
            result = JogResult(
                jid,
                requested,
                applied,
                "SERIAL NOT CONNECTED / NOT SENT",
                False,
                False,
            )
        else:
            # OrderedDict holds at most one latest value for each of five axes.
            # Repeated clicks on one joint coalesce instead of building commands.
            duration = self.settings.live_jog_time_ms if time_ms is None else int(time_ms)
            if not 100 <= duration <= 9999:
                raise ValueError("joint movement time outside protocol range")
            self._pending[jid] = (applied, duration)
            self._pending.move_to_end(jid)
            result = JogResult(jid, requested, applied, "QUEUED", True, False)
        self.last_jog_result = result
        LOGGER.info(
            "[STAGE7][JOG_REQUEST] joint=%03d requested=%d applied=%d status=%s pending=%d",
            jid,
            requested,
            applied,
            result.status,
            len(self._pending),
        )
        return result

    def flush_one_jog(self) -> JogResult | None:
        if not self._pending:
            return None
        jid, (pwm, duration) = self._pending.popitem(last=False)
        if self.settings.force_dry_run or self.controller.dry_run or self.dry_run:
            result = JogResult(jid, pwm, pwm, "DRY RUN / NOT SENT", False, False)
        elif not self.controller.is_connected:
            result = JogResult(
                jid, pwm, pwm, "SERIAL NOT CONNECTED / NOT SENT", False, False
            )
        else:
            command = self.controller.send_joint_pwm(
                jid,
                pwm,
                time_ms=duration,
            )
            result = JogResult(jid, pwm, pwm, "SENT", False, True, command)
        self.last_jog_result = result
        LOGGER.info(
            "[STAGE7][JOG_FLUSH] joint=%03d pwm=%d status=%s command=%s",
            jid,
            pwm,
            result.status,
            result.command or "-",
        )
        return result

    def clear_pending_jogs(self) -> None:
        self._pending.clear()

    @property
    def pending_jog_count(self) -> int:
        return len(self._pending)
