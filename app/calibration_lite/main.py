from __future__ import annotations

import argparse
import logging
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from app.config import AppConfig
from app.logging_config import configure_logging

from .window import CalibrationLiteWindow


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="J1 Gomoku Calibration Lite")
    parser.add_argument(
        "--test-pattern",
        action="store_true",
        help="Use the generated camera test image after clicking Connect Camera.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Use the simulated robot controller; no serial commands are sent.",
    )
    parser.add_argument("--smoke-test", action="store_true", help=argparse.SUPPRESS)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = AppConfig.load()
    session_log = configure_logging(config.logs_dir)
    logging.getLogger(__name__).info("CALIBRATION LITE START session=%s", session_log)
    app = QApplication(sys.argv[:1])
    app.setApplicationName("J1 Gomoku Calibration Lite")
    window = CalibrationLiteWindow(
        config,
        dry_run=args.dry_run,
        default_test_pattern=args.test_pattern,
    )
    window.show()
    if args.smoke_test:
        QTimer.singleShot(1200, window.close)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
