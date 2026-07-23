"""Project-wide console logger. Format: [TAG] message.

Usage:
    from fundamental.logger import Logger
    Logger.log("CONTROLLER", "Succeeded to open the port")
    # -> [CONTROLLER] Succeeded to open the port

`Logger.enabled` is a single switch shared by every module (controller, util,
kinematics, urdf_loader, main) — flip it once from main.py to silence all
logging for a run.
"""


class Logger:
    enabled = True

    @classmethod
    def log(cls, tag: str, message: str) -> None:
        if cls.enabled:
            print(f"[{tag}] {message}")
