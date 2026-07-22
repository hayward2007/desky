"""Entry point: launches the Flask control dashboard (see webapp/app.py).

Run:
    python main.py
Then open https://localhost:8000 in a browser (self-signed cert — see below),
or https://<this machine's LAN IP>:8000/mobile on the phone mounted on the
arm to stream its camera. Works with or without real hardware connected —
webapp/app.py falls back to a "no hardware connected" state if the arm isn't
reachable. Also opens a local cv2 preview window of the phone's camera feed
(see webapp.app.run for why) — press 'q' in that window or Ctrl+C here to quit.
"""

from logger import Logger
from src.app import run

Logger.enabled = True  # attach logging for this run; set False to silence

if __name__ == "__main__":
    run()