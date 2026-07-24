"""desky 진입점 — 통합 앱(팔 추적 + 웹 기능)을 실행한다.

실행:
    python main.py

그다음:
  · PC 브라우저          https://localhost:8000          제어 대시보드
  · 팔에 장착한 휴대폰    https://<이 PC의 LAN IP>:8000/mobile
자체 서명 인증서라 브라우저가 경고를 한 번 띄운다 — 허용하고 진행하면 된다
(카메라/마이크는 HTTPS에서만 열린다).

실제 하드웨어가 연결돼 있든 아니든 똑같이 뜬다 — 팔이 없으면 "no hardware
connected" 상태로 두고 FK/IK·3D 미리보기·웹 기능은 그대로 동작한다.
로컬에는 휴대폰 카메라 화면과 3D 씬 미리보기 창이 함께 열린다. 종료는 그 창에서
'q'(또는 Esc), 아니면 여기서 Ctrl+C.
"""

from fundamental.logger import Logger
from src.app import run

Logger.enabled = True  # 이번 실행의 로그를 켠다; False로 두면 전부 조용해진다

if __name__ == "__main__":
    run()
