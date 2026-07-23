"""일정(캘린더) 저장소 — /api/calendar/* 라우트를 담당.

Gemini/하드웨어와 같은 "기능 객체 하나" 패턴을 따른다. 다만 외부 의존성이
전혀 없어(표준 라이브러리 json/datetime만 사용) 항상 `configured=True`다.

설계상 이 서버는 **순수 저장소**다. "오늘/내일/이번 주 화요일" 같은 상대 날짜를
YYYY-MM-DD로 바꾸는 일은 전부 폰(브라우저)에서 그 기기의 로컬 시계로 처리하고,
서버에는 이미 확정된 날짜 문자열만 넘어온다. 이렇게 하면 서버와 폰의 시간대/
시계가 달라도 "오늘"이 어긋나지 않는다.

이벤트 한 건의 형태:
    {"id": 3, "date": "2026-03-05", "time": "15:00"|None, "title": "치과 예약"}

이벤트는 프로젝트 루트의 `calendar_events.json`에 저장돼 페이지를 새로고침하거나
서버를 재시작해도 유지된다. Flask가 threaded=True로 도므로 파일 접근은 Lock으로
직렬화하고, 쓰기는 임시 파일 → rename으로 원자적으로 처리한다.
"""

import json
import os
import threading
from datetime import datetime

from flask import jsonify, request

from fundamental.const import CalendarConst
from fundamental.logger import Logger

# src/api/calendar.py 기준 세 단계 위 = 저장소 루트. 파일 이름 설명은
# fundamental.const.CalendarConst 참고.
_DEFAULT_STORE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    CalendarConst.STORE_FILENAME,
)


def _valid_date(s):
    """'YYYY-MM-DD'면 그대로 돌려주고, 아니면 None."""
    if not isinstance(s, str):
        return None
    try:
        return datetime.strptime(s, CalendarConst.DATE_FORMAT).strftime(CalendarConst.DATE_FORMAT)
    except ValueError:
        return None


def _valid_time(s):
    """'HH:MM'이면 정규화해 돌려주고, 비었으면 None, 형식이 틀리면 None."""
    if not s:
        return None
    if not isinstance(s, str):
        return None
    try:
        return datetime.strptime(s.strip(), CalendarConst.TIME_FORMAT).strftime(CalendarConst.TIME_FORMAT)
    except ValueError:
        return None


class Calendar:
    """일정 목록 하나를 감싸고, 조회/추가/삭제 라우트를 모아 둔 객체.

    라우트 등록은 `src.app.DeskyApp._register_routes()`가 `register()`를 불러
    한 곳에서 처리한다 — Gemini/Camera/ScanAPI/Light와 같은 관례.
    """

    def __init__(self, path=_DEFAULT_STORE_PATH):
        self.path = path
        self._lock = threading.Lock()
        self._events = []
        self._next_id = 1
        self._load()

    # ------------------------------------------------------------------
    # 영속화 (호출부에서 self._lock을 이미 잡은 상태로 부른다)
    # ------------------------------------------------------------------
    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._events = data.get("events", [])
            self._next_id = data.get("next_id", 1)
            Logger.log("CALENDAR", f"{len(self._events)}개 일정 로드 ({self.path})")
        except FileNotFoundError:
            self._events = []
            self._next_id = 1
        except Exception as e:
            # 손상된 파일 때문에 앱이 죽지는 않게 — 빈 상태로 시작한다.
            Logger.log("CALENDAR", f"일정 파일을 읽지 못해 빈 상태로 시작: {e}")
            self._events = []
            self._next_id = 1

    def _save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"events": self._events, "next_id": self._next_id},
                      f, ensure_ascii=False, indent=2)
        os.replace(tmp, self.path)  # 원자적 교체

    @staticmethod
    def _sort_key(ev):
        # 날짜 오름차순, 같은 날은 시간 있는 것을 먼저(시간순), 시간 없으면 맨 뒤.
        return (ev["date"], ev.get("time") or CalendarConst.NO_TIME_SORT_KEY)

    # ------------------------------------------------------------------
    # 라우트
    # ------------------------------------------------------------------
    def list_events(self):
        """GET /api/calendar/events

        쿼리 파라미터(모두 선택):
          ?date=YYYY-MM-DD          해당 날짜만
          ?from=YYYY-MM-DD&to=...   기간(양끝 포함)
        아무것도 없으면 전체를 날짜순으로 돌려준다.
        """
        date = _valid_date(request.args.get("date"))
        d_from = _valid_date(request.args.get("from"))
        d_to = _valid_date(request.args.get("to"))

        with self._lock:
            events = list(self._events)

        if date:
            events = [e for e in events if e["date"] == date]
        else:
            if d_from:
                events = [e for e in events if e["date"] >= d_from]
            if d_to:
                events = [e for e in events if e["date"] <= d_to]

        events.sort(key=self._sort_key)
        return jsonify({"events": events})

    def add_event(self):
        """POST /api/calendar/events — body {date, title, time?}."""
        data = request.get_json(force=True, silent=True) or {}
        date = _valid_date(data.get("date"))
        title = (data.get("title") or "").strip()
        time = _valid_time(data.get("time"))

        if not date:
            return jsonify({"error": "date는 YYYY-MM-DD 형식이어야 합니다"}), 400
        if not title:
            return jsonify({"error": "title이 필요합니다"}), 400

        with self._lock:
            event = {"id": self._next_id, "date": date, "time": time, "title": title}
            self._next_id += 1
            self._events.append(event)
            self._save()

        Logger.log("CALENDAR", f"추가: {date} {time or ''} {title}")
        return jsonify({"event": event})

    def delete_event(self):
        """POST /api/calendar/delete — 아래 셋 중 하나로 삭제.

          {id: 3}                  그 id 하나
          {date, query?}           그 날짜에서 title에 query가 든 것(query 없으면 그 날 전부)
          {query}                  전체에서 title에 query가 든 것

        지운 이벤트 목록을 돌려준다(음성 안내에 쓴다).
        """
        data = request.get_json(force=True, silent=True) or {}
        ev_id = data.get("id")
        date = _valid_date(data.get("date"))
        query = (data.get("query") or "").strip()

        with self._lock:
            if ev_id is not None:
                matched = [e for e in self._events if e["id"] == ev_id]
            elif date:
                matched = [e for e in self._events if e["date"] == date
                           and (not query or query in e["title"])]
            elif query:
                matched = [e for e in self._events if query in e["title"]]
            else:
                return jsonify({"error": "id, date, query 중 하나가 필요합니다"}), 400

            if matched:
                remove_ids = {e["id"] for e in matched}
                self._events = [e for e in self._events if e["id"] not in remove_ids]
                self._save()

        matched.sort(key=self._sort_key)
        Logger.log("CALENDAR", f"삭제: {len(matched)}건")
        return jsonify({"removed": matched, "count": len(matched)})

    def register(self, app):
        """이 객체가 담당하는 라우트를 Flask 앱에 붙인다.

        GET  /api/calendar/events   list_events (?date= | ?from=&to=)
        POST /api/calendar/events   add_event   {date, title, time?}
        POST /api/calendar/delete   delete_event{id | date+query? | query}
        """
        app.route("/api/calendar/events", endpoint="calendar_list")(self.list_events)
        app.route("/api/calendar/events", methods=["POST"], endpoint="calendar_add")(self.add_event)
        app.route("/api/calendar/delete", methods=["POST"], endpoint="calendar_delete")(self.delete_event)
