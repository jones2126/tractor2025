"""Wake phrase detection and durable note logging for the voice-note POC."""

from __future__ import annotations

import csv
import json
import os
import re
import threading
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from time import monotonic
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError


WAKE_PATTERN = re.compile(r"\btractor\s+note\b[\s,:;.!?-]*(.*)$", re.IGNORECASE)
PARTIAL_WAKE_PATTERN = re.compile(r"\btractor\s*$", re.IGNORECASE)
SPACE_PATTERN = re.compile(r"\s+")


def new_york_time(value: datetime) -> datetime:
    """Use system tzdata when present, with a post-2007 US DST fallback."""
    try:
        return value.astimezone(ZoneInfo("America/New_York"))
    except ZoneInfoNotFoundError:
        # DST starts at 07:00 UTC on the second Sunday in March and ends at
        # 06:00 UTC on the first Sunday in November under current US rules.
        year = value.year
        march_1 = datetime(year, 3, 1, tzinfo=timezone.utc)
        second_sunday = 1 + ((6 - march_1.weekday()) % 7) + 7
        dst_start = datetime(year, 3, second_sunday, 7, tzinfo=timezone.utc)
        november_1 = datetime(year, 11, 1, tzinfo=timezone.utc)
        first_sunday = 1 + ((6 - november_1.weekday()) % 7)
        dst_end = datetime(year, 11, first_sunday, 6, tzinfo=timezone.utc)
        offset = -4 if dst_start <= value.astimezone(timezone.utc) < dst_end else -5
        return value.astimezone(timezone(timedelta(hours=offset), name="America/New_York"))


def clean_text(value: str) -> str:
    return SPACE_PATTERN.sub(" ", value).strip()


@dataclass(frozen=True)
class AcceptedPhrase:
    full_phrase: str
    note_text: str


class WakePhraseDetector:
    """Recognize a wake phrase even when Google splits adjacent final results."""

    def __init__(self, pending_seconds: float = 8.0) -> None:
        self.pending_seconds = pending_seconds
        self._pending_prefix = ""
        self._pending_until = 0.0
        self._previous_fragment = ""

    def feed(self, fragment: str, now: float | None = None) -> AcceptedPhrase | None:
        now = monotonic() if now is None else now
        fragment = clean_text(fragment)
        if not fragment:
            return None

        if self._pending_prefix:
            if now <= self._pending_until:
                full_phrase = clean_text(f"{self._pending_prefix} {fragment}")
                self._clear_pending()
                self._previous_fragment = fragment
                return AcceptedPhrase(full_phrase=full_phrase, note_text=fragment.strip(" ,:;.!?-"))
            self._previous_fragment = ""

        self._clear_pending()
        candidates = [fragment]
        if self._previous_fragment:
            candidates.append(clean_text(f"{self._previous_fragment} {fragment}"))

        for candidate in candidates:
            match = WAKE_PATTERN.search(candidate)
            if not match:
                continue
            note_text = clean_text(match.group(1)).strip(" ,:;.!?-")
            wake_start = match.start()
            full_phrase = candidate[wake_start:]
            if note_text:
                self._previous_fragment = fragment
                return AcceptedPhrase(full_phrase=full_phrase, note_text=note_text)
            self._pending_prefix = full_phrase
            self._pending_until = now + self.pending_seconds
            self._previous_fragment = fragment
            return None

        # Retain only a small fragment. It is enough to bridge "tractor" / "note".
        self._previous_fragment = fragment if PARTIAL_WAKE_PATTERN.search(fragment) else ""
        return None

    def _clear_pending(self) -> None:
        self._pending_prefix = ""
        self._pending_until = 0.0


CSV_FIELDS = [
    "note_id",
    "server_timestamp_utc",
    "server_timestamp_america_new_york",
    "client_timestamp",
    "seconds_since_session_start",
    "full_recognized_phrase",
    "note_text",
    "browser_user_agent",
    "session_id",
    "connection_id",
    "transcription_confidence",
    "audio_mime_type",
]


@dataclass(frozen=True)
class NoteRecord:
    note_id: str
    server_timestamp_utc: str
    server_timestamp_america_new_york: str
    client_timestamp: str
    seconds_since_session_start: float | None
    full_recognized_phrase: str
    note_text: str
    browser_user_agent: str
    session_id: str
    connection_id: str
    transcription_confidence: float | None
    audio_mime_type: str


class NoteLogger:
    """Append accepted notes to same-named daily CSV and JSONL files."""

    def __init__(self, log_dir: Path) -> None:
        self.log_dir = log_dir
        self._lock = threading.Lock()

    def append(
        self,
        accepted: AcceptedPhrase,
        *,
        client_timestamp: str,
        seconds_since_session_start: float | None,
        browser_user_agent: str,
        session_id: str,
        connection_id: str,
        confidence: float | None,
        audio_mime_type: str,
        now: datetime | None = None,
    ) -> NoteRecord:
        now = now or datetime.now(timezone.utc)
        if now.tzinfo is None:
            raise ValueError("now must be timezone-aware")
        now_utc = now.astimezone(timezone.utc)
        record = NoteRecord(
            note_id=uuid.uuid4().hex,
            server_timestamp_utc=now_utc.isoformat(timespec="milliseconds"),
            server_timestamp_america_new_york=new_york_time(now_utc).isoformat(timespec="milliseconds"),
            client_timestamp=clean_text(client_timestamp)[:80],
            seconds_since_session_start=(
                round(float(seconds_since_session_start), 3)
                if seconds_since_session_start is not None
                else None
            ),
            full_recognized_phrase=clean_text(accepted.full_phrase)[:1000],
            note_text=clean_text(accepted.note_text)[:800],
            browser_user_agent=clean_text(browser_user_agent)[:1000],
            session_id=clean_text(session_id)[:100],
            connection_id=clean_text(connection_id)[:100],
            transcription_confidence=(round(float(confidence), 6) if confidence is not None else None),
            audio_mime_type=clean_text(audio_mime_type)[:100],
        )
        day = now_utc.strftime("%Y%m%d")
        csv_path = self.log_dir / f"voice_notes_{day}.csv"
        jsonl_path = self.log_dir / f"voice_notes_{day}.jsonl"
        payload = asdict(record)

        with self._lock:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            new_csv = not csv_path.exists()
            with csv_path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
                if new_csv:
                    writer.writeheader()
                writer.writerow(payload)
                handle.flush()
                os.fsync(handle.fileno())
            with jsonl_path.open("a", encoding="utf-8", newline="\n") as handle:
                handle.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")
                handle.flush()
                os.fsync(handle.fileno())
        return record


class DuplicateGuard:
    """Drop a repeated note produced by overlapping/replayed recognition results."""

    def __init__(self, window_seconds: float = 15.0) -> None:
        self.window_seconds = window_seconds
        self._seen: dict[tuple[str, str], float] = {}

    def is_duplicate(self, session_id: str, note_text: str, now: float | None = None) -> bool:
        now = monotonic() if now is None else now
        normalized = re.sub(r"[^a-z0-9]+", " ", note_text.casefold()).strip()
        key = (session_id, normalized)
        cutoff = now - self.window_seconds
        self._seen = {item: timestamp for item, timestamp in self._seen.items() if timestamp >= cutoff}
        if key in self._seen:
            return True
        self._seen[key] = now
        return False
