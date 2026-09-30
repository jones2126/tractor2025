from __future__ import annotations

import csv
import json
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from core import (
    AcceptedPhrase,
    DuplicateGuard,
    NoteLogger,
    TranscriptDeltaFilter,
    WakePhraseDetector,
)


class WakePhraseDetectorTests(unittest.TestCase):
    def test_ordinary_speech_is_ignored(self) -> None:
        detector = WakePhraseDetector()
        self.assertIsNone(detector.feed("move one foot left", now=1))

    def test_single_final_result_is_accepted(self) -> None:
        accepted = WakePhraseDetector().feed("Tractor note, move one foot left.", now=1)
        self.assertIsNotNone(accepted)
        self.assertEqual(accepted.note_text, "move one foot left")

    def test_note_can_follow_wake_in_next_result(self) -> None:
        detector = WakePhraseDetector()
        self.assertIsNone(detector.feed("tractor note", now=1))
        accepted = detector.feed("clearance is too close", now=2)
        self.assertEqual(accepted.note_text, "clearance is too close")
        self.assertEqual(accepted.full_phrase, "tractor note clearance is too close")

    def test_wake_itself_can_split_across_results(self) -> None:
        detector = WakePhraseDetector()
        self.assertIsNone(detector.feed("tractor", now=1))
        accepted = detector.feed("note move one foot left", now=2)
        self.assertEqual(accepted.note_text, "move one foot left")

    def test_pending_wake_expires(self) -> None:
        detector = WakePhraseDetector(pending_seconds=3)
        self.assertIsNone(detector.feed("tractor note", now=1))
        self.assertIsNone(detector.feed("ordinary speech", now=5))


class PersistenceTests(unittest.TestCase):
    def test_csv_and_jsonl_contain_matching_record(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            logger = NoteLogger(Path(directory))
            when = datetime(2026, 9, 29, 17, 30, tzinfo=timezone.utc)
            record = logger.append(
                AcceptedPhrase("Tractor note move one foot left", "move one foot left"),
                client_timestamp="2026-09-29T17:29:59.000Z",
                seconds_since_session_start=12.3456,
                browser_user_agent="Firefox Android",
                session_id="session-1",
                connection_id="connection-1",
                confidence=0.91,
                audio_mime_type="audio/webm;codecs=opus",
                now=when,
            )
            csv_path = Path(directory) / "voice_notes_20260929.csv"
            jsonl_path = Path(directory) / "voice_notes_20260929.jsonl"
            with csv_path.open(newline="", encoding="utf-8") as handle:
                csv_record = next(csv.DictReader(handle))
            json_record = json.loads(jsonl_path.read_text(encoding="utf-8"))
            self.assertEqual(csv_record["note_id"], record.note_id)
            self.assertEqual(json_record["note_id"], record.note_id)
            self.assertEqual(json_record["note_text"], "move one foot left")
            self.assertIn("-04:00", json_record["server_timestamp_america_new_york"])

    def test_duplicate_guard_is_scoped_to_session_and_time(self) -> None:
        guard = DuplicateGuard(window_seconds=10)
        self.assertFalse(guard.is_duplicate("a", "Move one foot left!", now=1))
        self.assertTrue(guard.is_duplicate("a", "move one foot left", now=2))
        self.assertFalse(guard.is_duplicate("b", "move one foot left", now=2))
        self.assertFalse(guard.is_duplicate("a", "move one foot left", now=20))


class TranscriptDeltaFilterTests(unittest.TestCase):
    def test_cumulative_result_returns_only_new_words(self) -> None:
        transcript_filter = TranscriptDeltaFilter()
        self.assertEqual(
            transcript_filter.feed("Tractor note move. One foot left."),
            "Tractor note move. One foot left",
        )
        self.assertEqual(
            transcript_filter.feed("Tractor note move. One foot left. Talking to my robot."),
            "Talking to my robot",
        )

    def test_separate_note_is_not_stripped(self) -> None:
        transcript_filter = TranscriptDeltaFilter()
        transcript_filter.feed("Tractor note too close.")
        self.assertEqual(
            transcript_filter.feed("Tractor note, move back."),
            "Tractor note, move back",
        )

    def test_exact_repeated_result_is_empty(self) -> None:
        transcript_filter = TranscriptDeltaFilter()
        transcript_filter.feed("Tractor note too close.")
        self.assertEqual(transcript_filter.feed("Tractor note too close."), "")


if __name__ == "__main__":
    unittest.main()
