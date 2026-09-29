"""Standalone phone-to-NAS voice note service. It has no tractor control paths."""

from __future__ import annotations

import asyncio
import json
import logging
import os
import uuid
from contextlib import suppress
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import AsyncIterator
from urllib.parse import urlsplit

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from google.api_core.exceptions import GoogleAPICallError
from google.auth.exceptions import DefaultCredentialsError
from google.cloud import speech_v1 as speech

from core import DuplicateGuard, NoteLogger, WakePhraseDetector


HERE = Path(__file__).resolve().parent
STATIC = HERE / "static"
LOG = logging.getLogger("voice_note_poc")
MAX_AUDIO_MESSAGE_BYTES = 256 * 1024
GOOGLE_FRAME_BYTES = 24_000  # Google documents a 25 KB streaming-message limit.


@dataclass(frozen=True)
class Settings:
    log_dir: Path
    language_code: str
    model: str
    sample_rate_hertz: int
    max_stream_seconds: int
    fake_transcripts: bool

    @classmethod
    def from_environment(cls) -> "Settings":
        return cls(
            log_dir=Path(os.getenv("VOICE_NOTE_LOG_DIR", "/home/al/field_logs/voice_note_poc")),
            language_code=os.getenv("VOICE_NOTE_LANGUAGE", "en-US"),
            model=os.getenv("VOICE_NOTE_MODEL", "latest_short"),
            sample_rate_hertz=int(os.getenv("VOICE_NOTE_SAMPLE_RATE", "48000")),
            max_stream_seconds=min(int(os.getenv("VOICE_NOTE_MAX_STREAM_SECONDS", "270")), 285),
            fake_transcripts=os.getenv("VOICE_NOTE_FAKE_TRANSCRIPTS", "0") == "1",
        )


settings = Settings.from_environment()
note_logger = NoteLogger(settings.log_dir)
duplicates = DuplicateGuard()
app = FastAPI(title="Tractor Voice Note POC", docs_url=None, redoc_url=None)
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})


@app.get("/api/status")
async def status() -> JSONResponse:
    return JSONResponse(
        {
            "ok": True,
            "wake_phrase": "Tractor note",
            "max_stream_seconds": settings.max_stream_seconds,
            "fake_transcripts": settings.fake_transcripts,
        },
        headers={"Cache-Control": "no-store"},
    )


@dataclass
class StreamContext:
    session_id: str = ""
    connection_id: str = ""
    client_timestamp: str = ""
    seconds_since_session_start: float | None = None
    user_agent: str = ""
    mime_type: str = ""


def recognition_config(mime_type: str) -> speech.StreamingRecognitionConfig:
    encoding = (
        speech.RecognitionConfig.AudioEncoding.OGG_OPUS
        if mime_type.startswith("audio/ogg")
        else speech.RecognitionConfig.AudioEncoding.WEBM_OPUS
    )
    config = speech.RecognitionConfig(
        encoding=encoding,
        sample_rate_hertz=settings.sample_rate_hertz,
        language_code=settings.language_code,
        model=settings.model,
        enable_automatic_punctuation=True,
        max_alternatives=1,
        audio_channel_count=1,
        speech_contexts=[speech.SpeechContext(phrases=["Tractor note"], boost=20.0)],
    )
    return speech.StreamingRecognitionConfig(config=config, interim_results=True)


async def websocket_send(websocket: WebSocket, payload: dict) -> None:
    with suppress(WebSocketDisconnect, RuntimeError):
        await websocket.send_json(payload)


async def receive_audio(websocket: WebSocket, queue: asyncio.Queue[bytes | None], context: StreamContext) -> None:
    try:
        while True:
            message = await websocket.receive()
            if message["type"] == "websocket.disconnect":
                break
            if message.get("text") is not None:
                payload = json.loads(message["text"])
                kind = payload.get("type")
                if kind == "audio_meta":
                    context.client_timestamp = str(payload.get("client_timestamp", ""))
                    try:
                        context.seconds_since_session_start = float(payload["seconds_since_session_start"])
                    except (KeyError, TypeError, ValueError):
                        context.seconds_since_session_start = None
                elif kind == "stop":
                    break
            elif message.get("bytes") is not None:
                chunk = message["bytes"]
                if len(chunk) > MAX_AUDIO_MESSAGE_BYTES:
                    await websocket_send(websocket, {"type": "error", "message": "Audio chunk was too large."})
                    break
                if chunk:
                    await queue.put(chunk)
    except (WebSocketDisconnect, json.JSONDecodeError):
        pass
    finally:
        await queue.put(None)


async def google_requests(
    queue: asyncio.Queue[bytes | None], mime_type: str
) -> AsyncIterator[speech.StreamingRecognizeRequest]:
    yield speech.StreamingRecognizeRequest(streaming_config=recognition_config(mime_type))
    while True:
        chunk = await queue.get()
        if chunk is None:
            return
        for offset in range(0, len(chunk), GOOGLE_FRAME_BYTES):
            yield speech.StreamingRecognizeRequest(audio_content=chunk[offset : offset + GOOGLE_FRAME_BYTES])


async def accept_transcript(
    websocket: WebSocket,
    detector: WakePhraseDetector,
    context: StreamContext,
    transcript: str,
    confidence: float | None,
) -> None:
    accepted = detector.feed(transcript)
    if accepted is None:
        await websocket_send(websocket, {"type": "final_transcript", "transcript": transcript})
        return
    if duplicates.is_duplicate(context.session_id, accepted.note_text):
        await websocket_send(websocket, {"type": "duplicate_ignored", "note_text": accepted.note_text})
        return
    record = await asyncio.to_thread(
        note_logger.append,
        accepted,
        client_timestamp=context.client_timestamp,
        seconds_since_session_start=context.seconds_since_session_start,
        browser_user_agent=context.user_agent,
        session_id=context.session_id,
        connection_id=context.connection_id,
        confidence=confidence,
        audio_mime_type=context.mime_type,
    )
    await websocket_send(websocket, {"type": "note_saved", "note": asdict(record)})


async def run_google_stream(websocket: WebSocket, queue: asyncio.Queue[bytes | None], context: StreamContext) -> None:
    detector = WakePhraseDetector()
    client = speech.SpeechAsyncClient()
    responses = await client.streaming_recognize(requests=google_requests(queue, context.mime_type))
    async for response in responses:
        for result in response.results:
            if not result.alternatives:
                continue
            alternative = result.alternatives[0]
            transcript = alternative.transcript.strip()
            if not transcript:
                continue
            if result.is_final:
                confidence = float(alternative.confidence) if alternative.confidence else None
                await accept_transcript(websocket, detector, context, transcript, confidence)
            else:
                await websocket_send(websocket, {"type": "interim", "transcript": transcript})


async def run_fake_stream(websocket: WebSocket, queue: asyncio.Queue[bytes | None], context: StreamContext) -> None:
    """Local UI plumbing test: audio bytes are discarded; typed transcripts are handled elsewhere."""
    while await queue.get() is not None:
        pass


@app.websocket("/ws/transcribe")
async def transcribe(websocket: WebSocket) -> None:
    # Browsers set Origin on WebSocket handshakes. Requiring this service's own
    # host prevents an unrelated web page from using the private NAS as a
    # transcription relay while the phone is connected to ZeroTier.
    origin = websocket.headers.get("origin", "")
    host = websocket.headers.get("host", "")
    if not origin or urlsplit(origin).netloc.casefold() != host.casefold():
        await websocket.close(code=1008, reason="Cross-origin connection rejected")
        return
    await websocket.accept()
    queue: asyncio.Queue[bytes | None] = asyncio.Queue(maxsize=80)
    context = StreamContext(connection_id=uuid.uuid4().hex)
    try:
        start = await asyncio.wait_for(websocket.receive_json(), timeout=10)
        if start.get("type") != "start":
            await websocket.close(code=1008, reason="Expected start metadata")
            return
        context.session_id = str(start.get("session_id", ""))[:100] or uuid.uuid4().hex
        context.user_agent = str(start.get("user_agent", ""))[:1000]
        context.mime_type = str(start.get("mime_type", ""))[:100]
        if not context.mime_type.startswith(("audio/webm", "audio/ogg")):
            await websocket_send(websocket, {"type": "error", "message": "This test requires WebM/Opus or Ogg/Opus audio."})
            await websocket.close(code=1003)
            return
        await websocket_send(websocket, {"type": "ready", "connection_id": context.connection_id})
        receiver = asyncio.create_task(receive_audio(websocket, queue, context))
        try:
            if settings.fake_transcripts:
                await run_fake_stream(websocket, queue, context)
            else:
                await run_google_stream(websocket, queue, context)
        finally:
            if not receiver.done():
                receiver.cancel()
            with suppress(asyncio.CancelledError):
                await receiver
        await websocket_send(websocket, {"type": "stream_ended"})
    except asyncio.TimeoutError:
        await websocket.close(code=1008, reason="Start metadata timed out")
    except DefaultCredentialsError:
        await websocket_send(websocket, {"type": "error", "message": "Google credentials are not configured on the NAS."})
    except GoogleAPICallError as exc:
        LOG.warning("Google Speech stream failed: %s", exc)
        await websocket_send(websocket, {"type": "error", "message": "Google transcription is temporarily unavailable."})
    except WebSocketDisconnect:
        pass
    except Exception:
        LOG.exception("Unexpected transcription stream failure")
        await websocket_send(websocket, {"type": "error", "message": "The transcription stream stopped unexpectedly."})
    finally:
        with suppress(RuntimeError):
            await websocket.close()


if settings.fake_transcripts:
    @app.post("/api/fake-transcript")
    async def fake_transcript(payload: dict) -> JSONResponse:
        """Development-only endpoint, absent unless VOICE_NOTE_FAKE_TRANSCRIPTS=1."""
        detector = WakePhraseDetector()
        accepted = detector.feed(str(payload.get("transcript", "")))
        if accepted is None:
            return JSONResponse({"saved": False})
        context = StreamContext(
            session_id=str(payload.get("session_id", "fake-session")),
            connection_id="fake-connection",
            client_timestamp=str(payload.get("client_timestamp", "")),
            seconds_since_session_start=float(payload.get("seconds_since_session_start", 0)),
            user_agent=str(payload.get("user_agent", "fake-browser")),
            mime_type="audio/webm;codecs=opus",
        )
        record = note_logger.append(
            accepted,
            client_timestamp=context.client_timestamp,
            seconds_since_session_start=context.seconds_since_session_start,
            browser_user_agent=context.user_agent,
            session_id=context.session_id,
            connection_id=context.connection_id,
            confidence=None,
            audio_mime_type=context.mime_type,
        )
        return JSONResponse({"saved": True, "note": asdict(record)})
