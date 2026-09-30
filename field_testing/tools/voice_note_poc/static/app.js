"use strict";

const ui = {
  panel: document.getElementById("statusPanel"),
  status: document.getElementById("statusText"),
  start: document.getElementById("startButton"),
  stop: document.getElementById("stopButton"),
  count: document.getElementById("noteCount"),
  latest: document.getElementById("latestNote"),
  connection: document.getElementById("connectionState"),
  format: document.getElementById("audioFormat"),
  lastHeard: document.getElementById("lastHeard"),
};

const state = {
  wanted: false,
  stream: null,
  recorder: null,
  socket: null,
  wakeLock: null,
  reconnectTimer: null,
  restartTimer: null,
  sessionId: "",
  sessionStartedAt: 0,
  noteCount: 0,
  mimeType: "",
  maxStreamSeconds: 270,
  reconnectDelay: 1000,
  generation: 0,
  streamEndResolver: null,
};

function setStatus(text, kind = "idle") {
  ui.status.textContent = text;
  ui.panel.className = `status-panel ${kind}`;
}

function chooseMimeType() {
  if (!window.MediaRecorder) return "";
  const choices = ["audio/webm;codecs=opus", "audio/ogg;codecs=opus", "audio/webm", "audio/ogg"];
  return choices.find((type) => MediaRecorder.isTypeSupported(type)) || "";
}

function websocketUrl() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${location.host}/ws/transcribe`;
}

async function holdWakeLock() {
  if (!("wakeLock" in navigator) || document.visibilityState !== "visible") return;
  try { state.wakeLock = await navigator.wakeLock.request("screen"); } catch (_) { /* optional */ }
}

async function releaseWakeLock() {
  if (state.wakeLock) {
    try { await state.wakeLock.release(); } catch (_) { /* already released */ }
    state.wakeLock = null;
  }
}

function confirmationBeep() {
  try {
    const audio = new (window.AudioContext || window.webkitAudioContext)();
    const oscillator = audio.createOscillator();
    const gain = audio.createGain();
    oscillator.frequency.value = 880;
    gain.gain.setValueAtTime(0.0001, audio.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.16, audio.currentTime + 0.02);
    gain.gain.exponentialRampToValueAtTime(0.0001, audio.currentTime + 0.18);
    oscillator.connect(gain).connect(audio.destination);
    oscillator.start();
    oscillator.stop(audio.currentTime + 0.2);
    oscillator.onended = () => audio.close();
  } catch (_) { /* visual confirmation remains */ }
}

function stopRecorder() {
  clearTimeout(state.restartTimer);
  if (state.recorder && state.recorder.state !== "inactive") {
    try { state.recorder.stop(); } catch (_) { /* already stopping */ }
  }
  state.recorder = null;
}

function closeSocket(sendStop = true) {
  if (state.socket && state.socket.readyState === WebSocket.OPEN) {
    if (sendStop) {
      try { state.socket.send(JSON.stringify({ type: "stop" })); } catch (_) { /* disconnected */ }
    }
  }
  if (state.socket && state.socket.readyState < WebSocket.CLOSING) state.socket.close();
  state.socket = null;
}

function scheduleReconnect(generation) {
  if (!state.wanted || generation !== state.generation) return;
  clearTimeout(state.reconnectTimer);
  setStatus("OFFLINE — RETRYING", "error");
  ui.connection.textContent = `Retrying in ${Math.round(state.reconnectDelay / 1000)}s`;
  state.reconnectTimer = setTimeout(() => connect(generation), state.reconnectDelay);
  state.reconnectDelay = Math.min(state.reconnectDelay * 2, 15000);
}

function startRecorder(generation) {
  if (!state.wanted || generation !== state.generation || !state.stream) return;
  stopRecorder();
  const recorder = new MediaRecorder(state.stream, {
    mimeType: state.mimeType,
    audioBitsPerSecond: 32000,
  });
  state.recorder = recorder;
  recorder.ondataavailable = (event) => {
    if (!event.data.size || !state.socket || state.socket.readyState !== WebSocket.OPEN) return;
    const seconds = (Date.now() - state.sessionStartedAt) / 1000;
    state.socket.send(JSON.stringify({
      type: "audio_meta",
      client_timestamp: new Date().toISOString(),
      seconds_since_session_start: seconds,
    }));
    state.socket.send(event.data);
  };
  recorder.onerror = () => {
    setStatus("MICROPHONE ERROR", "error");
    scheduleReconnect(generation);
  };
  recorder.start(250);
  setStatus("LISTENING", "listening");
  // Restart before Google's five-minute stream limit. This also writes a fresh WebM header.
  state.restartTimer = setTimeout(() => {
    if (!state.wanted || generation !== state.generation) return;
    stopRecorder();
    closeSocket();
    connect(generation);
  }, state.maxStreamSeconds * 1000);
}

function handleServerMessage(message, generation) {
  if (generation !== state.generation) return;
  if (message.type === "ready") {
    ui.connection.textContent = "Connected";
    state.reconnectDelay = 1000;
    startRecorder(generation);
  } else if (message.type === "interim") {
    setStatus("HEARING SPEECH", "busy");
    ui.lastHeard.textContent = message.transcript;
  } else if (message.type === "final_transcript") {
    setStatus("TRANSCRIBING", "busy");
    ui.lastHeard.textContent = message.transcript;
    setTimeout(() => state.wanted && setStatus("LISTENING", "listening"), 650);
  } else if (message.type === "note_saved") {
    state.noteCount += 1;
    ui.count.textContent = String(state.noteCount);
    ui.latest.textContent = message.note.note_text;
    ui.lastHeard.textContent = message.note.full_recognized_phrase;
    setStatus("NOTE SAVED", "saved");
    confirmationBeep();
    setTimeout(() => state.wanted && setStatus("LISTENING", "listening"), 1400);
  } else if (message.type === "duplicate_ignored") {
    ui.lastHeard.textContent = `Duplicate ignored: ${message.note_text}`;
    setStatus("LISTENING", "listening");
  } else if (message.type === "error") {
    setStatus(message.message || "TRANSCRIPTION ERROR", "error");
    ui.connection.textContent = "Server error";
  } else if (message.type === "stream_ended") {
    if (state.streamEndResolver) {
      state.streamEndResolver();
      state.streamEndResolver = null;
    }
  }
}

function connect(generation) {
  if (!state.wanted || generation !== state.generation) return;
  closeSocket();
  ui.connection.textContent = "Connecting…";
  const socket = new WebSocket(websocketUrl());
  state.socket = socket;
  socket.onopen = () => socket.send(JSON.stringify({
    type: "start",
    session_id: state.sessionId,
    session_started_at: new Date(state.sessionStartedAt).toISOString(),
    user_agent: navigator.userAgent,
    mime_type: state.mimeType,
  }));
  socket.onmessage = (event) => {
    try { handleServerMessage(JSON.parse(event.data), generation); }
    catch (_) { setStatus("INVALID SERVER RESPONSE", "error"); }
  };
  socket.onerror = () => { ui.connection.textContent = "Connection failed"; };
  socket.onclose = () => {
    stopRecorder();
    if (state.socket === socket) {
      state.socket = null;
      if (state.wanted && generation === state.generation) scheduleReconnect(generation);
    }
  };
}

async function startListening() {
  if (state.wanted) return;
  if (!window.isSecureContext) {
    setStatus("HTTPS REQUIRED", "error");
    return;
  }
  state.mimeType = chooseMimeType();
  if (!navigator.mediaDevices?.getUserMedia || !state.mimeType) {
    setStatus("AUDIO RECORDING UNSUPPORTED", "error");
    return;
  }
  ui.format.textContent = state.mimeType;
  setStatus("REQUESTING MICROPHONE", "busy");
  try {
    state.stream = await navigator.mediaDevices.getUserMedia({
      audio: { channelCount: 1, echoCancellation: true, noiseSuppression: true, autoGainControl: true },
      video: false,
    });
  } catch (error) {
    setStatus(error.name === "NotAllowedError" ? "MICROPHONE BLOCKED" : "MICROPHONE ERROR", "error");
    return;
  }
  state.wanted = true;
  state.generation += 1;
  state.sessionId = crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}-${Math.random()}`;
  state.sessionStartedAt = Date.now();
  state.noteCount = 0;
  state.reconnectDelay = 1000;
  ui.count.textContent = "0";
  ui.start.disabled = true;
  ui.stop.disabled = false;
  await holdWakeLock();
  connect(state.generation);
}

async function stopListening() {
  state.wanted = false;
  clearTimeout(state.reconnectTimer);
  clearTimeout(state.restartTimer);
  ui.stop.disabled = true;
  setStatus("TRANSCRIBING", "busy");

  const recorder = state.recorder;
  state.recorder = null;
  if (recorder && recorder.state !== "inactive") {
    await new Promise((resolve) => {
      const timeout = setTimeout(resolve, 1500);
      recorder.addEventListener("stop", () => { clearTimeout(timeout); resolve(); }, { once: true });
      try { recorder.stop(); } catch (_) { clearTimeout(timeout); resolve(); }
    });
  }

  if (state.socket && state.socket.readyState === WebSocket.OPEN) {
    try { state.socket.send(JSON.stringify({ type: "stop" })); } catch (_) { /* disconnected */ }
    await new Promise((resolve) => {
      const timeout = setTimeout(() => {
        state.streamEndResolver = null;
        resolve();
      }, 8000);
      state.streamEndResolver = () => { clearTimeout(timeout); resolve(); };
    });
  }
  closeSocket(false);
  state.generation += 1;
  if (state.stream) state.stream.getTracks().forEach((track) => track.stop());
  state.stream = null;
  await releaseWakeLock();
  ui.start.disabled = false;
  ui.connection.textContent = "Stopped";
  setStatus("MICROPHONE READY", "idle");
}

async function initialize() {
  state.mimeType = chooseMimeType();
  ui.format.textContent = state.mimeType || "Unsupported";
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    const config = await response.json();
    state.maxStreamSeconds = config.max_stream_seconds || 270;
  } catch (_) {
    setStatus("OFFLINE — RETRYING", "error");
  }
}

ui.start.addEventListener("click", startListening);
ui.stop.addEventListener("click", stopListening);
document.addEventListener("visibilitychange", () => {
  if (state.wanted && document.visibilityState === "visible") holdWakeLock();
});
window.addEventListener("online", () => state.wanted && connect(state.generation));
window.addEventListener("offline", () => state.wanted && setStatus("OFFLINE — RETRYING", "error"));
initialize();
