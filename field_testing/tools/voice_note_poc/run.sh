#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/tractor-voice-notes"
ENV_FILE="$CONFIG_DIR/voice-note.env"
VENV="$HERE/.venv"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Missing $ENV_FILE"
  echo "Copy .env.example there, then edit it as described in README.md."
  exit 1
fi

set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a

if [[ ! -x "$VENV/bin/python" ]]; then
  python3 -m venv "$VENV"
  "$VENV/bin/python" -m pip install --upgrade pip
  "$VENV/bin/python" -m pip install -r "$HERE/requirements.txt"
fi

: "${VOICE_NOTE_BIND_HOST:=192.168.193.217}"
: "${VOICE_NOTE_PORT:=8443}"
: "${VOICE_NOTE_TLS_CERT:=$CONFIG_DIR/tls/server.crt}"
: "${VOICE_NOTE_TLS_KEY:=$CONFIG_DIR/tls/server.key}"

if [[ ! -r "$VOICE_NOTE_TLS_CERT" || ! -r "$VOICE_NOTE_TLS_KEY" ]]; then
  echo "HTTPS certificate or key is missing. Run: ./setup_private_https.sh"
  exit 1
fi

cd "$HERE"
exec "$VENV/bin/python" -m uvicorn app:app \
  --host "$VOICE_NOTE_BIND_HOST" \
  --port "$VOICE_NOTE_PORT" \
  --ssl-certfile "$VOICE_NOTE_TLS_CERT" \
  --ssl-keyfile "$VOICE_NOTE_TLS_KEY" \
  --no-server-header
