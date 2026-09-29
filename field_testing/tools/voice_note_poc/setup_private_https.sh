#!/usr/bin/env bash
set -euo pipefail

CONFIG_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/tractor-voice-notes"
TLS_DIR="$CONFIG_DIR/tls"
mkdir -p "$TLS_DIR"
chmod 700 "$CONFIG_DIR" "$TLS_DIR"

if [[ -e "$TLS_DIR/root-ca.key" || -e "$TLS_DIR/server.key" ]]; then
  echo "TLS keys already exist in $TLS_DIR; nothing was overwritten."
  echo "Move that directory aside yourself if you intentionally want a new CA."
  exit 1
fi

openssl genrsa -out "$TLS_DIR/root-ca.key" 4096
chmod 600 "$TLS_DIR/root-ca.key"
openssl req -x509 -new -sha256 -days 1825 \
  -key "$TLS_DIR/root-ca.key" \
  -out "$TLS_DIR/tractor-voice-root-ca.crt" \
  -subj "/CN=Al Tractor Voice Notes Private CA/O=Private Tractor Test"

openssl genrsa -out "$TLS_DIR/server.key" 3072
chmod 600 "$TLS_DIR/server.key"
openssl req -new -sha256 \
  -key "$TLS_DIR/server.key" \
  -out "$TLS_DIR/server.csr" \
  -subj "/CN=RPi5NAS"

EXT_FILE="$(mktemp)"
trap 'rm -f "$EXT_FILE"' EXIT
printf '%s\n' \
  'basicConstraints=critical,CA:FALSE' \
  'keyUsage=critical,digitalSignature,keyEncipherment' \
  'extendedKeyUsage=serverAuth' \
  'subjectAltName=DNS:RPi5NAS,DNS:rpi5nas,DNS:rpi5nas.local,IP:192.168.193.217,IP:192.168.1.2,IP:192.168.1.205' \
  > "$EXT_FILE"

openssl x509 -req -sha256 -days 825 \
  -in "$TLS_DIR/server.csr" \
  -CA "$TLS_DIR/tractor-voice-root-ca.crt" \
  -CAkey "$TLS_DIR/root-ca.key" \
  -CAcreateserial \
  -extfile "$EXT_FILE" \
  -out "$TLS_DIR/server.crt"
rm -f "$TLS_DIR/server.csr" "$TLS_DIR/root-ca.srl"

echo "Created private HTTPS files in $TLS_DIR"
echo "Install ONLY this public root certificate on the phone:"
echo "  $TLS_DIR/tractor-voice-root-ca.crt"
echo "Never copy root-ca.key or server.key off the NAS."
