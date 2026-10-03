#!/usr/bin/env bash
set -euo pipefail

TLS_DIR=/home/al/.config/tractor-wifi-control/tls
CA_KEY="$TLS_DIR/root-ca.key"
CA_CERT="$TLS_DIR/tractor-wifi-control-root-ca.crt"
SERVER_KEY="$TLS_DIR/server.key"
SERVER_CERT="$TLS_DIR/server.crt"
SERVER_CSR="$TLS_DIR/server.csr"
EXT_FILE="$TLS_DIR/server.ext"

if [[ -e "$CA_KEY" || -e "$CA_CERT" || -e "$SERVER_KEY" || -e "$SERVER_CERT" ]]; then
    echo "Refusing to overwrite existing TLS files in $TLS_DIR" >&2
    exit 1
fi

install -d -m 700 "$TLS_DIR"
openssl genrsa -out "$CA_KEY" 3072
openssl req -x509 -new -sha256 -days 3650 \
    -key "$CA_KEY" \
    -out "$CA_CERT" \
    -subj "/CN=Tractor01 Wi-Fi Control Private CA"

openssl genrsa -out "$SERVER_KEY" 2048
openssl req -new -sha256 \
    -key "$SERVER_KEY" \
    -out "$SERVER_CSR" \
    -subj "/CN=tractor01"

printf '%s\n' \
    'subjectAltName=IP:192.168.193.76,IP:192.168.1.151,DNS:tractor01,DNS:raspberrypi,DNS:tractor01.local,DNS:raspberrypi.local' \
    'extendedKeyUsage=serverAuth' \
    'keyUsage=digitalSignature,keyEncipherment' > "$EXT_FILE"

openssl x509 -req -sha256 -days 825 \
    -in "$SERVER_CSR" \
    -CA "$CA_CERT" \
    -CAkey "$CA_KEY" \
    -CAcreateserial \
    -out "$SERVER_CERT" \
    -extfile "$EXT_FILE"

chmod 600 "$CA_KEY" "$SERVER_KEY"
chmod 644 "$CA_CERT" "$SERVER_CERT"
rm -f "$SERVER_CSR" "$EXT_FILE" "$TLS_DIR/tractor-wifi-control-root-ca.srl"

echo "Created private HTTPS files in $TLS_DIR"
echo "Install this public CA certificate on the field phone:"
echo "$CA_CERT"
echo "Never copy root-ca.key or server.key off Tractor01."
