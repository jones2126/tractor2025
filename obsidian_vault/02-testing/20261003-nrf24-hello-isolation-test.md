# 2026-10-03 NRF24 hello isolation test

## Purpose

Test only the two NRF24 modules, their SPI connections, and the bidirectional
hardware acknowledgement. The handheld sends `HELLO` plus an incrementing
32-bit counter once per second. The tractor prints each received packet. The
handheld prints `ACK` only when the tractor NRF24 acknowledges the packet.

This test does not use the production `1Node`/`2Node` addresses or the
production 14-byte control structure. Both test sketches use address `HELLO`,
channel 76, 250 KBPS, a fixed 16-byte payload, auto acknowledgement, and low PA
power. Keep the two radios approximately 6 to 10 feet apart.
The PlatformIO test configurations pin both boards to RF24 library 1.6.2 so
the library version is not another test variable.

## Safety state

- Engine off and blades disengaged.
- Tractor secured against movement.
- Steering and transmission motor power disabled if independently possible.
- The tractor test firmware has no production actuator, radio-safety, GPS, or
  serial-bridge control logic. Do not operate or drive the tractor with it.
- Stop `teensy-bridge.service` before uploading the tractor test and leave it
  stopped until production tractor firmware has been restored.

## Expected result

Handheld, once per second:

```text
TX counter=12 message=HELLO result=ACK
```

Tractor, once per received packet:

```text
RX counter=12 message=HELLO status=OK
```

Continuous `NO_ACK` on the handheld and no tractor receive lines means the
basic RF exchange is failing. Intermittent counter jumps identify packet loss.

If the forward test fails, reverse the roles with the separately named reverse
test files. The tractor then transmits and the handheld receives. A successful
reverse test proves both modules can transmit, receive, and acknowledge. If
both directions fail, these two radios alone cannot identify which module is
faulty; a known-good third NRF24 or electrical/RF test equipment is required.

## Files

- Handheld Arduino sketch:
  `radiocontrol_nrf24radio/testing/NRF24_Hello_Handheld_20261003/NRF24_Hello_Handheld_20261003.ino`
- Optional handheld PlatformIO configuration:
  `radiocontrol_nrf24radio/platformio.nrf24-hello.ini`
- Tractor Arduino sketch:
  `tractor_teensy/testing/NRF24_Hello_Tractor_20261003/NRF24_Hello_Tractor_20261003.ino`
- Tractor PlatformIO configuration:
  `tractor_teensy/platformio.nrf24-hello.ini`
- Reverse tractor transmitter:
  `tractor_teensy/testing/NRF24_Hello_Tractor_TX_20261003/NRF24_Hello_Tractor_TX_20261003.ino`
- Reverse handheld receiver:
  `radiocontrol_nrf24radio/testing/NRF24_Hello_Handheld_RX_20261003/NRF24_Hello_Handheld_RX_20261003.ino`

The normal `platformio.ini` files are unchanged.
