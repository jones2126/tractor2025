# Teensy 4.1 — Pin Reference

**Official pinout:** [[teensy_41_pinout.pdf]]
**Source:** PJRC — www.pjrc.com/teensy

---

## Ethernet Add-On Board

The Teensy 4.1 Ethernet add-on (PJRC kit with DP83825 PHY) connects exclusively to **bottom pads** on the underside of the Teensy 4.1 PCB.

**It does NOT use any of the numbered edge pins (0–41).**

This means Ethernet is fully compatible with all standard peripherals including the NRF24 radio (pins 9–13). No pin conflicts exist.

> **Note:** An earlier analysis incorrectly stated that Ethernet uses pin 13. This was wrong — the mapping from GPIO pad names to Arduino pin numbers in the driver source was misinterpreted. The PJRC pinout card confirms pin 13 is SCK/LED only, with no Ethernet function.

---

## SPI Buses

| Bus | CS | MOSI | MISO | SCK | Notes |
|-----|----|------|------|-----|-------|
| SPI0 | 10 | 11 | 12 | 13 | Default SPI. Pin 13 also = on-board LED. |
| SPI1 | 38 | 26 | 39 | 27 | Second SPI bus (MOSI1/MISO1/SCK1 labels on card) |

---

## Pins Used — Tractor Robot Project

| Pin | Function | Device | Direction |
|-----|----------|--------|-----------|
| 2 | DATA_PIN (NeoPixel) | LED | OUT |
| 5 | RPWM | IBT-2 steering | OUT |
| 6 | LPWM | IBT-2 steering | OUT |
| 9 | CE | NRF24 radio | OUT |
| 10 | CSN (CS) | NRF24 radio | OUT |
| 11 | MOSI | NRF24 radio (SPI0) | OUT |
| 12 | MISO | NRF24 radio (SPI0) | IN |
| 13 | SCK / LED | NRF24 radio (SPI0) | OUT |
| 32 | ESTOP_RELAY_PIN | E-stop relay | OUT |
| A9 (23) | STEER_POT_PIN | Steering potentiometer | IN |
| Serial3 | JRK G2 | Transmission actuator | TX/RX |

**Bottom pads (Ethernet add-on — no edge pin conflicts):**
- ENET_RXD0, ENET_RXD1, ENET_RXEN, ENET_RXER
- ENET_TXD0, ENET_TXD1, ENET_TXEN
- ENET_REF_CLK, ENET_MDIO, ENET_MDC
- 3.3V, GND

---

## Key Analog Pins

| Pin | Label | Used for |
|-----|-------|----------|
| 14 | A0 | |
| 15 | A1 | |
| 16 | A2 | |
| 17 | A3 | |
| 18 | A4 | |
| 19 | A5 | |
| 20 | A6 | |
| 21 | A7 | |
| 22 | A8 | |
| 23 | A9 | Steering potentiometer |

---

## Notes

