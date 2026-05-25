# Radio UART pinout — production ESP32-S3 board

## Summary

The RFD868 radio modem is the **operator comms channel in deployment**
— all CMD / EVT / TLM frames travel through it. It lives on a UART
separate from UART0:

| Signal       | Chip GPIO | Module pin | Direction |
|--------------|-----------|------------|-----------|
| `RFD_RX`     | IO10      | Pin 18     | radio → ESP (chip RX) |
| `RFD_TX`     | IO11      | Pin 19     | ESP → radio (chip TX) |
| `RFD_STATUS` | (TBD)     | —          | radio → ESP, link-up indication |

The board also exposes the radio's 5 V supply and GND on the same
header.

## Working configuration

```cpp
// Arduino-ESP32 v3.x: begin(baud, cfg, RX, TX)
Serial1.begin(57600, SERIAL_8N1, /*RX=*/10, /*TX=*/11);
```

- Baud: **57600** (matches the previous-semester firmware
  `RADIO_BAUD`; confirm against the radio module's stored config on
  first contact, adjust if needed).
- UART instance: **`Serial1`** (UART1) — picked by convention; no
  hardware constraint forcing UART1 vs UART2.
- The radio handles its own RF protocol; the ESP just streams the
  wire-protocol bytes (`CMD,...*<adler>\n` and `EVT,...*<adler>\n`)
  through it.

## Architecture implication

Because the radio has its own UART, the CP210x adapter on UART0 is
*not* the deployment operator channel — it's a dev/programmer port.
Deployment firmware:

- Opens `Serial` (UART0) with the swapped pinout for the servo bus
  (see [`servo_uart_pinout.md`](servo_uart_pinout.md)).
- Opens `Serial1` for the radio link.
- Both run simultaneously, on physically separate UARTs — no time-
  multiplexing, no pin-swap dance.

## Open items

- **RFD_STATUS GPIO** — visible on the board silkscreen as
  `RFD868_STATUS`, but the chip GPIO it maps to is not labeled in the
  pinout PDF page I have. Identify before adding link-up detection.
- **Baud verification** — radio module's actual configured baud may
  not match the legacy 57600. Confirm on first contact.

## Code references

- `embedded/include/RadioTransport.h` — UART setup wrapper (pinout +
  baud constants)
- `embedded/include/RadioLink.h` — wire-protocol framing, transport-
  agnostic (takes any `Stream&`)
- `embedded/src/main.cpp` — composes `RadioTransport` + `RadioLink` in
  the operator env
