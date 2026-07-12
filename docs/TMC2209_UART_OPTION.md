# Optional TMC2209 UART wiring

UART is not required for SuperRot. The current firmware and CNC Shield V3 work in
standalone STEP/DIR mode.

The stock CNC Shield V3 does not route TMC2209 UART and has no UART-enable jumper.
On a typical StepStick-format TMC2209 carrier, UART must be wired to the carrier pad
labelled `PDN_UART`, `UART`, or `PDN`. Carrier layouts vary, so identify the exact
module before soldering.

## ESP32 single-wire connection

- ESP32 GPIO17 (UART2 TX) directly to the shared PDN_UART node.
- ESP32 GPIO16 (UART2 RX) through a 1 kohm resistor to that same node.
- ESP32, shield, and driver logic must share ground.
- Keep the existing STEP, DIR, and EN wiring unchanged.

For two drivers on one wire, give them different UART addresses using their
MS1_AD0/MS2_AD1 straps. A common arrangement is X at address 0 (both low) and Y at
address 1 (AD0 high), but do not change a CNC-shield jumper until the carrier's
schematic or clear board photos confirm how its MS pins and solder bridges are wired.
Once UART controls microstep resolution, the address straps no longer serve as the
firmware's microstep setting.

Some carriers require opening or closing a tiny solder bridge between PDN_UART and
the header pad. That bridge is carrier-specific, not part of the CNC Shield V3.

Do all soldering with USB and motor power disconnected. Verify continuity from the
labelled carrier pad to the module's PDN_UART net before connecting the ESP32.
