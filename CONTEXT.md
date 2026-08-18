# Heltec nRF52 Arduino Boards

This context captures the project language for Heltec nRF52 board variants and their examples.

## Language

**Mesh Node T114**:
The Heltec nRF52 mesh node board variant with onboard LoRa radio, GPS capability, and TFT display.
_Avoid_: T114 when the exact board family matters.

**Mesh Pocket V2**:
The Heltec nRF52 board variant shown as `Mesh Pocket V2` in the board menu and implemented by the `HT-mesh-pocket-v2` variant. It has SX1262 LoRa, UC6580 GNSS wired to `Serial1`, and an NV3001B TFT.

**Mesh Tower V2**:
The Heltec nRF52840 tower board variant shown as `Mesh Tower V2` in the board menu and implemented by the `HT-mesh-tower-v2` variant. It has SX1262 LoRa with an external PA/FEM, L76K GNSS wired to `Serial1`, battery sensing, USB PD controller I2C, and no onboard TFT or RGB.

**RadioCore-52840**:
The RC52-L62-derived nRF52840 board variant shown as `RadioCore-52840` in the board menu. It has SX1262 LoRa, battery sense, a header UART, and an optional TFT connector, but no onboard GNSS.

**Factory Test**:
A production validation sketch that reports hardware test results for fixture logs. It is not a user-facing demo and does not imply screen, GNSS, or BLE coverage unless a board explicitly has those test fixtures.

## Relationships

- A **Mesh Node T114** can run LoRa and GPS examples that are written against the shared nRF52 board abstraction.
- A **Mesh Pocket V2** GPS TFT example requires Arduino_GFX for the NV3001B display.
- **Mesh Tower V2** exposes GPS and battery support through the shared nRF52 board abstraction, but it is not a screen board.
- **RadioCore-52840** provides pin-level TFT connector support only; display examples need an external display library selected by the sketch.

## Example Dialogue

> **Dev:** "Should this LoRa example support the **Mesh Node T114**?"
> **Domain expert:** "Yes, but it must keep working on the existing ST7735 display boards too."
> **Dev:** "For the GPS serial display example, which board does **Mesh Node T114** mean?"
> **Domain expert:** "Use the board menu entry `Mesh Node T114(HT-n5262)` and its onboard TFT plus GPS serial wiring."
> **Dev:** "Should the **Mesh Pocket V2** GPS TFT example use the same display stack as Mesh Node T114?"
> **Domain expert:** "No. Mesh Pocket V2 uses an NV3001B display through Arduino_GFX, while Mesh Node T114 uses the existing ST7789 example path."

## Flagged Ambiguities

- "T114" was used as shorthand for **Mesh Node T114**; in this repository that is the board shown as `Mesh Node T114(HT-n5262)` in the board menu.
