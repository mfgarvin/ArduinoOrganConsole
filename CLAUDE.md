# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino MIDI Conversion Machine v2 - firmware for converting organ keyboard inputs to USB MIDI. Targets Arduino Due (3.3V logic). Uses multiplexed digital inputs to read 4 organ manuals (61 keys each), 32 pedals, 15 pistons, and stop controls. Includes shift register LED driver for 64 stop indicator LEDs.

## Build & Upload

This is a standard Arduino IDE sketch:
1. Open `Arduino_MIDI_Machine_v2.ino` in Arduino IDE
2. Install the MIDIUSB library (Sketch → Include Library → Manage Libraries → search "MIDIUSB")
3. Select target board (Arduino Leonardo or Mega with USB MIDI support)
4. Compile and upload

Serial debugging available at 9600 baud.

## Architecture

### Single-File Structure
The entire firmware is in `Arduino_MIDI_Machine_v2.ino`. No modular organization.

### Core Loop Pattern
```
loop() → processMidiInput() → readKeys() → checkForKeyChanges() → statusController()
```

### State Tracking
Each keyboard division uses triple-state tracking with 64-bit integers:
- `standby*` - accumulating next state from hardware reads
- `active*` - current confirmed state
- `past*` - previous state for change detection

### MIDI Channel Mapping
- Channel 0: Swell manual (Note On/Off)
- Channel 1: Great manual (Note On/Off)
- Channel 2: Choir manual (Note On/Off)
- Channel 3: Pedals (Note On/Off)
- Channel 4: Pistons (Control Change, send only, CC# 0-17)
- Channel 5: Stops (Control Change, bidirectional, CC# 0-63)

### Debounce System
Uses `debounceArray[10][64]` with separate counters for press/release per key. Threshold of 10 cycles before state change is confirmed.

### Multiplexer Addressing
4 control pins (4,5,6,7) select 1 of 16 multiplexer channels. Each manual has 4 multiplexed input banks.

### Key-to-MIDI Mapping
Bit position + 24 = MIDI note number (e.g., bit 0 = note 24 = C2). All velocities fixed at 127.

## Hardware Pin Assignments

**Multiplexer control**: Pins 4,5,6,7 (A,B,C,D)
**Swell inputs**: 28, 26, 24, 22
**Great inputs**: 29, 27, 25, 23
**Choir inputs**: 38, 36, 34, 32
**Pedal inputs**: 35, 33
**Piston inputs**: 39, 37
**Stop inputs**: 40, 41, 42, 43
**LEDs**: 13 (connection), 72/73 (activity)
**Stop LED driver**: SPI header (MOSI/SCK) + Pin 53 (latch) → 74HC595 chain

## Manual Reference Characters
Used internally in function parameters:
- 'g' = Great, 's' = Swell, 'c' = Choir, 'p' = Pedal, 'b' = Pistons, 'v' = Stops

## 64-bit Macro Extensions
Standard Arduino bit macros are redefined (lines 4-11) to support 64-bit `uint64_t` state variables using `1ULL` literals.

## Bidirectional Stop Sync

Stops use bidirectional MIDI Control Change messages on channel 5, allowing both hardware buttons and software to control stop state.

### State Variables
- `activeStops` / `pastStops` - raw button state (for edge detection)
- `stopState` - logical on/off state (synced with software, drives LEDs)
- `stopStatePrev` - previous logical state (for change detection)

### MIDI Protocol
- **Message type**: Control Change (0xB5)
- **CC number**: Stop index (0-63)
- **CC value**: 127 = engaged, 0 = disengaged (threshold at 64 for receiving)

### Behavior
1. **Hardware → Software**: Momentary button press toggles `stopState` bit, sends CC
2. **Software → Hardware**: Incoming CC updates `stopState`, LEDs update automatically
3. **No echo**: Arduino does not echo back CC messages received from software

### Key Functions
- `sendStopCC(byte stopIndex, byte state)` - sends CC message for stop change
- `processMidiInput()` - handles incoming MIDI, updates `stopState` on CC channel 5

## Stop LED Driver (74HC595 Shift Registers)

Driver code is implemented but circuit is not yet built. See `pointers/LED_Shift_Register_Circuit.txt` for full circuit documentation.

### Design Summary
- 8x 74HC595 shift registers daisy-chained for 64 stop indicator LEDs
- Driven by `stopState` variable (synced bidirectionally with software)
- Common-anode LED wiring with split voltage rails
- 74HC595 chips powered at 3.3V (matches Due logic levels)
- LEDs powered from 5V (for brightness)
- Output is inverted in code (`~stopState`) so LOW = LED ON

### Connections (Arduino Due)
- **SPI Header Pin 4 (MOSI)** → Data (DS) on first 74HC595
- **SPI Header Pin 3 (SCK)** → Clock (SHCP) on all chips (parallel)
- **Digital Pin 53** → Latch (STCP) on all chips (parallel)

### Key Functions
- `setupLEDs()` - initializes SPI and clears all LEDs
- `updateStopLEDs(uint64_t stopState)` - shifts out 64 bits to LED chain

### LED Wiring (per LED)
```
+5V ──[150Ω]──┬── LED Anode
              │
         LED Cathode
              │
        74HC595 Output (sinks current when LOW)
              │
             GND
```

### Hardware Notes
- Use standard 74HC595 (not 74HCT595)
- 150Ω resistors for ~17mA, or 270Ω for ~10mA if many LEDs on simultaneously
- Each 74HC595 can sink max 70mA total (watch current if all 8 outputs active)
- Need external 5V supply if more than ~50 LEDs on at once (Due USB limit ~800mA)

## Debug Logging

Set `DEBUG_MIDI = true` (line 18) to log all MIDI traffic to Serial Monitor at 9600 baud.

**Log format:**
```
TX NoteOn  ch=0 note=60 (s)       # Outgoing note on (s=swell, g=great, c=choir, p=pedal)
TX NoteOff ch=1 note=48 (g)       # Outgoing note off
TX CC ch=4 cc#=0 val=127 (piston) # Outgoing piston CC
TX CC ch=5 cc#=10 val=127 (stop)  # Outgoing stop CC
RX CC ch=5 cc#=10 val=0           # Incoming CC (stop from software)
RX NoteOn ch=0 note=60 vel=127    # Incoming note
```

## Known Issues
- Velocity is hardcoded to 127 (no dynamic support)
- No stuck key recovery (mentioned in TODO but not implemented)
