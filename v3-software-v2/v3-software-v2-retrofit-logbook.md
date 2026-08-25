---
started: 2026-08-25
status: in progress
applies-to: v3-software-v2/v3-software-v2.ino
---

# Retrofit Logbook

Chronological record of the physical retrofit described in
[`v3-software-v2-retrofit-guide.md`](v3-software-v2-retrofit-guide.md). One entry per
work session. Each step taken gets a line here, checked off against the guide's §4
procedure / pre-power checklist / §8 verification as it's done. This log is the
record of what actually happened; the guide stays the design reference and isn't
edited to reflect log entries.

Entry format:

```
## YYYY-MM-DD

- [x] Step taken, brief result/measurement
- [ ] Step attempted, outcome / issue
```

Use `[x]` done, `[~]` partial/in progress, `[ ]` blocked or not yet done. Note actual
measured values (resistances, voltages) inline, not just "done" — the pre-power
checklist and §6 calibration depend on them.

---

## 2026-08-25

Hardware still running v1.ino during this session (no reflash yet).

- [x] §4 step 2 — TEC-8A harness removed: D9 (TXS0108E `OE`), D10/D11
      (SoftwareSerial RX/TX), A1 (3.3V sense wire) all disconnected.
- Confirmed D3/D4 have nothing wired to them (from earlier bench check) — no
  "other hardware" harness existed there, so no removal action was needed on
  those pins for this step.
- [x] §4 step 3 — monitor dividers built and connected:
      - ACT (TACT): measured 21.7k / 46.3k, tapped to Arduino A0 and
        ADS1115 #1 (0x49) **ch3** (physical pin A3 on that breakout).
      - SET (TSET): measured 21.7k / 47.0k, tapped to Arduino A1 and
        ADS1115 #1 (0x49) **ch2** (physical pin A2 on that breakout).
      - Confirmed via ADDR-pin check: the 0x49 chip (ADDR->5V) is the
        one carrying IMON/ACT/SET; the 0x48 chip (ADDR->GND) is
        untouched and still carries the existing TEC100L VTEC/ITEC/TSET/TACT
        readings — no conflict between the two chips.
- **Design deviation from the original guide**: bench wiring landed ACT on
  ADS1 ch3 instead of the originally documented ch1, leaving ch1 (`A1C1`)
  spare instead of ch3. Rather than rewire, **`v3-software-v2.ino` and
  `v3-software-v2-retrofit-guide.md` were both updated to match this
  wiring** (channel-name PROGMEM strings, `ads1_values[]` divider-undo
  branch, `ptcActV_ADS` source index, CSV `ACTMR`/`ACTMV`/`A1C1R`/`A1C1V`
  fields, guide §2/§3.1/§4/§5.3 text). SET on ch2 was already correct, no
  change there. Recompiled clean with `arduino-cli compile --fqbn
  arduino:avr:nano v3-software-v2`: 18880 B flash (61%), 1424 B RAM (69%) —
  matches guide §1 table within rounding. Committed as firmware+doc change,
  not yet flashed to hardware (still running v1.ino).
- [x] §4 step 3 (cont.) — Nano-side taps landed: divider nodes also run to
      **A0** (PTC-TACT / ACT T MON, `PIN_PTC_ACT`) and **A1** (PTC-TSET /
      SET T MON, `PIN_PTC_SET`), per the original pin map — unaffected by
      the ADS1 channel swap above.
- [x] §4 step 5 — manual enable switch wired to **A2** (`PIN_PTC_MANUAL`),
      pulldown to GND used **15k** in place of the guide's 10k. This pin is
      read as a digital HIGH/LOW (`digitalRead`, no divider math), so the
      substitution needs no firmware or calibration change — any pulldown
      in this range holds a solid LOW with the switch open. No guide edit
      needed either, since 10k there is just the recommended value, not a
      constraint tied to a computed constant.
- [x] §4 step 6 — PTC ENABLE wired: D9 through 1k, 10k pulldown at the PTC
      end, as documented.
- [x] §4 step 7 — indicator LEDs wired: D10 (green OK), D11/D12 (fault,
      parallel).
- [x] Updated `R_TOP_K`/`R_BOTTOM_K` in `v3-software-v2.ino` to the
      bench-measured dividers: `R_TOP_K` 21.73 -> **21.70** (both ACT and
      SET measured 21.7); `R_BOTTOM_K` 47.31 -> **46.65**, the average of
      ACT's measured 46.3 and SET's measured 47.0 (the sketch shares one
      `DIVIDER_RATIO` constant across both channels, so this is the
      closest single-value fit). `R_PTC_OUT_K` left at 1.00 — that's the
      PTC's own output-impedance spec, not a bench-measured resistor.
      Recompiled clean: 18880 B flash (61%), 1424 B RAM (69%), unchanged
      from before the edit.
- [x] §4 step 8 — common ground confirmed across Arduino, PTC10K-CH, and
      divider/LED grounds.
- [ ] 5V rail measurement deferred — `ADC_REF_V` stays at the 5.000
      placeholder for now.
- [x] Manual switch verified at the A2 pin: reads 5V with switch ON, GND
      with switch OFF.
- [x] PTC ENABLE confirmed physically disconnected from the PTC10K-CH end.
      Pre-power checklist clear (5V rail measurement excepted, deferred) —
      ready to flash.
- [x] Page 3 reworked to follow the D5 pause switch as a display-source
      select: normal (HIGH) shows the ADS1115 (16-bit) reading, paused
      (LOW) falls back to Nano `analogRead()` (10-bit) — the source is
      labeled on row 2 (`~UNCAL SRC:ADC`/`NANO`). This drops the old
      side-by-side ACT/SET cross-check display in favor of one source
      per screen with more decimal digits (row 0/1 now 4 decimal places
      on voltage and temp, vs 1 decimal on temp before); the cross-check
      is still available by toggling the switch and comparing screens.
      `serviceInterlock()`'s trip decision is untouched — always reads
      A0/A1 directly, independent of this switch.
- [x] Added two diagnostic states to `ptcStateText()` (shared by page 0's
      status line and page 3 row 3), ahead of the normal
      MANUAL OFF/WARMUP/FAULT states:
      - `PTC NO PWR` — both ACT and SET monitor voltages under
        `PTC_NO_POWER_V` (0.05V), i.e. the PTC10K-CH itself unpowered.
      - `NTC DISCONN` — either monitor voltage over `NTC_DISCONNECT_V`
        (6.0V), the open-thermistor signature (divider node pulled near
        the ADC rail, ~7.4V once undone with this unit's resistors).
      Display-only change — both conditions already sit outside
      `V_LOW`/`V_HIGH` (0.750–1.400V), so ENABLE was already
      guaranteed to stay latched LOW in either case; this just names
      the cause instead of showing a generic FAULT.
      Recompiled clean: 19002 B flash (61%), 1454 B RAM (70%).
- [~] Uploading this firmware to hardware now (first flash of
      `v3-software-v2.ino`, replacing v1.ino) — §8 verification not yet
      started.
