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
