# Pump Device (dev_pump)

## 1. Overview

**PumpDevice** is a non-blocking driver that:

- Reads pump status from a contact input (debounced).
- Drives the pump ON/OFF via two output pins using fixed pulse sequences.
- Enforces a **start-protection period**: after the pump is turned OFF by the contact (without a prior HEX/remote turn-off), the pump cannot be turned ON again until the protection window expires (configured in `config.h`).

It implements the **Device** interface (`initialize`, `start`, `timeout`, `event`) and is driven by the framework’s timeout scheduler. It does not subscribe to events (`EVENT_NONE`) and runs on the loop core.

---

## 2. Hardware Interface

| Role          | Parameter    | Direction | Description                                                  |
|---------------|--------------|-----------|--------------------------------------------------------------|
| Contact sense | `contactPin` | INPUT     | Digital input: HIGH = pump ON, LOW = pump OFF (debounced).  |
| Turn ON       | `pwrOnPin`   | OUTPUT    | Pulse HIGH to command pump ON.                              |
| Turn OFF      | `pwrOffPin`  | OUTPUT    | Pulse HIGH to command pump OFF.                             |

**Timing constants** (in header):

- **Debounce:** 4 samples, 20 ms between samples.
- **Read interval:** 1000 ms (periodic status read).
- **ON sequence:** `pwrOnPin` HIGH 5000 ms → wait 5000 ms → release (short 10 ms delay).
- **OFF sequence:** `pwrOffPin` HIGH 1000 ms → release (short 10 ms delay).

Protection duration and tick come from `config.h`: `PROTECT_ON_MS`, `PROTECT_TICK_MS`.

---

## 3. Execution Flow Summary

```
setup()
   │
   ├── initialize()
   │      • Set contactPin INPUT, pwrOnPin/pwrOffPin OUTPUT, outputs LOW
   │      • Read initial contact → _pumpState, _prevPumpState
   │
   └── start()
          • Reset debounce & control state machines
          • Return DURATION_IMMEDIATELY → first timeout() runs soon

loop (scheduler calls timeout() when due)
   │
   └── timeout()
          │
          ├── 1) CONTROL (higher priority)
          │      If control state != IDLE or _controlRequested:
          │         processControl()
          │         • New request: validate (protection, already ON/OFF), then start ON/OFF sequence
          │         • Else advance control state (pulse → wait → release); return next delay
          │         If return != DURATION_IGNORE → use as next timeout
          │
          ├── 2) DEBOUNCE
          │      If debounce in progress (not IDLE and not DONE):
          │         processDebounce()
          │         • Sample contact 4× at 20 ms; on 4th: all HIGH→ON, all LOW→OFF, else keep previous
          │         • updatePumpState() → edge detect: ON→OFF and !_lastOffByHexFlag → arm protection
          │
          ├── 3) PROTECTION
          │      processProtection()
          │         • If _protectOnStart: when elapsed >= PROTECT_ON_MS clear it; else log remain every PROTECT_TICK_MS
          │
          └── 4) NEXT TIMEOUT
                 • If control active → return 10 (poll every 10 ms)
                 • If debounce IDLE → start new cycle, return DURATION_IMMEDIATELY
                 • If debounce DONE → set IDLE, return _readIntervalMs (1000 ms)
                 • Else (during debounce) → return PUMP_DEBOUNCE_DELAY_MS (20 ms)
```

---

## 4. State Machines

### 4.1 Debounce (contact reading)

- **Purpose:** Stabilize contact input and avoid noise.
- **Steps:** `DEBOUNCE_READING_SAMPLE_0` → `…_1` → `…_2` → `…_3` → `DEBOUNCE_DONE`.
- **Rule:** State is updated only when all 4 samples are HIGH (pump ON) or all LOW (pump OFF); otherwise the previous state is kept.
- **Timing:** Each step runs on timeout after `PUMP_DEBOUNCE_DELAY_MS` (20 ms).

### 4.2 Control (ON/OFF)

- **Turn ON** (when not protected and pump currently OFF):
  - `CONTROL_TURNING_ON_PULSE1`: drive `pwrOnPin` HIGH → next in 5000 ms.
  - `CONTROL_TURNING_ON_WAIT`: next in 5000 ms.
  - `CONTROL_TURNING_ON_PULSE2`: drive `pwrOnPin` LOW, short delay, then `CONTROL_IDLE`.
- **Turn OFF** (when pump currently ON):
  - `CONTROL_TURNING_OFF_PULSE`: drive `pwrOffPin` HIGH → next in 1000 ms.
  - `CONTROL_TURNING_OFF_WAIT`: drive `pwrOffPin` LOW, short delay, then `CONTROL_IDLE`.

Turn-on requests are ignored if protection is active or pump is already ON; turn-off is ignored if pump is already OFF.

---

## 5. Start Protection Logic

- **Armed when:** Contact read sees a transition ON → OFF and `_lastOffByHexFlag` is false (i.e. the OFF was not initiated by HEX/remote).
- **Effect:** `_protectOnStart = true`; any **requestTurnOn()** is rejected until `PROTECT_ON_MS` has elapsed.
- **Clearing:** Caller sets `_lastOffByHexFlag` before requesting turn-off when the OFF is initiated by HEX/remote, so the subsequent contact-driven ON→OFF does not trigger protection.

In short: protection blocks restart for a fixed period after the pump was turned off by the contact (e.g. external/mechanical), not by the device.

---

## 6. Public API

| Method | Description |
|--------|-------------|
| `initialize()` | Configure pins and read initial pump state. |
| `start()` | Reset state machines; first timeout is immediate. |
| `timeout()` | Run control, debounce, protection; return next timeout (ms). |
| `event()` | No-op; returns `DURATION_IGNORE`. |
| `isPumpOn()` | Current debounced pump state (from contact). |
| `getPreviousPumpState()` | State before last debounced update. |
| `isProtectionActive()` | Whether the start-protection window is active. |
| `getProtectionRemainingMs()` | Remaining protection time in ms, or 0. |
| `requestTurnOn()` | Request non-blocking turn-on (subject to protection and current state). |
| `requestTurnOff()` | Request non-blocking turn-off. |
| `setLastOffByHexFlag(bool)` | Set so the next contact-driven ON→OFF does not arm protection. |
| `armProtection(const char* reason)` | Manually arm protection (e.g. for logging). |
| `getPumpStatusByte()` | `1` if pump ON, `0` if OFF (for compatibility). |

---

## 7. Design Notes

- **Non-blocking:** All sequencing is timeout-driven; only a short 10 ms delay is used after releasing ON/OFF pins.
- **Watchdog:** `WDT_RESET()` is used during debounce and control so long pulses do not trigger the watchdog.
- **Priority:** Control is processed before debounce in `timeout()`, so ON/OFF sequences are not delayed by the 1 s read interval.
