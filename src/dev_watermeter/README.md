# Modbus Water Meter (dev_watermeter)

## 1. Overview

**ModbusWaterMeter** is a driver that:

- Reads flow data from a water meter over **RS485 Modbus RTU** (9600 baud).
- Reads **flow unit** (register 0x0006), **flow rate** (0x0005), and **cumulative flow** (0x000A) in a fixed sequence each cycle.
- Converts flow rate to **m³/s** using the meter’s flow unit (e.g. m³/h, L/min).
- Keeps a **circular history** of readings and provides **statistics** (min/max/avg flow, total consumed over the stored period).

It implements the **Device** interface (`initialize`, `start`, `timeout`, `event`). It does not subscribe to events (`EVENT_NONE`) and runs on the loop core. Reading is **blocking** during request/response (Modbus timeout and inter-byte timeouts).

---

## 2. Hardware and Protocol

| Item | Description |
|------|-------------|
| **Interface** | `Uart*` (e.g. SERCOM0) for RS485; RX/TX pins per `config.h` (e.g. 5, 6 with `pinPeripheral(..., PIO_SERCOM_ALT)`). |
| **Baud rate** | 9600. |
| **Protocol** | Modbus RTU, function code **0x04** (Read Input Registers). |
| **Address** | `METER_ADDRESS` (e.g. 0x01). |
| **Registers** | Flow unit: `METER_FLOW_UNIT_ADDRESS` (0x0006), 1 register, uint16. Flow rate: `METER_FLOW_RATE_ADDRESS` (0x0005), 2 registers, Float Inverse. Cumulative: `METER_CUMULATIVE_ADDRESS` (0x000A), 2 registers, Float Inverse. |

**Flow unit values (0–8):** 0=none, 1=m³/s, 2=m³/min, 3=m³/h, 4=m³/d, 5=m³/h, 6=L/s, 7=L/min, 8=L/h. Flow rate is converted to m³/s internally using these.

---

## 3. Execution Flow Summary

```
setup()
   │
   ├── initialize()
   │      • pinPeripheral() for RS485 pins, _rs485->begin(9600)
   │      • Drain RX buffer
   │
   └── start()
          • _lastReadTime = millis(); _readingState = 0
          • return DURATION_IMMEDIATELY

loop (timeout() when due)
   │
   └── timeout() — switch (_readingState)
          │
          ├── 0 (flow unit)
          │      • readUint16Parameter(METER_FLOW_UNIT_ADDRESS) → _flowUnit (clamped 0–8)
          │      • _readingState = 1; return 50
          │
          ├── 1 (flow rate)
          │      • readParameter(METER_FLOW_RATE_ADDRESS) → float
          │      • _latestReading.currentFlow = convertToM3PerSecond(value, _flowUnit)
          │      • _readingState = 2; return 50
          │
          └── 2 (cumulative flow)
                 • readParameter(METER_CUMULATIVE_ADDRESS) → _latestReading.cumulativeFlow
                 • _latestReading.valid = success; update _totalValidReadings / _totalFailedReadings
                 • addToHistory(_latestReading)
                 • _readingState = 0; return _readIntervalMs (e.g. 15000)
```

So each **cycle** is: flow unit → 50 ms → flow rate → 50 ms → cumulative → store → wait `_readIntervalMs` (e.g. 15 s), then repeat.

---

## 4. Reading Sequence and Timing

- **State 0:** One Modbus read (1 register) for flow unit; 50 ms delay to next step.
- **State 1:** One Modbus read (2 registers, float) for flow rate; convert to m³/s; 50 ms delay.
- **State 2:** One Modbus read (2 registers, float) for cumulative flow; set `valid`; append to history; next timeout = `_readIntervalMs` (from `config.h`: `WATER_METER_READ_INTERVAL_MS`, e.g. 15000 ms).

Within each `readParameter` / `readUint16Parameter`: send request, 50 ms delay, then parse response with **first-byte timeout** (e.g. 200 ms) and **inter-byte timeout** (e.g. 30 ms). CRC and header are verified; float is parsed as IEEE754 **Float Inverse** (byte order reversed per L-MAG-BM).

---

## 5. Data Structures

- **WaterMeterReading:** `timestamp` (millis), `currentFlow` (m³/s), `cumulativeFlow` (m³), `valid`.
- **WaterMeterStats:** `currentFlowMin/Max/Avg`, `cumulativeFlowStart/End`, `totalConsumed`, `validReadings`, `failedReadings` (over the history buffer).
- **History:** Circular buffer of `WaterMeterReading`, size `MAX_HISTORY_SIZE` (e.g. 60 from `config.h`). New reading overwrites oldest when full.

---

## 6. Public API

| Method | Description |
|--------|-------------|
| `initialize()` | Configure RS485 pins, begin UART, clear RX. |
| `start()` | Reset read state and timestamps; return DURATION_IMMEDIATELY. |
| `timeout()` | Run one step of the 3-step read; return next timeout (50 ms or _readIntervalMs). |
| `event()` | No-op; returns DURATION_IGNORE. |
| `begin()` | Alias for `initialize()`. |
| `readAllParameters()` | Blocking: flow unit + flow rate + cumulative, then update latest and history. |
| `getCurrentFlow()`, `getCumulativeFlow()` | Latest reading values (m³/s, m³). |
| `isLastReadingValid()` | Whether the last cumulative read succeeded. |
| `getHistoricalReading(index)` | History slot (0 = latest, 1 = previous, …). |
| `getStatistics()` | Min/max/avg flow and total consumed over history. |
| `getHistoryCount()` | Number of readings currently in history. |
| `clearHistory()` | Clear history and reset indices. |
| `printLastReading()`, `printStatistics()` | Debug output to Serial. |

---

## 7. Design Notes

- **Blocking:** Request is sent and the code waits for response (with timeouts); the main loop is blocked during each read. Interval between cycles (e.g. 15 s) keeps impact acceptable.
- **Float Inverse:** Meter uses reversed byte order for floats; `parseFloatInverse()` reorders bytes before interpreting as IEEE754.
- **Conversion:** `convertToM3PerSecond()` normalizes all flow units to m³/s for consistent reporting and stats.
- **Config:** Register addresses, meter address, baud, `WATER_METER_READ_INTERVAL_MS`, and `MAX_HISTORY_SIZE` are in `config.h`.
