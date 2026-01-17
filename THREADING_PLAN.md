# FreeRTOS Threading Plan for Blocking IO

This plan addresses packet rate anomalies observed during rocket launches, hypothesized to be caused by blocking IO operations interfering with ESP-NOW packet transmission/reception.

## Analysis Summary

### Rocket (Flash Writes are the Primary Culprit)

| Blocking Operation | Duration | Impact | Recommendation |
|--------------------|----------|--------|----------------|
| Flash CSV writes | 10-50ms | **HIGH** | Move to Core 0 task with queue |
| Serial debug output | 1-5ms | Medium | Disable during flight or thread |
| I2C sensor reads | 5-10ms | N/A | Keep (necessary) |
| ESP-NOW send | <1ms | N/A | Already non-blocking |

**Root cause**: Flash writes take up to 50ms, blocking the 20ms sensor loop and delaying ESP-NOW transmission.

### Ground Station (UART in ISR Context is Critical)

| Blocking Operation | Duration | Impact | Recommendation |
|--------------------|----------|--------|----------------|
| Serial.printf in onDataRecv() | 5-21ms | **HIGH** | Queue + dedicated task |
| Serial command reading | Variable | Low | Optional: move to Core 0 |
| Debug logging | 1-5ms | Medium | Remove from hot paths |

**Root cause**: `onDataRecv()` callback runs in WiFi task context and directly calls `Serial.printf()`, blocking for 5+ milliseconds and preventing reception of the next packet.

---

## Proposed Implementation Plan

### Phase 1: Ground Station (Highest ROI)

**Task 1.1**: Queue-based telemetry forwarding
- Create `QueueHandle_t telemetryQueue` (20 items)
- In `onDataRecv()`: enqueue packet with `xQueueSendFromISR()` (~50μs)
- Create `telemetryForwardingTask` on Core 1 to dequeue and `Serial.printf()`
- **Impact**: Reduces callback from ~5ms to <50μs (100x faster)

**Task 1.2**: Remove debug logging from hot paths
- Remove/conditionalize `Serial.printf()` in `espnow_comms.cpp` callback paths

### Phase 2: Rocket (Primary Suspect)

**Task 2.1**: Queue-based CSV logging
- Create `QueueHandle_t csvLogQueue` (100 items = 2 seconds buffer at 20ms)
- In `takeSensorReading()`: enqueue reading with `xQueueSend(..., 0)` (non-blocking)
- Create `csvLoggerTask` on Core 0 to dequeue and write to flash
- Add filesystem mutex if concurrent access needed
- **Impact**: Reduces main loop from ~60ms worst-case to ~10ms

**Task 2.2**: Disable debug output during flight
- Skip `Serial.printf()` when `state.transmissionEnabled == true`
- Alternative: Queue debug messages to Core 0 task

### Phase 3: Polish (Optional)

- Increase ground station UART baud rate (115200 → 921600)
- Separate file chunk handling on ground station
- Add queue depth monitoring for diagnostics

---

## Architecture After Implementation

```
ROCKET                                    GROUND STATION
=======                                   ==============

Core 1 (Time-Critical)                    WiFi Task (ISR Context)
├─ loop() @ 20ms                          └─ onDataRecv()
│   ├─ I2C sensor read (5-10ms)               └─ xQueueSendFromISR() (<50μs)
│   ├─ ESP-NOW send (<1ms)
│   └─ xQueueSend(csvLogQueue) (<1ms)     Core 1 (Time-Critical)
│                                         └─ telemetryForwardingTask
└─ Command processing                         └─ xQueueReceive()
                                              └─ Serial.printf()
Core 0 (Background)
├─ csvLoggerTask                          Core 0 (Background)
│   └─ Flash writes (10-50ms, non-blocking)  └─ Command processing task
└─ Optional: debug logging task
```

---

## Data Structures

```cpp
// Both projects
struct QueuedTelemetry {
    SensorReading reading;
    int8_t rssi;  // Ground station only
};

// Rocket only - queue for CSV logging
QueueHandle_t csvLogQueue;  // 100 items × 20 bytes = 2KB

// Ground station - queue for UART forwarding
QueueHandle_t telemetryQueue;  // 20 items × 24 bytes = 480 bytes
```

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Queue overflow | Size queues for 2+ seconds buffering; monitor with `uxQueueMessagesWaiting()` |
| LittleFS race condition | Filesystem mutex for rocket (CSV write vs download) |
| Stack overflow | 4KB stacks for tasks with file/formatting operations |
| Priority inversion | Simple priority scheme: telemetry tasks > background tasks |

---

## Expected Improvement

| Metric | Before | After |
|--------|--------|-------|
| Rocket main loop worst-case | ~60ms | ~10ms |
| Ground station callback time | ~5ms | <50μs |
| Packet drop probability | High during flash writes | Near-zero with buffering |
| Max sustainable packet rate | ~20Hz | 50Hz+ (limited by 20ms sensor interval) |
