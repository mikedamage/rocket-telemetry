# Code Quality Improvements - Zed Shaw Review Follow-up

Based on the code review performed earlier in the session, here are the remaining issues to address.

## Completed Items ✓

- [x] **Commented-Out Code** - Removed all commented-out code blocks from both rocket and ground station ESP-NOW modules
- [x] **Fake Atomic Operations** - Replaced fake atomic increments with proper `std::atomic<uint32_t>` and `.fetch_add()` operations
- [x] **Global State Organization** - Consolidated scattered global variables into organized structs:
  - Rocket: `RocketState` struct (15+ variables consolidated)
  - Ground Station: `GroundStationState` struct

## Remaining Issues

### High Priority

1. **Error Handling Inconsistency** (rocket/src/main.cpp, ground-station/src/main.cpp)
   - Sometimes logs and continues
   - Sometimes logs and halts in infinite loop (e.g., ground-station/src/main.cpp:149-151)
   - Sometimes just returns false
   - **Action**: Document current strategy in code - halt on critical errors (ESP-NOW init), continue on non-critical
   - **Impact**: Low (documentation only) - current behavior is acceptable
   - **Decision**: Keep halt-on-critical-error for safety

2. **Buffer Overflow Risk in Serial Input** (ground-station/src/main.cpp:128-145)
   - When 256+ chars arrive without newline, error is logged but remaining bad input isn't consumed
   - Next character starts a new command, leading to potential command parsing bugs
   - **Action**: Consume all input until newline when buffer overflows
   - **Impact**: High - could cause command misinterpretation

3. **CSV Logging Flash Management** (rocket/src/main.cpp:385-404)
   - Check runs on EVERY write (expensive)
   - When space runs out, logging silently stops (state.csvLoggingEnabled = false)
   - No notification sent to ground station
   - **Action**: Check every 100 writes, notify ground station when storage full
   - **Impact**: Medium-High - improves performance and visibility
   - **Decision**: Check every 100 writes (~5 seconds at 20ms intervals)

### Medium Priority

4. **Watchdog Band-Aid** (both main.cpp files)
   - `delay(1)` in main loops to prevent watchdog issues
   - Indicates architectural problem or improper watchdog configuration
   - **Action**: Investigate blocking operations and remove delay if possible
   - **Impact**: Medium - may reveal underlying timing issues
   - **Decision**: Find and fix root cause instead of masking with delay

5. **No Flow Control on File Downloads** (rocket/src/main.cpp:406-462)
   - Sends chunks with 5ms delay
   - No ACKs, no retries, no flow control
   - If ground station buffer fills or packet lost, data is gone
   - **Action**: Document limitation in code comment
   - **Impact**: Low (documentation only) - acceptable for post-flight data recovery
   - **Decision**: Document limitation, no retry protocol needed

6. **Hardcoded File Path** (rocket/src/main.cpp:60)
   - Single file `/telemetry.csv`
   - If truncate fails, all data potentially lost
   - No rotation, no backup
   - **Action**: Document design choice in code comment
   - **Impact**: Low (documentation only) - TRUNCATE command clears before each flight
   - **Decision**: Keep single file for simplicity

### Low Priority

7. **Magic Numbers** (ground-station/src/main.cpp:27, rocket/src/main.cpp various)
   - Buffer sizes like 256, 50, 1000 (buffer margins) defined inline
   - Some have rationale, others don't
   - **Action**: Define as named constants with comments explaining rationale
   - **Impact**: Low - code clarity and maintainability

8. **Negative Altitude Clamping** (rocket/src/main.cpp:308-310)
   - Clamps negative altitude to 0 silently
   - Masks potential sensor calibration problems
   - **Action**: Log when clamping occurs so sensor issues are visible
   - **Impact**: Low - diagnostic visibility

9. **Debug Output Frequency** (rocket/src/main.cpp:336-343)
   - Outputs every 25 readings (arbitrary)
   - Should be time-based or configurable
   - **Action**: Make frequency configurable or time-based
   - **Impact**: Low - debug convenience

10. **Silent Send Callback** (ground-station/src/espnow_comms.cpp:47-51)
    - onDataSent callback does nothing
    - Comment says "could add error counter if needed"
    - **Action**: Either remove callback registration or implement error counting
    - **Impact**: Low - currently no observable impact

## User Decisions

Based on clarifying questions:
- **Error Handling**: Keep halt-on-critical-error (current behavior) - ESP-NOW init failure halts system
- **Storage Check Frequency**: Check every 100 writes instead of every write
- **Watchdog Band-Aid**: Investigate and remove the `delay(1)` by finding/fixing blocking operations
- **File Download Flow Control**: Document the limitation (no ACK/retry) - acceptable for post-flight downloads
- **File Backup**: Keep single file `/telemetry.csv` - TRUNCATE command clears before each flight

## Implementation Plan

### Phase 1: Quick Wins (Items 2, 7, 8, 9, 10)
1. **Serial buffer overflow** - Consume input until newline on overflow
2. **Magic numbers** - Define as named constants with rationale comments
3. **Altitude clamping** - Log when clamping occurs
4. **Debug output** - Make frequency configurable (add to RocketState)
5. **Silent send callback** - Remove unused callback or add error counter

### Phase 2: Storage Management (Item 3)
1. Add write counter to RocketState
2. Check storage every 100 writes instead of every write
3. When storage full, send error notification to ground station via Serial.printf

### Phase 3: Watchdog Investigation (Item 4)
1. Identify what operations might be blocking (I2C reads, file writes, etc.)
2. Check if operations can be optimized
3. Remove `delay(1)` if no blocking found
4. Document findings

### Phase 4: Documentation (Items 1, 5, 6)
1. Document error handling strategy in code comments
2. Add comment in handleFileDownload about no retry mechanism
3. Add comment about single file design choice

For each phase:
1. Read the relevant code sections
2. Implement fix following existing code style
3. Test compilation for both projects
4. Document any behavior changes

## Critical Files

- `rocket/src/main.cpp` - Main rocket firmware logic
- `rocket/src/espnow_comms.cpp` - Rocket ESP-NOW communication
- `ground-station/src/main.cpp` - Ground station relay logic
- `ground-station/src/espnow_comms.cpp` - Ground station ESP-NOW communication

## Verification

After implementing each fix:
1. Compile both projects: `cd rocket && uv run pio run` and `cd ground-station && uv run pio run`
2. Verify no regressions in memory usage
3. For runtime changes (error handling, serial input), manual testing would be required with actual hardware
