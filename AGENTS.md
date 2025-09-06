# AGENTS

- Ensure the PlatformIO CLI is installed; tests rely on `pio`.
- Tests use custom Arduino stubs in `test/Arduino.h`; extend them when new Serial features are needed.
- Passthrough boots active; keep `tx_paused` true so host sees only OI bytes until interpreter mode.
- Idle tests may override battery level via `setBatteryPercentOverride` and inspect LED state with `getLedPattern`.
