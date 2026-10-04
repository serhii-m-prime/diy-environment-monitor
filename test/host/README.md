# Sensor regression tests

Run from PowerShell with a C++17 host compiler:

```powershell
./test/host/run.ps1
# Or select a compiler explicitly:
./test/host/run.ps1 -Cxx C:/msys64/ucrt64/bin/g++.exe
```

Compiles the production PIR, CO2 and light sensor adapters and State header
against small Arduino and sensor-library stubs. Tests cover short HIGH/LOW
PIR pulses, confirmed transitions, UART timeout/CRC errors, checking errors
before the next UART command, failed temperature reads, invalid light values,
sensor recovery, N/A diagnostics and retaining brightness input across failures.
Generated files go into `.pio/host-tests`.

These checks do not exercise real UART/I2C, display rendering or hardware timing.
