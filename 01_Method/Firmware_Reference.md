# Firmware reference

Updated 23 July 2026.

## Active source of truth

The integrated CubeIDE firmware is:

- `Core/Src/main.c`;
- `Core/Inc/smartbias_firmware_profile.h`;
- CAN/INA/sweep modules under `Core/Src/` and `Core/Inc/`;
- PC supervisor `tools/can_iv_host/waveshare_can_a.py`.

`Core/Src/main.c` is profile-driven. The current selector is:

```c
SMARTBIAS_PROFILE_MATRIX_15V_STRESS_150MA_100C
```

and the UART banner is:

```text
FW_CAN_INA_15V_STRESS_018
```

Current compiled values are 15 V/100 mA, 2062 mV base,
Kp/Ki/Kd = 1.20/0.06/0.10, 4.0 mV/degC feed-forward, 150 mA overcurrent and
100 degC overtemperature thresholds. The physical PSU limit remains at or
below 140 mA.

## Final-data firmware identities

| Dataset | Firmware basis | Current source | Key parameters |
|---|---|---|---|
| `final_can_ina/10V_50mA` | `_013` | INA226 only | 50 mA, 1934 mV, PID 0.82/0.035/0.08, FF 0 or 3.0 mV/degC, 80 mA/70 degC |
| `final_can_ina/15V_100mA` | 23 July current `_018` source build | INA226 only | 100 mA, 2062 mV, PID 1.20/0.06/0.10, FF 0 or 4.0 mV/degC, 150 mA/100 degC |

Within each voltage group the PID-no-FF and PID-with-FF modes share the same
target, base and PID gains. The feed-forward gain is the intended control-law
difference.

The host allow-list in `waveshare_can_a.py` is aligned to 1934 mV for 10 V and
2062 mV for 15 V.

## Runtime modes

The binary retains four selectable profiles:

| Profile | Function | Final-analysis status |
|---:|---|---|
| 0 | current PID, no NTC FF | included |
| 1 | temperature-only limiter | implemented but excluded |
| 2 | current PID + NTC FF | included |
| 3 | fixed compiled base | included |

Physical USER-button arming is required. CAN can select an exact compiled
profile, start it after arming, request telemetry and issue STOP. It cannot
raise hard limits, bypass physical arming or clear a latched fault.

## Peripheral configuration

- FDCAN1 peripheral operated in Classic-CAN mode;
- 500 kbit/s, 11-bit standard identifiers, eight-byte payloads;
- PA11 = FDCAN1_RX, PA12 = FDCAN1_TX;
- I2C1 at 100 kHz, PA15 = SCL, PB9 = SDA;
- INA226 address 0x41;
- PA4 = DAC1_OUT1;
- PA0 = NTC ADC;
- PA1 legacy current ADC is deliberately ignored in final INA-only builds;
- PA2/PA3 = LPUART1/XCOM.

## CAN protocol

| ID | Direction | Purpose |
|---:|---|---|
| 0x100 | PC to MCU | command: ping, status, start, abort, point request, STOP |
| 0x101 | PC to MCU | sweep configuration A |
| 0x102 | PC to MCU | sweep configuration B |
| 0x103 | PC to MCU | profile/PID configuration |
| 0x180 | MCU to PC | ACK/result |
| 0x181 | MCU to PC | controller status |
| 0x182 | MCU to PC | INA226 telemetry |
| 0x200/0x201 | MCU to PC | indexed sweep-point pair |
| 0x280 | MCU to PC | event/fault/completion |

The MCU stores up to 64 sweep points locally; the PC retrieves indexed points
after completion to avoid overflowing the USB-CAN-A receive buffer.

## Safety and faults

Active-output faults force DAC to zero and latch until reset:

| Bit | Meaning |
|---:|---|
| 0 | overcurrent |
| 1 | overtemperature |
| 2 | NTC near ADC rail |

Invalid INA226 data while output is active also fails safe. Remote STOP returns
to the idle/DAC-off state but does not clear a hard fault.

## Build and artifact traceability

The final pre-test Debug ELF was built on 23 July 2026 before the final 15 V
CSV acquisition:

```text
Debug/G431_Bringup_v1.elf
SHA-256 862E1880E55F6F51E5FB36AEF76092D13365AA44061F427AC71F4325356BEA0D
```

Associated source hashes:

```text
Core/Inc/smartbias_firmware_profile.h
A70DE0E6A0448AABEB0E1CA6EFB171EF6D58EF145C083E43B9DBD3F78822AC2B

Core/Src/main.c
A09394B3C71139E207C0CFFDA00E14DBB655460BD707AF201BE0F8FC8D7EFFAD

tools/can_iv_host/waveshare_can_a.py
F8A8B78E566E8337FB88DAE07A74F87796E29D7A74F0A23D08472EAF5F88121A
```

Traceability limitation: `firmware_builds/stress/FW_CAN_INA_15V_STRESS_018.*`
was packaged on 22 July and represents the earlier 2040 mV/0.82/0.04/0.10
revision. Its reused `_018` label is ambiguous. Do not use that packaged
artifact to reproduce the 23 July 2062 mV dataset.

## Retained historical firmware

- `_011`/`_012`: calibration-only builds;
- `_014`/`_016`: earlier 15 V 2040 mV matrix revisions;
- `_015`: 10 V 1934 mV revision with a 30 degC limiter;
- generated `firmware_test_variants/`: historical PA1/Rs four-mode matrix;
- 13 V variants: uncalibrated design-only extension.

Always confirm the complete UART banner and numeric locked-basis line after
flashing; a filename alone is not sufficient evidence.
