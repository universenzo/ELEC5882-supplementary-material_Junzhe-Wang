# Hardware reference

Updated 23 July 2026.

## As-built final architecture

```text
Current-limited PSU+
  -> INA226 INPUT -> INA226 OUTPUT
  -> external RL (10 Ohm)
  -> IRLZ44N drain/source power path
  -> external Rs / common return
  -> PSU-

PA0 <- 10 kOhm NTC divider
PA4 -> DAC gate-control path
PA11/PA12 <-> SN65HVD230 <-> CANH/CANL <-> USB-CAN-A <-> PC
PA15/PB9 <-> M5Stack INA226-1A I2C
PA2/PA3 <-> XCOM UART
```

Required common reference:

```text
PSU- = PWR_GND = SIG_GND = STM32 GND = CAN/INA host GND
```

The final CAN/INA builds use INA226 as the only current feedback, protection
and telemetry source. PA1 is not combined with INA226 and is deliberately
ignored. The historical PA1/external-Rs path remains useful only as earlier
validation and optional manual cross-check evidence.

## Pin map

| Signal | MCU pin | Final status |
|---|---|---|
| NTC ADC | PA0 / ADC1_IN1 | working |
| Legacy Rs ADC | PA1 / ADC1_IN2 | electrically available; disabled in final INA-only firmware |
| UART TX/RX | PA2 / PA3 | working |
| DAC gate command | PA4 / DAC1_OUT1 | working |
| USER button | PC13 | required physical arm |
| FDCAN1 RX/TX | PA11 / PA12 | working in Classic-CAN mode |
| I2C1 SCL/SDA | PA15 / PB9 | working at 100 kHz |
| SWD | PA13 / PA14 | programming/debug |

## External modules

| Module | Used configuration |
|---|---|
| Waveshare USB-CAN-A Model A | Classic CAN 2.0A/B only; normal mode; 500 kbit/s; USB serial 2 Mbaud; variable-length protocol |
| Waveshare SN65HVD230 CAN Board | 3.3 V; MCU TX to board TX input and board RX output to MCU RX; CANH/CANL bus |
| M5Stack Unit INA226-1A | 5 V host supply; address 0x41; INPUT toward PSU; OUTPUT toward load |

For the two-node bench bus, CANH connects to CANH and CANL to CANL. With both
terminations fitted, the unpowered resistance across CANH/CANL was about
60 Ohm. The SN65HVD230 board must not be powered from 5 V.

## Main hardware values

| Item | Value |
|---|---|
| MCU board | NUCLEO-G431RB |
| MOSFET | IRLZ44N |
| External load | 10 Ohm, 5 W |
| External sense resistor | approximately 1.2 Ohm; historical/verification path |
| NTC | B57891M0103K000, 10 kOhm, beta about 3950 K |
| NTC top resistor | measured 9.88 kOhm |
| Gate series / pulldown | 100 Ohm / 47 kOhm |
| DAC range | 1400-2100 mV active command; zero when idle/faulted |

## Verified measurements

- early PCB gate path: about 39 mV before USER and 1.877 V after USER;
- gate-disconnected dry commands of 1.50/1.60/1.70 V produced approximately
  1.45/1.55/1.65 V at PA4, returning to about 33 mV off;
- NUCLEO 3V3 rail measured about 3.31 V;
- at approximately 36 mA, 43.8 mV across the 1.2 Ohm path was consistent with
  the expected current;
- at 10 V/approximately 101 mA, measured values included
  `GATE_DRV=1.88 V`, `MOS_DRAIN=8.99 V`, `MOS_S=122.5 mV`;
- INA226 identity/configuration and powered voltage/current telemetry passed;
- real Classic-CAN ping/ACK/status exchange passed with zero adapter error
  counters in the recorded check;
- the powered 10 V sweep completed without a fault or CAN drop.

## Interpretation limits

- INA226 is the final quantitative current source, but paired CAN current
  fields are not two independent sensors.
- NTC temperature is package-adjacent and delayed; it is not junction
  temperature.
- DAC telemetry is the commanded PA4 voltage, not measured VGS.
- Estimated VDS from bus voltage and known resistors is not a direct drain-
  source voltage measurement.

## Safety boundary

- 10 V experiments: 90 mA physical PSU limit.
- 15 V experiments: no more than 140 mA physical PSU limit.
- Keep RL in series for all powered tests.
- Keep NTC electrically insulated from the MOSFET drain tab.
- Confirm DAC below 0.1 V before arming.
- Stop on PSU constant-current indication, invalid INA data, unexpected
  heating, unstable current, a fault or uncertain wiring.

No further powered tests are currently planned. These values are retained for
reproducibility and viva discussion.
