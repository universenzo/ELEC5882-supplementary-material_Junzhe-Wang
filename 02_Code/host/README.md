# USB-CAN-A host tool

`waveshare_can_a.py` implements the Waveshare USB-CAN-A variable-length serial
protocol directly. The adapter is a USB-serial CAN device, not SocketCAN.
Close `USB-CAN-A_TOOL_2.12` before running the Python host.

## Final hardware configuration

- USB serial: 2 Mbaud;
- CAN: normal mode, 500 kbit/s;
- standard 11-bit frames;
- variable-length adapter protocol;
- STM32/SN65HVD230/USB-CAN-A two-node bus.

## Tests

```powershell
python tools/can_iv_host/waveshare_can_a.py self-test
python -m unittest discover -s tools/can_iv_host -p "test_*.py"
```

Hardware plotting requires PySerial and Matplotlib.

## Basic communication

```powershell
python tools/can_iv_host/waveshare_can_a.py --port COM5 --configure-adapter ping
python tools/can_iv_host/waveshare_can_a.py --port COM5 --configure-adapter status
python tools/can_iv_host/waveshare_can_a.py --port COM5 --configure-adapter stop
```

## Final matrix command

```powershell
python tools/can_iv_host/waveshare_can_a.py --port COM5 --configure-adapter `
  matrix-demo 10 pid-ff --repeat 1 --seconds 600
```

Final retained modes are:

- `pid-no-ff`;
- `pid-ff`;
- `open-loop`.

`temp-limiter` remains available for firmware diagnostics but is excluded from
the final 18-run analysis.

The host allow-list uses:

- 10 V: 50 mA, 1934 mV, 0 or 3.0 mV/degC FF;
- 15 V: 100 mA, 2062 mV, 0 or 4.0 mV/degC FF.

It creates deterministic filenames below
`lab_data/final_can_ina/<voltage>V_<target>mA/`, records live telemetry, saves
CSV/PNG and sends STOP during cleanup. Automatic exit after `--seconds` is
normal behaviour.

## Automated sweep

The tool configures a bounded local MCU sweep, waits for physical arming,
starts it, then retrieves indexed point pairs. Indexed retrieval avoids
overrunning the adapter receive buffer.

Reported values distinguish direct telemetry from estimates:

- INA current and bus voltage: measured by INA226;
- DAC: commanded value;
- VDS/VGS fields: calculated estimates unless independently measured.

The result is a fixed-supply transfer/load line, not a complete ID-VDS family.

## Safety

The PC cannot bypass physical arming, change compiled hard limits or clear a
latched fault. Retain 90 mA physical PSU limiting at 10 V and no more than
140 mA at 15 V.
