# L103 + DRV8313 + AS5600 Joint Motor Driver

[中文](README.md) | [English](README.en.md)

A joint-motor driver board built around **CH32L103 + DRV8313 + AS5600**, with field-oriented-control
firmware, a desktop tuning GUI, and a matching **two-stage planetary gearbox (rev J, 25:1)**.

![demo](media/demo.gif)

> Live demo: power-up, CAN link to the GUI, enable, index moves, continuous speed. The screen and
> background in the clip were masked by AI.

MIT License.

---

## Layout

| Path | Contents |
|---|---|
| `hardware/` | KiCad 10 project (schematic + PCB), ready-to-order Gerbers in `gerber/`, pin map and netlist checks in `docs/` |
| `firmware/` | CH32L103 FOC firmware: Makefile build plus a MounRiver Studio 2 project |
| `tools/` | Command-line helpers: CAN/UART commands, motion sequences, speed sweeps, build and flash scripts |
| `gearbox-J/` | Two-stage planetary gearbox rev J (25:1): STEP / STL, renders, dimension drawings, assembly notes |
| `docs/` | Firmware design write-up (the USB-to-CAN bridge half of the same board) |
| `media/` | Demo GIF and MP4 |

The GUI lives in its own repo: [`l103-servo-gui`](https://github.com/bjthemost-hub/l103-servo-gui)
(the same GUI also drives the DRV8316C board).

---

## Hardware

Design values, from the netlist-verified header comment in `firmware/User/board.h`:

| Item | Value |
|---|---|
| MCU | `CH32L103K8U6` (QFN-32) |
| Gate driver | `DRV8313` (three half-bridges, per-phase EN1/2/3) |
| PWM | `TIM3` centre-aligned, **20 kHz** -> PA6 / PA7 / PB0 |
| Enable | PB3 / PB4 / PB5 -> EN1 / EN2 / EN3 |
| Fault | `nFAULT` -> PC14, 10 kOhm pull-up to the **MCU-side** 3V3 |
| Current sense | `INA240A2` x2, 50 V/V over 10 mOhm = **0.5 V/A**, roughly +/-3.3 A -> PA1 / PA4 |
| Encoder | `AS5600` on I2C1: **PB6 = SCL / PB7 = SDA**, 12-bit (4096 CPR) |
| CAN | `SN65HVD230`, PB8 = RX / PB9 = TX, **1 Mbps** |
| Debug UART | UART1: PA9 = TX / PA10 = RX |
| Bus | 8-60 V per the DRV8313 datasheet; firmware limits it to **8-36 V**; designed around 24 V |

> The `nFAULT` pull-up lives on the MCU side. A high level only means the driver is not pulling the
> line low; it does not prove the driver chip is healthy or even present.

## Firmware

- 20 kHz centre-aligned PWM. Open-loop angle advance runs in a 3 kHz ISR; the closed loop runs in the
  main loop because it has to read the encoder.
- AS5600 over I2C1, 12-bit raw angle, with multi-turn accumulation and speed estimation in firmware.
- Pole pairs, stall threshold and bus voltage are stored in a Flash config page and restored at boot;
  compile-time defaults are used when the page is blank. Default motor `HT4310`, **14 pole pairs**.
- CAN at 1 Mbps (node 1-63) or UART at 115200, same command set. Full command reference in
  `firmware/命令表与使用说明_UART与CAN_2026-09-28.md` (Chinese).

```bash
make -B TOOLCHAIN_PATH="<MounRiver>/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC"
```

> **Flashing erases the whole chip, including the parameter page.** Re-enter pole pairs / bus voltage
> and re-run alignment afterwards.

> **Never enable `-flto`.** It lets the weak default handlers in the startup file replace the real ISRs:
> it still links, but the board comes up completely dead (no CAN, no UART output).

## Gearbox (rev J, 25:1)

Two stages, three planets per stage, and both stages share the **same planet part** (six copies), so a
first-stage planet cannot be installed in the second stage by mistake. Tooth counts 12 / 18 / 48,
module 0.95, overall envelope Ø54.811 x 27.10 mm, ten printed parts. See `gearbox-J/README.md`.

> Those are design/calculated values (CAD plus closed-form checks); no printed sample was measured
> for backlash.

## Safety

- Development-board grade hardware, no UL/CE certification. Check your bus and thermals.
- **Never let a screw or the housing touch GND** - we destroyed two DRV8313 and one Si2319 that way.
- Always start with **open loop** (no feedback) to confirm direction, phase current and that no phase
  is dead, before closing the loop.
- Stop immediately on no-rotation, vibration, or speed that does not track the setpoint. Do not just
  raise the voltage.
- `A` (alignment) forcibly drags the rotor; the shaft must be free to turn.

## AI assistance

The hardware design, firmware, desktop GUI, gearbox models and documentation were produced with
**substantial AI assistance** (OpenAI Codex / GPT models), including the GUI screenshot used as the
cover image, which was captured by an AI agent operating the program. All AI-generated code and text
was reviewed by a human and verified on real boards and benches. AI is a tool, not an author: design
decisions, acceptance criteria and conclusions are the human's responsibility.

## License

MIT, see `LICENSE`. Hardware design files and firmware sources are released under the same terms.