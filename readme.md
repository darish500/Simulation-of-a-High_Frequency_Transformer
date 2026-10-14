# High-Frequency Single-Phase Inverter: Simulink / SPS Model

A Simulink model of a single-phase, isolated, high-frequency inverter built entirely from
**Simscape Electrical > Specialized Power Systems (SPS)** blocks in MATLAB R2025b. The model
is used to compare a **conventional (5 kHz)** design against a **high-frequency (20 kHz)**
design delivering the same output, and to show what the higher switching frequency buys.

![Block diagram of the proposed inverter](docs/images/fig01_block_diagram.png)

## Table of contents

1. [What this project does](#what-this-project-does)
2. [System overview](#system-overview)
3. [Repository contents](#repository-contents)
4. [Requirements](#requirements)
5. [How to run](#how-to-run)
6. [Model structure](#model-structure)
7. [Parameters and design rules](#parameters-and-design-rules)
8. [Results](#results)
9. [Assumptions and limitations](#assumptions-and-limitations)
10. [Known issues](#known-issues)
11. [Author](#author)

## What this project does

- Builds the inverter in the block diagram above: DC source, DC-DC converter, full-bridge
  MOSFET inverter, T-LCL filter, 1:1 transformer, and a single-phase load with power and
  power-factor measurement, closed with a PI controller.
- Drives every design from **one parameter script** (`inverter.m`), so the conventional and
  high-frequency cases run on the same model by changing a single switch (`case_sel`).
- Logs the key signals and computes **per-stage efficiency**, THD, and component-size
  indicators with `analyze_run.m`, then prints a side-by-side comparison of the two cases.

## System overview

| Stage | Implementation |
|---|---|
| DC source | 48 V DC source with a small series resistance |
| DC-DC converter | Isolated **flyback** (n = 8, D = 0.55); its transformer is the high-frequency transformer |
| Inverter | Single-phase full bridge (Universal Bridge, 2 arms, MOSFET/Diodes) with **unipolar PWM** (3-level output) |
| Filter | **T-LCL** low-pass filter with a damping resistor in series with the shunt capacitor |
| Isolation | 1:1 transformer (Linear Transformer, per-unit parameters) |
| Load | Series RLC load, 1 kW at 0.9 power factor (lagging), 230 V RMS, 60 Hz |
| Control | PI controller regulating output RMS voltage through the modulation index, with feedforward |

**Operating point:** 48 V input, 230 V RMS / 60 Hz output, 1 kW, PF 0.9 lagging.

## Repository contents

```
.
├── high_frequency_inverter.slx   # Simulink model
├── inverter.m                    # all workspace parameters + case switch
├── analyze_run.m                 # post-run analysis and case comparison
├── README.md
├── docs/
│   └── images/                   # figures used in this README
└── report/                       # detailed report (LaTeX)
```

## Requirements

- MATLAB **R2025b** with Simulink
- **Simscape Electrical** (Specialized Power Systems library)

## How to run

1. Open MATLAB and set the project folder as the current folder.
2. Open `inverter.m` and choose the design:
   ```matlab
   case_sel = 1;   % conventional: 5 kHz inverter and flyback switching
   case_sel = 2;   % high-frequency: 20 kHz inverter and flyback switching
   ```
3. Run the parameter script:
   ```matlab
   inverter
   ```
4. Open `high_frequency_inverter.slx` and **run the model**. Wait until it finishes.
5. Analyse the run:
   ```matlab
   analyze_run
   ```
6. Repeat steps 2 to 5 for the other case. After the second case, `analyze_run` prints the
   comparison table automatically (each run is saved as `results_case1.mat` /
   `results_case2.mat`).

> **Order matters.** Always run `inverter`, then the model, then `analyze_run`.
> `analyze_run` checks that the logged data matches the current case and stops with a message
> if it does not (for example, if the model was not rerun after switching `case_sel`).

**Signals that must be logged** (names must match exactly): `V_in`, `I_in`, `V_dc`, `I_dc`,
`V_load`, `I_load`. They appear in `out.logsout` after a run.

## Model structure

Every part of the circuit is its own labelled subsystem, and **no block contains a typed-in
number**: every value comes from the MATLAB workspace.

| Subsystem | Contents |
|---|---|
| `DC_Source` | DC voltage source, series resistance, input voltage/current measurement |
| `DC_DC_Converter` | Flyback: MOSFET, Linear Transformer (HF transformer), diode, output capacitor, gate pulse, grounding resistor |
| `H_Bridge_Inverter` | Universal Bridge (2 arms), inverter output voltage measurement |
| `PWM_Generator` | Sine reference, triangle carrier, comparators, complementary gate signals |
| `T_LCL_Filter` | Two series RL branches and a shunt RC branch (damped) |
| `HF_Transformer_1to1` | 1:1 isolation transformer with a grounded secondary reference |
| `Load_and_PF_Measurement` | Series RLC load, V/I measurement, active and reactive power, RMS, THD, power factor |
| `PI_Controller` | Voltage error, discrete PI, feedforward modulation index, limiter |

**Gate signal order** for the bridge is `[S1 S2 S3 S4]` = [arm 1 top, arm 1 bottom, arm 2 top,
arm 2 bottom]. Arm 2 is driven by the inverted reference against the same carrier, which gives
the three output levels (+Vdc, 0, -Vdc).

![Model overview](docs/images/fig02_model_overview.png)

## Parameters and design rules

All parameters live in `inverter.m`. The main design rules are:

| Item | Rule |
|---|---|
| Time step | `Ts_power = 1/(spp * max(fsw, fsw_dc))`, with `spp = 100` samples per switching period |
| Flyback magnetizing inductance | `Lm_fly = Vin*D_fly/(fsw_dc*dIm)`, with ripple `dIm = 24.5 A` |
| Flyback leakage inductance | `Llk_fly = 0.01*Lm_fly` (1 % of the magnetizing inductance) |
| Flyback gate | `N_fly = round(1/(fsw_dc*Ts_power))` samples per period, `W_fly = round(D_fly*N_fly)` on-samples |
| Filter inductor L1 | `L1 = Vdc/(16*fsw*dI)`, with `dI = 0.2*Ipk` (20 % current ripple) |
| Filter inductor L2 | `L2 = L1/2` |
| Filter capacitor | `Cf = 0.10*(P_load/PF_ref)/(2*pi*f_out*Vout_rms^2)` (10 % of rated reactive power) |
| Damping resistor | `R_d = 0.3*sqrt((L1+L2)/Cf)` |
| Feedforward modulation | `m_ff = sqrt(2)*Vout_rms/Vdc` |
| PI gains | `Kp = 0.002`, `Ki = 0.03`, trim limit `m_trim = 0.1`, `m_max = 0.95` |
| Load reactive power | `Q_load = P_load*tan(acos(PF_ref))` |

The two cases differ **only** in the switching frequencies. Everything else, including the PI
gains and the filter capacitor, is the same, so the comparison isolates the effect of
frequency.

## Results

Both designs were run to steady state with the same controller and load. Values are measured
over the last five output cycles by `analyze_run.m`.

| | 5 kHz (conventional) | 20 kHz (high-frequency) |
|---|---|---|
| Output RMS voltage | 227.8 V | 230.0 V |
| Output power | 979.8 W | 999.5 W |
| Power factor | 0.90 | 0.90 |
| Total efficiency | 90.7 % | 91.2 % |
| Flyback stage efficiency | 93.2 % | 93.4 % |
| Bridge + filter + transformer efficiency | 97.3 % | 97.7 % |
| THD (harmonics 2 to 50) | 1.19 % | 1.68 % |
| Filter inductors L1 / L2 | 3.66 / 1.83 mH | **0.91 / 0.46 mH** |
| Inductor stored energy | 0.128 J | **0.032 J** |
| Flyback magnetizing inductance | 215.5 uH | **53.9 uH** |
| Flyback volt-seconds (core size indicator) | 5280 uV.s | **1320 uV.s** |

**Summary:** both designs deliver the same output with equal efficiency, but the 20 kHz design
needs **about 4x smaller** filter inductors and flyback transformer.

![Output voltage, 5 kHz and 20 kHz](docs/images/fig10_output_voltage_comparison.png)

## Assumptions and limitations

- **Device data are placeholders** (MOSFET on-resistances, diode drop, winding resistances,
  leakage rule). Absolute efficiency depends on them, so compare the two cases rather than
  relying on the absolute figure.
- The SPS switches model **conduction loss only**. Switching loss is estimated separately in
  `inverter.m` (a few watts at these settings) and core loss is not modelled in detail.
- THD is computed over harmonics 2 to 50 of the 60 Hz fundamental (up to 3 kHz) and does
  **not** include switching-frequency ripple.
- The 5 kHz run had not fully settled at the end of the simulation (227.8 V against 230 V),
  which slightly lowers its output power and efficiency.
- Load, ratings and frequencies are placeholders until confirmed by the client.

## Known issues

- A Simulink warning about the buffer size of the `Mean` block's internal Transport Delay
  appears during simulation. It does not affect results and only matters for code generation.
- The analysis script uses variable names that do not clash with the model workspace
  (`Vin` and `Vdc` must stay as the scalar 48 V and 400 V).

## Author

Khalid - [github.com/darish500](https://github.com/darish500)