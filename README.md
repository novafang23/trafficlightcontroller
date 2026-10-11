# Traffic Light Controller (Verilog)

A Verilog FSM-based traffic light controller for a 4-way intersection with pedestrian walking phases.

## State Cycle

The controller cycles deterministically through 6 sequential states:

```
NS_GREEN → NS_YELLOW → EW_WALK → EW_GREEN → EW_YELLOW → NS_WALK → (repeat)
```

1. **`S0_NS_G` (North-South Green)**: NS Green (`3'b001`), EW Red (`3'b100`). Duration: `T_GREEN` (default 10 cycles).
2. **`S1_NS_Y` (North-South Yellow)**: NS Yellow (`3'b010`), EW Red (`3'b100`). Duration: `T_YELLOW` (default 4 cycles).
3. **`S2_EW_W` (East-West Pedestrian Walk)**: NS Red (`3'b100`), EW Red (`3'b100`), `walk_ew = 1`. Duration: `T_RED` (default 6 cycles).
4. **`S3_EW_G` (East-West Green)**: NS Red (`3'b100`), EW Green (`3'b001`). Duration: `T_GREEN` (default 10 cycles).
5. **`S4_EW_Y` (East-West Yellow)**: NS Red (`3'b100`), EW Yellow (`3'b010`). Duration: `T_YELLOW` (default 4 cycles).
6. **`S5_NS_W` (North-South Pedestrian Walk)**: NS Red (`3'b100`), EW Red (`3'b100`), `walk_ns = 1`. Duration: `T_RED` (default 6 cycles).

## Signal Encoding

- `ns_lights` / `ew_lights`: 3-bit one-hot bus `[2:0]` formatted as `{RED, YELLOW, GREEN}`:
  - `3'b001` : Green
  - `3'b010` : Yellow
  - `3'b100` : Red
- `walk_ns` / `walk_ew`: 1-bit active-high pedestrian walk indicators.
- `rst_n`: Synchronous active-low reset.

## Project Structure

```
TRAFFIC LIGHT CONTROLLER/
├── rtl/
│   └── Trafficlightcontroller.v   # RTL: Top, FSM, and cycle-accurate counter
├── tb/
│   └── testbench.v                # Self-checking testbench with safety assertions
├── README.md                      # Documentation
└── .gitignore                     # Simulation artifacts (*.vvp, *.vcd)
```

## Running Simulation

Compile and simulate using **Icarus Verilog**:

```bash
# 1. Compile RTL and Testbench
iverilog -g2005-sv -o sim.vvp rtl/Trafficlightcontroller.v tb/testbench.v

# 2. Run simulation
vvp sim.vvp

# 3. View waveforms (optional)
gtkwave simulation.vcd
```
