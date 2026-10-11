# Smart Actuated Traffic Light Controller (Verilog)

A production-grade, Verilog FSM-based **Smart Actuated Traffic Light Controller** for a 4-way intersection (North-South arterial main street and East-West side street). Designed with dynamic vehicle loop sensors, pedestrian call push-buttons, gap-out timers, and conditional phase skipping for high-efficiency smart city intersections.

---

## Key Features

1. **Dynamic Inductive Loop Sensors (`car_sensor_ns`, `car_sensor_ew`)**:
   - Detects vehicle presence in real-time.
   - **Arterial Resting in Green**: If no vehicles or pedestrians are waiting on the side street (EW), the main street (NS) remains in Green indefinitely, eliminating needless stops for main-street commuters.
2. **Dynamic Green Extension (Gap-Out / Max-Out)**:
   - **Minimum Green (`T_GREEN_MIN`)**: Guarantees queued vehicles have adequate time to accelerate into the intersection before any transition can occur.
   - **Gap-Out Early Transition**: When traffic gaps out (no more vehicles detected over the sensor), the controller transitions early to Yellow instead of wasting green time on empty lanes.
   - **Max-Out Protection (`T_GREEN_MAX`)**: When dense continuous traffic is detected on both streets, the controller extends green up to `T_GREEN_MAX` and then terminates to prevent phase starvation.
3. **Pedestrian Actuation & Pulse Latching (`ped_call_ns`, `ped_call_ew`)**:
   - Captures momentary (even 1-cycle) button presses using dedicated latch registers.
   - **Conditional Walk Skipping**: Pedestrian walk phases (`EW_WALK`, `NS_WALK`) are completely skipped in cycles where no button was pressed.
   - Automatically clears the latched call once the requested walk interval is serviced.
4. **Side-Street Phase Skipping**:
   - If no side-street vehicles or pedestrians are detected, side-street phases (`EW_WALK`, `EW_GREEN`, `EW_YELLOW`) are skipped entirely.
5. **Fail-Safe Safety Invariants**:
   - **Mutually Exclusive Greens**: Dual green is physically impossible.
   - **All-Red Pedestrian Protection**: All vehicular signals are forced to Red (`3'b100`) whenever a pedestrian walk signal (`walk_ns` or `walk_ew`) is active.
   - **Dual Walk Prevention**: Only one pedestrian crossing phase can be active at any time.
   - **One-Hot Vehicular Encoding**: Lights are strictly encoded on 3-bit buses as `{RED, YELLOW, GREEN}` (`3'b001` Green, `3'b010` Yellow, `3'b100` Red).
   - **Synchronous Active-Low Reset (`rst_n`)**: Instantly and safely forces the intersection to North-South Green and clears all pending actuation latches.

---

## State Diagram & Transitions

```
                    +--------------------+
                    |     S0_NS_G        | <--------------------------------------------+
                    | Main Street Green  |                                              |
                    +--------------------+                                              |
                              |                                                         |
                       (side demand &                                                   |
                    gap-out or max-out)                                                 |
                              v                                                         |
                    +--------------------+                                              |
                    |     S1_NS_Y        |                                              |
                    | Main Street Yellow |                                              |
                    +--------------------+                                              |
                       /       |        \                                               |
             (ped_ew) / (car_ew) \ (ped_ns only)                                        |
                     /         |    \                                                   |
                    v          |     \                                                  |
            +--------------+   |      |                                                 |
            |   S2_EW_W    |   |      |                                                 |
            |   EW Walk    |   |      |                                                 |
            +--------------+   |      |                                                 |
               /     \         |      |                                                 |
       (car_ew)   (!car_ew)    |      |                                                 |
             /         \       |      |                                                 |
            v           \      v      |                                                 |
    +--------------+     \ +--------------+                                             |
    |   S3_EW_G    |      \|   S5_NS_W    |                                             |
    |  Side Green  |       |   NS Walk    |---------------------------------------------+
    +--------------+       +--------------+                                             |
            |                     ^                                                     |
       (gap-out or                | (ped_ns)                                            |
         max-out)                 |                                                     |
            v                     |                                                     |
    +--------------+              |                                                     |
    |   S4_EW_Y    |--------------+                                                     |
    | Side Yellow  |--------------------------------------------------------------------+
    +--------------+                     (!ped_ns)
```

### State Definitions & Transition Priority

| State Code | Identifier | Description | Vehicular NS | Vehicular EW | Walk NS | Walk EW | Conditional Next-State Logic |
|:---|:---|:---|:---:|:---:|:---:|:---:|:---|
| `3'd0` | `S0_NS_G` | North-South Green | `3'b001` (Green) | `3'b100` (Red) | `0` | `0` | $\ge T_{MIN}$: gaps out early if `!car_ns_pending` or maxes out at $T_{MAX}$; rests indefinitely if no conflicting demand |
| `3'd1` | `S1_NS_Y` | North-South Yellow | `3'b010` (Yellow) | `3'b100` (Red) | `0` | `0` | Fixed $T_{YELLOW}$ cycles $\rightarrow$ `S2_EW_W` if `ped_ew`; `S3_EW_G` if `car_ew`; `S5_NS_W` if `ped_ns`; else `S0_NS_G` |
| `3'd2` | `S2_EW_W` | East-West Pedestrian Walk | `3'b100` (Red) | `3'b100` (Red) | `0` | `1` | Fixed $T_{RED}$ cycles $\rightarrow$ `S3_EW_G` if `car_ew_pending`; else `S5_NS_W` if `ped_ns_pending`; else skips to `S0_NS_G` |
| `3'd3` | `S3_EW_G` | East-West Green | `3'b100` (Red) | `3'b001` (Green) | `0` | `0` | $\ge T_{MIN}$: gaps out early if `!car_ew_pending` or maxes out at $T_{MAX}$ on continuous traffic $\rightarrow$ `S4_EW_Y` |
| `3'd4` | `S4_EW_Y` | East-West Yellow | `3'b100` (Red) | `3'b010` (Yellow) | `0` | `0` | Fixed $T_{YELLOW}$ cycles $\rightarrow$ `S5_NS_W` if `ped_ns_pending`; else skips directly to `S0_NS_G` |
| `3'd5` | `S5_NS_W` | North-South Pedestrian Walk | `3'b100` (Red) | `3'b100` (Red) | `1` | `0` | Fixed $T_{RED}$ cycles $\rightarrow$ unconditional return to `S0_NS_G` (guaranteeing vehicular green) |

---

## Module Ports & Parameters

### Parameters

| Parameter | Type | Default | Description |
|:---|:---:|:---:|:---|
| `BITS` | `integer` | `16` | Width of internal cycle counter registers |
| `T_GREEN_MIN` | `[BITS-1:0]` | `16'd6` | Minimum guaranteed green time before gap-out can trigger ($\ge 1$) |
| `T_GREEN` | `[BITS-1:0]` | `16'd15` | Backward-compatibility alias for green duration (defaults `T_GREEN_MAX`) |
| `T_GREEN_MAX` | `[BITS-1:0]` | `T_GREEN` | Maximum allowable green extension during dense traffic ($\ge T_{GREEN\_MIN}$) |
| `T_YELLOW` | `[BITS-1:0]` | `16'd4` | Vehicular yellow clearance interval ($\ge 1$) |
| `T_RED` | `[BITS-1:0]` | `16'd6` | Pedestrian walk interval (vehicular all-red, $\ge 1$) |

> **Parameter Elaboration Assertions**: The RTL includes simulation assertions that automatically enforce $1 \le T_{GREEN\_MIN} \le T_{GREEN\_MAX}$, $T_{YELLOW} \ge 1$, and $T_{RED} \ge 1$ to catch invalid overrides and prevent counter underflow.

### Top-Level Port List

| Port Name | Direction | Width | Description |
|:---|:---:|:---:|:---|
| `clk` | Input | `1` | System clock (100 MHz in simulation) |
| `rst_n` | Input | `1` | Synchronous active-low reset |
| `car_sensor_ns` | Input | `1` | North-South vehicle presence / passage sensor |
| `car_sensor_ew` | Input | `1` | East-West vehicle presence / passage sensor |
| `ped_call_ns` | Input | `1` | North-South pedestrian push-button (latched on posedge) |
| `ped_call_ew` | Input | `1` | East-West pedestrian push-button (latched on posedge) |
| `ns_lights` | Output | `3` | North-South light bus `{RED, YELLOW, GREEN}`: `001`=G, `010`=Y, `100`=R |
| `ew_lights` | Output | `3` | East-West light bus `{RED, YELLOW, GREEN}`: `001`=G, `010`=Y, `100`=R |
| `walk_ns` | Output | `1` | North-South active-high pedestrian walk lamp |
| `walk_ew` | Output | `1` | East-West active-high pedestrian walk lamp |

---

## Project Structure

```
TRAFFIC LIGHT CONTROLLER/
├── rtl/
│   └── Trafficlightcontroller.v   # RTL: Top, Actuated FSM, and Cycle Counter
├── tb/
│   └── testbench.v                # Self-checking testbench (17 scenarios + safety checkers)
├── README.md                      # Architecture, timing, and run guide
└── .gitignore                     # Simulation artifacts (*.vvp, *.vcd)
```

---

## Running Simulation & Verification

Compile and simulate using **Icarus Verilog** (`iverilog`) and `vvp`:

```bash
# 1. Compile RTL and Testbench with strict warnings enabled
iverilog -Wall -g2005-sv -o sim.vvp rtl/Trafficlightcontroller.v tb/testbench.v

# 2. Execute simulation
vvp sim.vvp

# 3. (Optional) Inspect waveforms in GTKWave
gtkwave simulation.vcd
```

### Testbench Coverage Scenarios

The included self-checking testbench (`tb/testbench.v`) automatically exercises 17 comprehensive scenarios with continuous safety assertion monitoring (275 cycle-accurate invariant checks):

1. **Scenario 0 (Reset & Init)**: Verifies safe initial state (`S0_NS_G`) during synchronous active-low reset.
2. **Scenario 1 (Main Street Resting Green)**: Proves main street rests in green for 25+ cycles when no side demand is detected.
3. **Scenario 2 (NS Gap-Out Early Transition)**: Confirms NS green terminates at cycle 8 (< 15) when the vehicle sensor drops.
4. **Scenario 3 (EW Walk Skipping)**: Verifies `EW_WALK` is skipped when no button is pressed; transitions directly to `EW_GREEN`.
5. **Scenario 4 (EW Gap-Out Early Transition)**: Confirms EW green terminates at `T_GREEN_MIN` (6 cycles) when traffic gaps out.
6. **Scenario 5 (NS Walk Skipping)**: Verifies `NS_WALK` is skipped when no button is pressed; transitions directly to `NS_GREEN`.
7. **Scenario 6 (Dense Traffic Max-Out)**: Continuous traffic on both approaches extends green to `T_GREEN_MAX` (15 cycles) before maxing out.
8. **Scenario 7 (EW Pedestrian Actuation)**: Momentary 1-cycle button pulse latches and services `EW_WALK` for `T_RED` cycles with all-red vehicular lights.
9. **Scenario 8 (NS Pedestrian Actuation)**: Momentary 1-cycle button pulse latches and services `NS_WALK` for `T_RED` cycles with all-red vehicular lights.
10. **Scenario 9 (Mid-Operation Synchronous Reset)**: Verifies dynamic reset assertion restores `S0_NS_G` and clears all pending latches within 1 clock cycle.
11. **Scenario 10 (Side-Street Vehicle Call Latching)**: 1-cycle pulse on side-street loop while red latches the call and triggers service.
12. **Scenario 11 (Simultaneous Pedestrian Calls)**: Both buttons pressed simultaneously are arbitrated and serviced sequentially without safety conflicts.
13. **Scenario 12 (Main-Street Vehicle Call Latching)**: 1-cycle pulse on main-street loop while side street is green latches call and guarantees arterial return.
14. **Scenario 13 (Side-Street Phase Skipping)**: When resting in green and only an NS pedestrian calls, skips ALL EW phases (`EW_WALK`, `EW_GREEN`, `EW_YELLOW`) directly to `NS_WALK` and back to `NS_GREEN`.
15. **Scenario 14 (Side-Street Vehicular Green Skipping)**: When resting in green and only an EW pedestrian calls (no side cars), services `EW_WALK` and completely skips `EW_GREEN` and `EW_YELLOW`.
16. **Scenario 15 (Anti-Starvation Under Stuck Button)**: Continuously held pedestrian call button is serviced without starving vehicular traffic (minimum green guaranteed each cycle).
17. **Scenario 16 (Parameter Cascade & Backward Compatibility)**: Verifies legacy parameter override `T_GREEN` properly cascades to `T_GREEN_MAX`.
