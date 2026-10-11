# Actuated Traffic Light Controller (Verilog)

FSM-based controller for a 4-way intersection: NS is the main road, EW is the side street. It reacts to vehicle sensors and pedestrian buttons instead of running a fixed timer loop.

## What it does

- **NS rests in green** when nobody is waiting on EW and nobody pressed a ped button.
- **Min / max green:** green is held for at least `T_GREEN_MIN` cycles. After that it gaps out (drops to yellow) as soon as the sensor on that road goes quiet, or is cut off at `T_GREEN_MAX` if traffic never lets up.
- **Calls are latched**, so a 1-cycle pulse on a car sensor or ped button is not lost. A latch clears when its phase is served.
- **Phases get skipped** when there is nothing to serve: no ped call means no walk phase, no EW demand means no EW green/yellow.
- **Sync active-low reset** (`rst_n`) puts it back in NS green and clears the latches.

Inputs are expected to be synchronised and debounced outside this block.

## States

```
S_NS_G -> S_NS_Y -> S_EW_W -> S_EW_G -> S_EW_Y -> S_NS_W -> S_NS_G
```

| State | NS | EW | Walk | Leaves when |
|:--|:-:|:-:|:-:|:--|
| `S_NS_G` | G | R | - | after min green, if someone is waiting on EW or a ped called, and NS gapped out or hit max |
| `S_NS_Y` | Y | R | - | `T_YELLOW`; next is `S_EW_W` if EW ped, else `S_EW_G` if EW car, else `S_NS_W` if NS ped, else back to `S_NS_G` |
| `S_EW_W` | R | R | EW | `T_RED`; next is `S_EW_G` if EW car, else `S_NS_W` if NS ped, else `S_NS_G` |
| `S_EW_G` | R | G | - | after min green, EW gapped out or hit max |
| `S_EW_Y` | R | Y | - | `T_YELLOW`; next is `S_NS_W` if NS ped, else `S_NS_G` |
| `S_NS_W` | R | R | NS | `T_RED`, always back to `S_NS_G` |

Lights are 3-bit `{R,Y,G}`: `001` green, `010` yellow, `100` red.

## Parameters

| Name | Default | Notes |
|:--|:-:|:--|
| `BITS` | 16 | timer width |
| `T_GREEN_MIN` | 6 | must be >= 1 |
| `T_GREEN_MAX` | `T_GREEN` (15) | must be >= `T_GREEN_MIN` |
| `T_GREEN` | 15 | old name, just feeds `T_GREEN_MAX` |
| `T_YELLOW` | 4 | must be >= 1 |
| `T_RED` | 6 | ped walk time, must be >= 1 |

Bad values are caught at time 0 in simulation and the run stops.

## Ports (top)

| Port | Dir | Description |
|:--|:-:|:--|
| `clk`, `rst_n` | in | clock, sync active-low reset |
| `car_sensor_ns`, `car_sensor_ew` | in | vehicle presence |
| `ped_call_ns`, `ped_call_ew` | in | ped buttons |
| `ns_lights[2:0]`, `ew_lights[2:0]` | out | `{R,Y,G}` |
| `walk_ns`, `walk_ew` | out | walk lamps |

## Layout

```
rtl/Trafficlightcontroller.v   counter, fsm, top
tb/testbench.v                 self-checking tb
```

## Simulating

```bash
iverilog -Wall -g2005-sv -o sim.vvp rtl/Trafficlightcontroller.v tb/testbench.v
vvp sim.vvp
gtkwave simulation.vcd     # optional
```

Expected tail of the output: `275 checks, 0 errors` then `PASSED`.

## Testbench

Always-on checks (every clock after reset): never both greens, no non-red on both roads, all cars red during any walk, never both walks, lights always one-hot.

Directed scenarios:

| # | What |
|:-:|:--|
| 1 | NS rests in green with no demand |
| 2 | NS gap-out |
| 3 | EW walk skipped without a ped call |
| 4 | EW gap-out at min green |
| 5 | NS walk skipped without a ped call |
| 6 | dense traffic, both greens run to max |
| 7 | EW ped call (1-cycle pulse) served |
| 8 | NS ped call (1-cycle pulse) served |
| 9 | reset mid-cycle |
| 10 | EW car pulse latched |
| 11 | both ped buttons at once |
| 12 | NS car pulse latched during EW green |
| 13 | NS ped only: EW phases skipped |
| 14 | EW ped only: EW green/yellow skipped |
| 15 | stuck ped button doesn't lock out NS |
| 16 | `T_GREEN` override reaches `T_GREEN_MAX` |
