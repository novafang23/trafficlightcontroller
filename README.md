**Traffic Light Controller (Verilog)**

**A Verilog FSM-based traffic light controller for a 4-way intersection along with pedestrian walking**
What it does-:
Cycles through 6 states:

NS_GREEN → NS_YELLOW → EW_WALK → EW_GREEN → EW_YELLOW → NS_WALK → (repeat)
Each state's duration is set by a shared counter module. NS and EW are never green at once, and each direction gets a walk phase after its yellow.
