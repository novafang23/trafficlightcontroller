`timescale 1ns/1ps

module toptb();

    // Testbench Parameters matching RTL configuration
    localparam BITS        = 16;
    localparam T_GREEN_MIN = 16'd6;
    localparam T_GREEN_MAX = 16'd15;
    localparam T_YELLOW    = 16'd4;
    localparam T_RED       = 16'd6; // Pedestrian walk duration

    reg clk;
    reg rst_n;
    reg car_sensor_ns;
    reg car_sensor_ew;
    reg ped_call_ns;
    reg ped_call_ew;

    wire [2:0] ns, ew;
    wire w_ns, w_ew;

    integer errors = 0;
    integer checks = 0;
    integer cycle_count = 0;
    integer state_duration = 0;

    // State decoding for monitor and debug
    reg [2:0] prev_state;
    reg [2:0] current_state;

    // Unit Under Test (UUT)
    top #(
        .BITS(BITS),
        .T_GREEN_MIN(T_GREEN_MIN),
        .T_GREEN_MAX(T_GREEN_MAX),
        .T_YELLOW(T_YELLOW),
        .T_RED(T_RED)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .car_sensor_ns(car_sensor_ns),
        .car_sensor_ew(car_sensor_ew),
        .ped_call_ns(ped_call_ns),
        .ped_call_ew(ped_call_ew),
        .ns_lights(ns),
        .ew_lights(ew),
        .walk_ns(w_ns),
        .walk_ew(w_ew)
    );

    // Auxiliary instance for verifying backward-compatible parameter cascading (T_GREEN -> T_GREEN_MAX)
    wire [2:0] p_ns, p_ew;
    wire p_wns, p_wew;
    top #(
        .BITS(16),
        .T_GREEN_MIN(16'd4),
        .T_GREEN(16'd12), // Overriding T_GREEN must cascade to T_GREEN_MAX
        .T_YELLOW(16'd2),
        .T_RED(16'd3)
    ) uut_param (
        .clk(clk),
        .rst_n(rst_n),
        .car_sensor_ns(car_sensor_ns),
        .car_sensor_ew(car_sensor_ew),
        .ped_call_ns(ped_call_ns),
        .ped_call_ew(ped_call_ew),
        .ns_lights(p_ns),
        .ew_lights(p_ew),
        .walk_ns(p_wns),
        .walk_ew(p_wew)
    );

    // Clock generator: 10ns period (100 MHz)
    always #5 clk = ~clk;

    // Derive current abstract state from output combination
    always @(*) begin
        if (ns == 3'b001 && ew == 3'b100 && !w_ns && !w_ew)
            current_state = 3'd0; // S0_NS_G
        else if (ns == 3'b010 && ew == 3'b100 && !w_ns && !w_ew)
            current_state = 3'd1; // S1_NS_Y
        else if (ns == 3'b100 && ew == 3'b100 && !w_ns && w_ew)
            current_state = 3'd2; // S2_EW_W
        else if (ns == 3'b100 && ew == 3'b001 && !w_ns && !w_ew)
            current_state = 3'd3; // S3_EW_G
        else if (ns == 3'b100 && ew == 3'b010 && !w_ns && !w_ew)
            current_state = 3'd4; // S4_EW_Y
        else if (ns == 3'b100 && ew == 3'b100 && w_ns && !w_ew)
            current_state = 3'd5; // S5_NS_W
        else
            current_state = 3'd7; // INVALID / TRANSITIONAL
    end

    // Safety Invariant Checkers evaluated on every clock edge
    always @(posedge clk) begin
        if (rst_n) begin
            checks = checks + 1;

            // Invariant 1: Mutually exclusive Green lights
            if (ns[0] && ew[0]) begin
                $display("[ERROR @ %0t ns] CRITICAL SAFETY VIOLATION: Both NS and EW are GREEN simultaneously!", $time);
                errors = errors + 1;
            end

            // Invariant 2: Conflicting non-red vehicular lights
            if ((ns[0] | ns[1]) && (ew[0] | ew[1])) begin
                $display("[ERROR @ %0t ns] CONFLICT VIOLATION: Conflicting non-red lights on NS and EW!", $time);
                errors = errors + 1;
            end

            // Invariant 3: Pedestrian Walk Safety (All vehicles must be RED during walk)
            if (w_ew && (ns != 3'b100 || ew != 3'b100)) begin
                $display("[ERROR @ %0t ns] PEDESTRIAN SAFETY VIOLATION: EW walk active while vehicular traffic is not all-Red!", $time);
                errors = errors + 1;
            end

            if (w_ns && (ns != 3'b100 || ew != 3'b100)) begin
                $display("[ERROR @ %0t ns] PEDESTRIAN SAFETY VIOLATION: NS walk active while vehicular traffic is not all-Red!", $time);
                errors = errors + 1;
            end

            // Invariant 4: No dual pedestrian walk
            if (w_ns && w_ew) begin
                $display("[ERROR @ %0t ns] CONFLICT VIOLATION: Both NS and EW pedestrian walk signals active simultaneously!", $time);
                errors = errors + 1;
            end

            // Invariant 5: One-hot lights for vehicular signals
            if (ns != 3'b001 && ns != 3'b010 && ns != 3'b100) begin
                $display("[ERROR @ %0t ns] ENCODING VIOLATION: Invalid NS light value: %b", $time, ns);
                errors = errors + 1;
            end

            if (ew != 3'b001 && ew != 3'b010 && ew != 3'b100) begin
                $display("[ERROR @ %0t ns] ENCODING VIOLATION: Invalid EW light value: %b", $time, ew);
                errors = errors + 1;
            end
        end
    end

    // State duration tracker and cycle verifier
    always @(posedge clk) begin
        if (!rst_n) begin
            cycle_count    <= 0;
            state_duration <= 0;
            prev_state     <= 3'd7;
        end else begin
            cycle_count <= cycle_count + 1;

            if (current_state == prev_state) begin
                state_duration <= state_duration + 1;
            end else begin
                if (prev_state != 3'd7) begin
                    $display("[MONITOR @ %0t ns] State %0d held for %0d cycles -> Entering State %0d",
                             $time, prev_state, state_duration, current_state);
                end
                prev_state     <= current_state;
                state_duration <= 1; // 1st cycle of new state
            end
        end
    end

    // Main Test Stimulus
    initial begin
        $dumpfile("simulation.vcd");
        $dumpvars(0, toptb);

        $display("================================================================================");
        $display("   STARTING SMART ACTUATED TRAFFIC LIGHT CONTROLLER TESTBENCH                   ");
        $display("   Parameters: T_MIN=%0d, T_MAX=%0d, T_YEL=%0d, T_WALK=%0d                      ",
                 T_GREEN_MIN, T_GREEN_MAX, T_YELLOW, T_RED);
        $display("================================================================================");

        clk           = 0;
        rst_n         = 0;
        car_sensor_ns = 0;
        car_sensor_ew = 0;
        ped_call_ns   = 0;
        ped_call_ew   = 0;

        // -------------------------------------------------------------------------
        // SCENARIO 0: Synchronous Reset and Initialization
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 0] Testing Synchronous Reset Assertion and Initial State...");
        repeat (3) @(negedge clk);
        if (ns !== 3'b001 || ew !== 3'b100 || w_ns !== 1'b0 || w_ew !== 1'b0) begin
            $display("[ERROR @ %0t ns] Reset initial state mismatch: ns=%b, ew=%b", $time, ns, ew);
            errors = errors + 1;
        end else begin
            $display("[PASS] Controller properly held in S0_NS_G during reset.");
        end

        @(negedge clk);
        rst_n = 1;
        $display("[INFO @ %0t ns] Reset released synchronously.", $time);

        // -------------------------------------------------------------------------
        // SCENARIO 1: Main Street Resting in Green (No Side-Street Demand)
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 1] Main Street Resting in Green (No side cars or pedestrian calls)...");
        // Run for 25 clock cycles (> T_GREEN_MAX of 15 cycles)
        repeat (25) @(negedge clk);
        if (current_state !== 3'd0) begin
            $display("[ERROR @ %0t ns] Controller failed to rest in S0_NS_G! State = %0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] NS correctly stayed in S0_NS_G for 25 cycles (> T_GREEN_MAX=%0d) with no side demand.", T_GREEN_MAX);
        end

        // -------------------------------------------------------------------------
        // SCENARIO 2: Gap-Out Early Transition on NS when vehicle sensor drops
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 2] Gap-Out Early Transition on NS (drops at cycle 8 < T_MAX=15)...");
        // Demand arrives on EW: car waiting on side street
        car_sensor_ew = 1;
        // Cars flowing continuously on NS
        car_sensor_ns = 1;

        // Let NS green run for 7 cycles, then drop car_sensor_ns to simulate gap-out
        repeat (7) @(negedge clk);
        car_sensor_ns = 0; // Traffic gaps out after minimum green (6) is satisfied
        $display("[INFO @ %0t ns] NS traffic gapped out (car_sensor_ns=0). Expecting transition to S1_NS_Y.", $time);

        // Transition should occur on the next clock cycle to S1_NS_Y
        @(negedge clk);
        if (current_state !== 3'd1) begin
            $display("[ERROR @ %0t ns] Gap-out failed to transition to S1_NS_Y! Current state=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] NS gapped out early and entered S1_NS_Y successfully.");
        end

        // -------------------------------------------------------------------------
        // SCENARIO 3: Pedestrian Walk Skipping (EW Walk Skipped when no button pressed)
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 3] Verifying EW Pedestrian Walk Phase Skipping (no ped call)...");
        // In S1_NS_Y for T_YELLOW (4 cycles). At end of Yellow, with ped_call_ew=0 and car_sensor_ew=1,
        // it must skip S2_EW_W and transition DIRECTLY to S3_EW_G!
        repeat (T_YELLOW - 1) @(negedge clk);
        @(negedge clk);
        if (current_state !== 3'd3) begin
            $display("[ERROR @ %0t ns] Expected direct transition to S3_EW_G (skipping S2_EW_W)! Got state=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] S2_EW_W was skipped; transitioned directly from S1_NS_Y to S3_EW_G.");
        end

        // -------------------------------------------------------------------------
        // SCENARIO 4: Gap-Out Early Transition on EW (drops at T_GREEN_MIN)
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 4] Side-Street (EW) Gap-Out at T_GREEN_MIN (6 cycles < T_MAX=15)...");
        // car_sensor_ew is high. At cycle 5 (0-indexed 6th cycle = T_GREEN_MIN), drop car_sensor_ew
        repeat (T_GREEN_MIN - 1) @(negedge clk);
        car_sensor_ew = 0; // Traffic gaps out
        @(negedge clk);
        if (current_state !== 3'd4) begin
            $display("[ERROR @ %0t ns] EW Green failed to gap out to S4_EW_Y! State=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] EW Green gapped out after exactly T_GREEN_MIN (%0d cycles) to S4_EW_Y.", T_GREEN_MIN);
        end

        // -------------------------------------------------------------------------
        // SCENARIO 5: Pedestrian Walk Skipping on NS (NS Walk Skipped when no call)
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 5] Verifying NS Pedestrian Walk Phase Skipping (no ped call)...");
        // In S4_EW_Y for T_YELLOW (4 cycles). With ped_call_ns=0, must skip S5_NS_W directly to S0_NS_G
        repeat (T_YELLOW - 1) @(negedge clk);
        @(negedge clk);
        if (current_state !== 3'd0) begin
            $display("[ERROR @ %0t ns] Expected transition from S4_EW_Y to S0_NS_G! Got state=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] S5_NS_W was skipped; transitioned directly from S4_EW_Y to S0_NS_G.");
        end

        // -------------------------------------------------------------------------
        // SCENARIO 6: Continuous Dense Traffic - Phase Extension up to Max Green
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 6] Continuous Dense Traffic: Phase Extension up to T_GREEN_MAX (%0d cycles)...", T_GREEN_MAX);
        // Both vehicle sensors continuously active
        car_sensor_ns = 1;
        car_sensor_ew = 1;
        ped_call_ns   = 0;
        ped_call_ew   = 0;

        // NS Green starts now. It should hold for exactly T_GREEN_MAX (15 cycles)
        repeat (T_GREEN_MAX) @(negedge clk);
        if (current_state !== 3'd1) begin
            $display("[ERROR @ %0t ns] NS Green did not extend to T_GREEN_MAX (%0d) before Yellow! State=%0d",
                     $time, T_GREEN_MAX, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] NS Green extended full duration of T_GREEN_MAX (%0d cycles) under continuous traffic.", T_GREEN_MAX);
        end

        // Yellow holds for T_YELLOW
        repeat (T_YELLOW) @(negedge clk);
        // Skipped walk, entered EW Green (S3_EW_G)
        if (current_state !== 3'd3) begin
            $display("[ERROR @ %0t ns] Expected S3_EW_G! Got state=%0d", $time, current_state);
            errors = errors + 1;
        end

        // EW Green with car_sensor_ew=1 should hold for exactly T_GREEN_MAX (15 cycles)
        repeat (T_GREEN_MAX) @(negedge clk);
        if (current_state !== 3'd4) begin
            $display("[ERROR @ %0t ns] EW Green did not extend to T_GREEN_MAX (%0d) before Yellow! State=%0d",
                     $time, T_GREEN_MAX, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] EW Green extended full duration of T_GREEN_MAX (%0d cycles) under continuous traffic.", T_GREEN_MAX);
        end

        // Yellow holds for T_YELLOW
        repeat (T_YELLOW) @(negedge clk);
        if (current_state !== 3'd0) begin
            $display("[ERROR @ %0t ns] Expected return to S0_NS_G! Got state=%0d", $time, current_state);
            errors = errors + 1;
        end

        // -------------------------------------------------------------------------
        // SCENARIO 7: Pedestrian Actuation & Latching on EW (ped_call_ew pulse)
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 7] Pedestrian Call EW: 1-Cycle Pulse Latching & Walk Service...");
        car_sensor_ns = 0;
        car_sensor_ew = 1;

        // Apply a momentary 1-cycle pulse to pedestrian call button
        @(negedge clk);
        ped_call_ew = 1;
        $display("[INFO @ %0t ns] Pedestrian pressed EW call button (1-cycle pulse).", $time);
        @(negedge clk);
        ped_call_ew = 0;

        // NS Green completes (gaps out at T_GREEN_MIN since car_sensor_ns=0)
        repeat (T_GREEN_MIN) @(negedge clk);
        // In S1_NS_Y for T_YELLOW
        repeat (T_YELLOW) @(negedge clk);

        // Now S2_EW_W MUST be entered!
        if (current_state !== 3'd2 || w_ew !== 1'b1 || ns !== 3'b100 || ew !== 3'b100) begin
            $display("[ERROR @ %0t ns] EW Walk phase not properly serviced! state=%0d, w_ew=%b, ns=%b, ew=%b",
                     $time, current_state, w_ew, ns, ew);
            errors = errors + 1;
        end else begin
            $display("[PASS] Latched EW Pedestrian call successfully serviced: S2_EW_W active with walk_ew=1.");
        end

        // Check duration in S2_EW_W (T_RED = 6 cycles)
        repeat (T_RED) @(negedge clk);
        if (current_state !== 3'd3) begin
            $display("[ERROR @ %0t ns] Expected transition from S2_EW_W to S3_EW_G! State=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] S2_EW_W held for exactly T_RED (%0d cycles); transitioned to S3_EW_G.", T_RED);
        end

        // -------------------------------------------------------------------------
        // SCENARIO 8: Pedestrian Actuation & Latching on NS (ped_call_ns pulse)
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 8] Pedestrian Call NS: 1-Cycle Pulse Latching & Walk Service...");
        // In S3_EW_G: gap out quickly
        car_sensor_ew = 0;
        repeat (T_GREEN_MIN) @(negedge clk);
        // Now entering S4_EW_Y

        // Pulse ped_call_ns
        @(negedge clk);
        ped_call_ns = 1;
        $display("[INFO @ %0t ns] Pedestrian pressed NS call button (1-cycle pulse).", $time);
        @(negedge clk);
        ped_call_ns = 0;

        // Finish remaining Yellow cycles
        repeat (T_YELLOW - 2) @(negedge clk);
        @(negedge clk);

        // Now S5_NS_W MUST be entered!
        if (current_state !== 3'd5 || w_ns !== 1'b1 || ns !== 3'b100 || ew !== 3'b100) begin
            $display("[ERROR @ %0t ns] NS Walk phase not properly serviced! state=%0d, w_ns=%b, ns=%b, ew=%b",
                     $time, current_state, w_ns, ns, ew);
            errors = errors + 1;
        end else begin
            $display("[PASS] Latched NS Pedestrian call successfully serviced: S5_NS_W active with walk_ns=1.");
        end

        // Check duration in S5_NS_W (T_RED = 6 cycles)
        repeat (T_RED) @(negedge clk);
        if (current_state !== 3'd0) begin
            $display("[ERROR @ %0t ns] Expected transition from S5_NS_W to S0_NS_G! State=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] S5_NS_W held for exactly T_RED (%0d cycles); returned to S0_NS_G.", T_RED);
        end

        // -------------------------------------------------------------------------
        // SCENARIO 9: Dynamic Synchronous Reset Mid-Operation
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 9] Dynamic Synchronous Reset Mid-Operation...");
        // Stimulate EW traffic and transition into S3_EW_G
        car_sensor_ew = 1;
        car_sensor_ns = 0;
        repeat (T_GREEN_MIN + T_YELLOW + 1) @(negedge clk);
        $display("[INFO @ %0t ns] Asserting rst_n mid-operation during state %0d...", $time, current_state);

        @(negedge clk);
        rst_n = 0;
        @(negedge clk);
        if (current_state !== 3'd0 || ns !== 3'b001 || ew !== 3'b100) begin
            $display("[ERROR @ %0t ns] Mid-operation reset failed to restore S0_NS_G! state=%0d, ns=%b, ew=%b",
                     $time, current_state, ns, ew);
            errors = errors + 1;
        end else begin
            $display("[PASS] Synchronous reset successfully restored S0_NS_G and cleared all latches.");
        end

        rst_n = 1;
        car_sensor_ew = 0;
        car_sensor_ns = 0;
        repeat (10) @(negedge clk);

        // -------------------------------------------------------------------------
        // SCENARIO 10: Vehicle Call Latching on Side Street (1-cycle pulse)
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 10] Vehicle Call Latching: 1-Cycle Pulse on car_sensor_ew...");
        car_sensor_ns = 0;
        car_sensor_ew = 0;
        ped_call_ns   = 0;
        ped_call_ew   = 0;

        // Pulse car_sensor_ew for 1 cycle
        @(negedge clk);
        car_sensor_ew = 1;
        $display("[INFO @ %0t ns] Car passed over EW inductive loop (1-cycle pulse).", $time);
        @(negedge clk);
        car_sensor_ew = 0; // Sensor back to 0, but latched call should remain!

        // NS transitions to Yellow
        repeat (T_YELLOW) @(negedge clk);

        // EW Walk skipped (no ped), enters EW Green S3_EW_G!
        if (current_state !== 3'd3) begin
            $display("[ERROR @ %0t ns] Latched vehicle call failed to enter S3_EW_G! State=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] Latched EW vehicle call successfully entered S3_EW_G.");
        end

        // EW Green runs for T_GREEN_MIN, then gaps out to S4_EW_Y since car_sensor_ew=0
        repeat (T_GREEN_MIN) @(negedge clk);
        if (current_state !== 3'd4) begin
            $display("[ERROR @ %0t ns] Expected transition to S4_EW_Y after T_GREEN_MIN! State=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] Latched vehicle served with minimum green, then gapped out to S4_EW_Y.");
        end

        // Return to S0_NS_G
        repeat (T_YELLOW) @(negedge clk);
        if (current_state !== 3'd0) begin
            $display("[ERROR @ %0t ns] Expected return to S0_NS_G! State=%0d", $time, current_state);
            errors = errors + 1;
        end

        // -------------------------------------------------------------------------
        // SCENARIO 11: Simultaneous Pedestrian Calls on Both Approaches
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 11] Simultaneous Pedestrian Calls on NS and EW...");
        car_sensor_ns = 0;
        car_sensor_ew = 0;

        @(negedge clk);
        ped_call_ns = 1;
        ped_call_ew = 1;
        $display("[INFO @ %0t ns] Both NS and EW pedestrian buttons pressed simultaneously.", $time);
        @(negedge clk);
        ped_call_ns = 0;
        ped_call_ew = 0;

        // NS Green completes minimum green (T_GREEN_MIN cycles)
        repeat (T_GREEN_MIN) @(negedge clk);
        // NS Yellow runs for T_YELLOW cycles
        repeat (T_YELLOW) @(negedge clk);

        // S2_EW_W entered first
        if (current_state !== 3'd2 || w_ew !== 1'b1 || w_ns !== 1'b0) begin
            $display("[ERROR @ %0t ns] EW Walk phase not entered! State=%0d, w_ew=%b", $time, current_state, w_ew);
            errors = errors + 1;
        end else begin
            $display("[PASS] EW Walk phase safely entered first: walk_ew=1, walk_ns=0.");
        end

        // Hold for T_RED
        repeat (T_RED) @(negedge clk);

        // Since no EW cars, skips EW vehicular green, transitions to S5_NS_W
        if (current_state !== 3'd5 || w_ns !== 1'b1 || w_ew !== 1'b0) begin
            $display("[ERROR @ %0t ns] NS Walk phase not entered! State=%0d, w_ns=%b", $time, current_state, w_ns);
            errors = errors + 1;
        end else begin
            $display("[PASS] NS Walk phase safely entered next: walk_ns=1, walk_ew=0.");
        end

        // Hold for T_RED
        repeat (T_RED) @(negedge clk);

        // -------------------------------------------------------------------------
        // SCENARIO 12: Vehicle Call Latching on North-South Main Street
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 12] Vehicle Call Latching: 1-Cycle Pulse on car_sensor_ns during EW Green...");
        car_sensor_ns = 0;
        car_sensor_ew = 1;
        ped_call_ns   = 0;
        ped_call_ew   = 0;

        // Transition into S3_EW_G
        repeat (T_GREEN_MIN) @(negedge clk);
        repeat (T_YELLOW) @(negedge clk);
        if (current_state !== 3'd3) begin
            $display("[ERROR @ %0t ns] Failed to enter S3_EW_G for Scenario 12! State=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] Controller in S3_EW_G. Now pulsing car_sensor_ns...");
        end

        // Pulse car_sensor_ns for 1 clock cycle while EW is Green
        @(negedge clk);
        car_sensor_ns = 1;
        $display("[INFO @ %0t ns] Car passed over NS inductive loop (1-cycle pulse) while EW is green.", $time);
        @(negedge clk);
        car_sensor_ns = 0;
        car_sensor_ew = 0; // EW traffic gaps out

        // EW green finishes minimum green and gaps out to S4_EW_Y
        repeat (T_GREEN_MIN - 2) @(negedge clk);
        @(negedge clk);
        if (current_state !== 3'd4) begin
            $display("[ERROR @ %0t ns] Expected S4_EW_Y! State=%0d", $time, current_state);
            errors = errors + 1;
        end

        // EW yellow finishes
        repeat (T_YELLOW) @(negedge clk);
        if (current_state !== 3'd0 || ns !== 3'b001 || ew !== 3'b100) begin
            $display("[ERROR @ %0t ns] Latched NS vehicle call failed to return to S0_NS_G! State=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] Latched NS vehicle call successfully entered S0_NS_G.");
        end

        // -------------------------------------------------------------------------
        // SCENARIO 13: Side-Street Phase Skipping when only NS Pedestrian Calls
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 13] Side-Street Phase Skipping: Only NS Pedestrian calls from resting green...");
        // Controller is in S0_NS_G, let it rest
        repeat (10) @(negedge clk);
        car_sensor_ns = 0;
        car_sensor_ew = 0;
        ped_call_ns   = 0;
        ped_call_ew   = 0;

        // Pulse ped_call_ns for 1 clock cycle
        @(negedge clk);
        ped_call_ns = 1;
        $display("[INFO @ %0t ns] Pedestrian pressed NS call button (1-cycle pulse).", $time);
        @(negedge clk);
        ped_call_ns = 0;

        // NS transitions immediately to S1_NS_Y since T_GREEN_MIN is satisfied
        @(negedge clk);
        if (current_state !== 3'd1) begin
            $display("[ERROR @ %0t ns] Expected transition to S1_NS_Y! State=%0d", $time, current_state);
            errors = errors + 1;
        end

        // NS Yellow runs for T_YELLOW
        repeat (T_YELLOW) @(negedge clk);

        // MUST skip all EW phases (S2_EW_W, S3_EW_G, S4_EW_Y) and transition directly to S5_NS_W!
        if (current_state !== 3'd5 || w_ns !== 1'b1 || ns !== 3'b100 || ew !== 3'b100) begin
            $display("[ERROR @ %0t ns] Failed to skip side street directly to S5_NS_W! State=%0d, w_ns=%b",
                     $time, current_state, w_ns);
            errors = errors + 1;
        end else begin
            $display("[PASS] Side street completely skipped: transitioned directly from S1_NS_Y to S5_NS_W (walk_ns=1).");
        end

        // S5_NS_W runs for T_RED and returns directly to S0_NS_G
        repeat (T_RED) @(negedge clk);
        if (current_state !== 3'd0) begin
            $display("[ERROR @ %0t ns] Expected return to S0_NS_G from S5_NS_W! State=%0d", $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] S5_NS_W returned cleanly to S0_NS_G.");
        end

        // -------------------------------------------------------------------------
        // SCENARIO 14: Side-Street Vehicular Green Skipping when only EW Pedestrian Calls
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 14] Side-Street Vehicular Green Skipping: Only EW Pedestrian calls (no cars on EW)...");
        repeat (10) @(negedge clk);
        car_sensor_ns = 0;
        car_sensor_ew = 0;
        ped_call_ns   = 0;
        ped_call_ew   = 0;

        // Pulse ped_call_ew for 1 clock cycle
        @(negedge clk);
        ped_call_ew = 1;
        $display("[INFO @ %0t ns] Pedestrian pressed EW call button (1-cycle pulse).", $time);
        @(negedge clk);
        ped_call_ew = 0;

        // Transitions through S1_NS_Y
        @(negedge clk);
        repeat (T_YELLOW) @(negedge clk);

        // Enters S2_EW_W
        if (current_state !== 3'd2 || w_ew !== 1'b1) begin
            $display("[ERROR @ %0t ns] Expected S2_EW_W! State=%0d, w_ew=%b", $time, current_state, w_ew);
            errors = errors + 1;
        end else begin
            $display("[PASS] Entered S2_EW_W for EW pedestrian walk.");
        end

        // Hold for T_RED
        repeat (T_RED) @(negedge clk);

        // Since no cars on EW (car_sensor_ew=0), MUST skip S3_EW_G and S4_EW_Y directly to S0_NS_G!
        if (current_state !== 3'd0) begin
            $display("[ERROR @ %0t ns] Failed to skip EW vehicular green/yellow directly to S0_NS_G! State=%0d",
                     $time, current_state, w_ew);
            errors = errors + 1;
        end else begin
            $display("[PASS] S3_EW_G and S4_EW_Y skipped: transitioned directly from S2_EW_W to S0_NS_G.");
        end

        // -------------------------------------------------------------------------
        // SCENARIO 15: Stuck / Continuous Pedestrian Button Holding
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 15] Stuck / Continuous Pedestrian Call Button Holding (Anti-Starvation)...");
        // Hold ped_call_ew high continuously
        ped_call_ew = 1;
        $display("[INFO @ %0t ns] ped_call_ew held high continuously.", $time);

        // NS green must hold for at least T_GREEN_MIN cycles before transition
        repeat (T_GREEN_MIN) @(negedge clk);
        // S1_NS_Y for T_YELLOW
        repeat (T_YELLOW) @(negedge clk);
        // S2_EW_W for T_RED
        if (current_state !== 3'd2) begin
            $display("[ERROR @ %0t ns] Expected S2_EW_W! State=%0d", $time, current_state);
            errors = errors + 1;
        end
        repeat (T_RED) @(negedge clk);

        // Must return to S0_NS_G and NOT lock in S2_EW_W
        if (current_state !== 3'd0) begin
            $display("[ERROR @ %0t ns] Controller failed to return to S0_NS_G during stuck button! State=%0d",
                     $time, current_state);
            errors = errors + 1;
        end else begin
            $display("[PASS] Controller returned to S0_NS_G despite held button; vehicular traffic not starved.");
        end

        // Release stuck button
        ped_call_ew = 0;
        repeat (10) @(negedge clk);

        // -------------------------------------------------------------------------
        // SCENARIO 16: Parameter Cascading & Backward Compatibility Verification
        // -------------------------------------------------------------------------
        $display("\n>>> [SCENARIO 16] Verifying T_GREEN parameter cascade to T_GREEN_MAX...");
        if (uut_param.controller.T_GREEN_MAX !== 16'd12) begin
            $display("[ERROR @ %0t ns] Parameter cascade failed: expected T_GREEN_MAX=12, got %0d",
                     $time, uut_param.controller.T_GREEN_MAX);
            errors = errors + 1;
        end else begin
            $display("[PASS] T_GREEN alias successfully cascaded to T_GREEN_MAX=%0d in auxiliary instance.",
                     uut_param.controller.T_GREEN_MAX);
        end

        // -------------------------------------------------------------------------
        // Final Results Summary
        // -------------------------------------------------------------------------
        $display("\n================================================================================");
        $display("                         TESTBENCH RESULTS SUMMARY                              ");
        $display("================================================================================");
        $display("Total Invariant Checks Evaluated: %0d", checks);
        $display("Total Verification Errors Found:  %0d", errors);

        if (errors == 0) begin
            $display(">>> ALL TESTS PASSED: SAFETY INVARIANTS, GAP-OUT, MAX-OUT & WALK SERVICE MET <<<");
        end else begin
            $display(">>> TEST FAILED: %0d ERROR(S) DETECTED <<<", errors);
        end
        $display("================================================================================");

        $finish;
    end

endmodule
