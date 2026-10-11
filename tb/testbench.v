`timescale 1ns/1ps

module toptb();

    // Testbench Parameters matching RTL defaults
    localparam BITS     = 16;
    localparam T_GREEN  = 16'd10;
    localparam T_YELLOW = 16'd4;
    localparam T_RED    = 16'd6;

    reg clk;
    reg rst_n;
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
        .T_GREEN(T_GREEN),
        .T_YELLOW(T_YELLOW),
        .T_RED(T_RED)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .ns_lights(ns),
        .ew_lights(ew),
        .walk_ns(w_ns),
        .walk_ew(w_ew)
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

            // Invariant 2: Green & Yellow conflict
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
            prev_state     <= 3'd7; // Invalid state so first cycle after reset initializes cleanly
        end else begin
            cycle_count <= cycle_count + 1;

            if (current_state == prev_state) begin
                state_duration <= state_duration + 1;
            end else begin
                // State changed: Verify the previous state duration
                case (prev_state)
                    3'd0: begin // S0_NS_G
                        $display("[INFO @ %0t ns] State 0 (NS_GREEN) held for %0d cycles (Expected: %0d)", $time, state_duration, T_GREEN);
                        if (state_duration != T_GREEN) begin
                            $display("[ERROR] S0 duration mismatch! Got %0d, expected %0d", state_duration, T_GREEN);
                            errors = errors + 1;
                        end
                    end
                    3'd1: begin // S1_NS_Y
                        $display("[INFO @ %0t ns] State 1 (NS_YELLOW) held for %0d cycles (Expected: %0d)", $time, state_duration, T_YELLOW);
                        if (state_duration != T_YELLOW) begin
                            $display("[ERROR] S1 duration mismatch! Got %0d, expected %0d", state_duration, T_YELLOW);
                            errors = errors + 1;
                        end
                    end
                    3'd2: begin // S2_EW_W
                        $display("[INFO @ %0t ns] State 2 (EW_WALK) held for %0d cycles (Expected: %0d)", $time, state_duration, T_RED);
                        if (state_duration != T_RED) begin
                            $display("[ERROR] S2 duration mismatch! Got %0d, expected %0d", state_duration, T_RED);
                            errors = errors + 1;
                        end
                    end
                    3'd3: begin // S3_EW_G
                        $display("[INFO @ %0t ns] State 3 (EW_GREEN) held for %0d cycles (Expected: %0d)", $time, state_duration, T_GREEN);
                        if (state_duration != T_GREEN) begin
                            $display("[ERROR] S3 duration mismatch! Got %0d, expected %0d", state_duration, T_GREEN);
                            errors = errors + 1;
                        end
                    end
                    3'd4: begin // S4_EW_Y
                        $display("[INFO @ %0t ns] State 4 (EW_YELLOW) held for %0d cycles (Expected: %0d)", $time, state_duration, T_YELLOW);
                        if (state_duration != T_YELLOW) begin
                            $display("[ERROR] S4 duration mismatch! Got %0d, expected %0d", state_duration, T_YELLOW);
                            errors = errors + 1;
                        end
                    end
                    3'd5: begin // S5_NS_W
                        $display("[INFO @ %0t ns] State 5 (NS_WALK) held for %0d cycles (Expected: %0d)", $time, state_duration, T_RED);
                        if (state_duration != T_RED) begin
                            $display("[ERROR] S5 duration mismatch! Got %0d, expected %0d", state_duration, T_RED);
                            errors = errors + 1;
                        end
                    end
                    3'd7: begin
                        // Initial state entry from reset
                    end
                    default: begin
                        $display("[WARN @ %0t ns] Transition from unrecognized state %0d", $time, prev_state);
                    end
                endcase

                prev_state     <= current_state;
                state_duration <= 1; // 1st cycle of new state
            end
        end
    end

    // Main Test Stimulus
    initial begin
        $dumpfile("simulation.vcd");
        $dumpvars(0, toptb);

        $display("======================================================");
        $display("   STARTING TRAFFIC LIGHT CONTROLLER TESTBENCH        ");
        $display("======================================================");

        clk   = 0;
        rst_n = 0;

        // Apply synchronous reset for 3 full clock cycles
        repeat (3) @(negedge clk);
        rst_n = 1;
        $display("[INFO @ %0t ns] Reset released synchronously.", $time);

        // Run through 2 full FSM cycles
        // Each full cycle = 10 + 4 + 6 + 10 + 4 + 6 = 40 clock cycles = 400 ns
        repeat (85) @(negedge clk);

        // Mid-cycle reset test: Assert reset asynchronously/synchronously during active operation
        $display("[INFO @ %0t ns] Testing dynamic synchronous reset assertion mid-operation...", $time);
        @(negedge clk);
        rst_n = 0;
        repeat (2) @(negedge clk);

        if (ns !== 3'b001 || ew !== 3'b100) begin
            $display("[ERROR @ %0t ns] Reset did not properly restore S0_NS_G state! ns=%b ew=%b", $time, ns, ew);
            errors = errors + 1;
        end else begin
            $display("[INFO @ %0t ns] Reset restored state to S0_NS_G successfully.", $time);
        end

        rst_n = 1;
        repeat (50) @(negedge clk);

        // Summary and Results Check
        $display("======================================================");
        $display("               TESTBENCH RESULTS SUMMARY              ");
        $display("======================================================");
        $display("Total Invariant Checks: %0d", checks);
        $display("Total Errors Found:     %0d", errors);

        if (errors == 0) begin
            $display(">>> TEST PASSED: ALL SAFETY INVARIANTS & TIMINGS MET <<<");
        end else begin
            $display(">>> TEST FAILED: %0d ERRORS DETECTED <<<", errors);
        end
        $display("======================================================");

        $finish;
    end

endmodule
