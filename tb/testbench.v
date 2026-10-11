`timescale 1ns/1ps

module toptb;
    localparam BITS  = 16;
    localparam T_MIN = 6, T_MAX = 15, T_YEL = 4, T_RED = 6;

    reg clk = 0, rst_n = 0;
    reg car_sensor_ns = 0, car_sensor_ew = 0;
    reg ped_call_ns = 0, ped_call_ew = 0;

    wire [2:0] ns, ew;
    wire w_ns, w_ew;

    integer errors = 0, checks = 0;
    integer dur = 0;
    reg [2:0] cur, prev;

    top #(.BITS(BITS), .T_GREEN_MIN(T_MIN), .T_GREEN_MAX(T_MAX),
          .T_YELLOW(T_YEL), .T_RED(T_RED)) uut (
        .clk(clk), .rst_n(rst_n),
        .car_sensor_ns(car_sensor_ns), .car_sensor_ew(car_sensor_ew),
        .ped_call_ns(ped_call_ns), .ped_call_ew(ped_call_ew),
        .ns_lights(ns), .ew_lights(ew), .walk_ns(w_ns), .walk_ew(w_ew)
    );

    // second copy, only here to check T_GREEN -> T_GREEN_MAX still cascades
    wire [2:0] p_ns, p_ew;
    wire p_wns, p_wew;
    top #(.BITS(16), .T_GREEN_MIN(4), .T_GREEN(12), .T_YELLOW(2), .T_RED(3)) uut_param (
        .clk(clk), .rst_n(rst_n),
        .car_sensor_ns(car_sensor_ns), .car_sensor_ew(car_sensor_ew),
        .ped_call_ns(ped_call_ns), .ped_call_ew(ped_call_ew),
        .ns_lights(p_ns), .ew_lights(p_ew), .walk_ns(p_wns), .walk_ew(p_wew)
    );

    always #5 clk = ~clk;

    // rebuild the fsm state from the outputs, 7 = not a legal combo
    always @(*) begin
        cur = 3'd7;
        if      (ns == 3'b001 && ew == 3'b100 && !w_ns && !w_ew) cur = 0;
        else if (ns == 3'b010 && ew == 3'b100 && !w_ns && !w_ew) cur = 1;
        else if (ns == 3'b100 && ew == 3'b100 && !w_ns &&  w_ew) cur = 2;
        else if (ns == 3'b100 && ew == 3'b001 && !w_ns && !w_ew) cur = 3;
        else if (ns == 3'b100 && ew == 3'b010 && !w_ns && !w_ew) cur = 4;
        else if (ns == 3'b100 && ew == 3'b100 &&  w_ns && !w_ew) cur = 5;
    end

    task tick(input integer n);
        repeat (n) @(negedge clk);
    endtask

    task check(input ok, input [8*56-1:0] msg);
        if (!ok) begin
            $display("FAIL @ %0t: %0s (state=%0d ns=%b ew=%b)", $time, msg, cur, ns, ew);
            errors = errors + 1;
        end
    endtask

    task idle;
        {car_sensor_ns, car_sensor_ew, ped_call_ns, ped_call_ew} = 4'b0;
    endtask

    // always-on safety checks
    always @(posedge clk) if (rst_n) begin
        checks = checks + 1;
        check(!(ns[0] && ew[0]),                      "both directions green");
        check(!((ns[0]|ns[1]) && (ew[0]|ew[1])),      "ns/ew non-red at same time");
        check(!((w_ns || w_ew) && (ns != 3'b100 || ew != 3'b100)), "walk while cars not all-red");
        check(!(w_ns && w_ew),                        "both walks active");
        check(ns == 3'b001 || ns == 3'b010 || ns == 3'b100, "ns lights not one-hot");
        check(ew == 3'b001 || ew == 3'b010 || ew == 3'b100, "ew lights not one-hot");
    end

    // log how long each state was held
    always @(posedge clk) begin
        if (!rst_n) begin
            dur  <= 0;
            prev <= 3'd7;
        end else if (cur == prev) dur <= dur + 1;
        else begin
            if (prev != 3'd7) $display("  [%0t] state %0d held %0d -> %0d", $time, prev, dur, cur);
            prev <= cur;
            dur  <= 1;
        end
    end

    initial begin
        $dumpfile("simulation.vcd");
        $dumpvars(0, toptb);
        $display("tb: T_MIN=%0d T_MAX=%0d T_YEL=%0d T_RED=%0d", T_MIN, T_MAX, T_YEL, T_RED);

        // reset
        tick(3);
        check(ns === 3'b001 && ew === 3'b100 && !w_ns && !w_ew, "not in NS green during reset");
        tick(1);
        rst_n = 1;

        $display("1: NS rests in green with no demand");
        tick(25);
        check(cur == 0, "left NS green with nobody waiting");

        $display("2: NS gap-out");
        car_sensor_ew = 1;
        car_sensor_ns = 1;
        tick(7);
        car_sensor_ns = 0;
        tick(1);
        check(cur == 1, "no gap-out to NS yellow");

        $display("3: EW walk skipped, no ped call");
        tick(T_YEL - 1);
        tick(1);
        check(cur == 3, "didn't go straight to EW green");

        $display("4: EW gap-out at min green");
        tick(T_MIN - 1);
        car_sensor_ew = 0;
        tick(1);
        check(cur == 4, "EW didn't gap out to yellow");

        $display("5: NS walk skipped, no ped call");
        tick(T_YEL - 1);
        tick(1);
        check(cur == 0, "didn't return to NS green");

        $display("6: dense traffic maxes out both greens");
        car_sensor_ns = 1;
        car_sensor_ew = 1;
        ped_call_ns   = 0;
        ped_call_ew   = 0;
        tick(T_MAX);
        check(cur == 1, "NS green didn't run to max");
        tick(T_YEL);
        check(cur == 3, "expected EW green");
        tick(T_MAX);
        check(cur == 4, "EW green didn't run to max");
        tick(T_YEL);
        check(cur == 0, "expected NS green");

        $display("7: EW ped call, 1 cycle pulse");
        car_sensor_ns = 0;
        car_sensor_ew = 1;
        tick(1);
        ped_call_ew = 1;
        tick(1);
        ped_call_ew = 0;
        tick(T_MIN);
        tick(T_YEL);
        check(cur == 2 && w_ew && ns === 3'b100 && ew === 3'b100, "EW walk not served");
        tick(T_RED);
        check(cur == 3, "no EW green after EW walk");

        $display("8: NS ped call, 1 cycle pulse");
        car_sensor_ew = 0;
        tick(T_MIN);
        tick(1);
        ped_call_ns = 1;
        tick(1);
        ped_call_ns = 0;
        tick(T_YEL - 2);
        tick(1);
        check(cur == 5 && w_ns && ns === 3'b100 && ew === 3'b100, "NS walk not served");
        tick(T_RED);
        check(cur == 0, "no NS green after NS walk");

        $display("9: reset mid-cycle");
        car_sensor_ew = 1;
        car_sensor_ns = 0;
        tick(T_MIN + T_YEL + 1);
        tick(1);
        rst_n = 0;
        tick(1);
        check(cur == 0 && ns === 3'b001 && ew === 3'b100, "reset didn't restore NS green");
        rst_n = 1;
        idle;
        tick(10);

        $display("10: EW car pulse is latched");
        idle;
        tick(1);
        car_sensor_ew = 1;
        tick(1);
        car_sensor_ew = 0;
        tick(T_YEL);
        check(cur == 3, "latched EW car not served");
        tick(T_MIN);
        check(cur == 4, "EW didn't gap out after min green");
        tick(T_YEL);
        check(cur == 0, "no return to NS green");

        $display("11: both ped buttons at once");
        car_sensor_ns = 0;
        car_sensor_ew = 0;
        tick(1);
        ped_call_ns = 1;
        ped_call_ew = 1;
        tick(1);
        ped_call_ns = 0;
        ped_call_ew = 0;
        tick(T_MIN);
        tick(T_YEL);
        check(cur == 2 && w_ew && !w_ns, "EW walk should go first");
        tick(T_RED);
        check(cur == 5 && w_ns && !w_ew, "NS walk should follow");
        tick(T_RED);

        $display("12: NS car pulse during EW green is latched");
        car_sensor_ns = 0;
        car_sensor_ew = 1;
        ped_call_ns   = 0;
        ped_call_ew   = 0;
        tick(T_MIN);
        tick(T_YEL);
        check(cur == 3, "never reached EW green");
        tick(1);
        car_sensor_ns = 1;
        tick(1);
        car_sensor_ns = 0;
        car_sensor_ew = 0;
        tick(T_MIN - 2);
        tick(1);
        check(cur == 4, "expected EW yellow");
        tick(T_YEL);
        check(cur == 0 && ns === 3'b001 && ew === 3'b100, "latched NS car not served");

        $display("13: NS ped only, side street skipped");
        tick(10);
        idle;
        tick(1);
        ped_call_ns = 1;
        tick(1);
        ped_call_ns = 0;
        tick(1);
        check(cur == 1, "expected NS yellow");
        tick(T_YEL);
        check(cur == 5 && w_ns && ns === 3'b100 && ew === 3'b100, "side street not skipped");
        tick(T_RED);
        check(cur == 0, "no return to NS green");

        $display("14: EW ped only, EW green skipped");
        tick(10);
        idle;
        tick(1);
        ped_call_ew = 1;
        tick(1);
        ped_call_ew = 0;
        tick(1);
        tick(T_YEL);
        check(cur == 2 && w_ew, "expected EW walk");
        tick(T_RED);
        check(cur == 0, "EW green/yellow not skipped");

        $display("15: stuck ped button");
        ped_call_ew = 1;
        tick(T_MIN);
        tick(T_YEL);
        check(cur == 2, "expected EW walk");
        tick(T_RED);
        check(cur == 0, "stuck button locked out NS green");
        ped_call_ew = 0;
        tick(10);

        $display("16: T_GREEN alias");
        check(uut_param.controller.T_GREEN_MAX == 12, "T_GREEN didn't reach T_GREEN_MAX");

        $display("\n%0d checks, %0d errors", checks, errors);
        if (errors == 0) $display("PASSED");
        else             $display("FAILED");
        $finish;
    end
endmodule
