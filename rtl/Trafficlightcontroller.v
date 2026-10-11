`timescale 1ns/1ps

// free-running state timer, saturates instead of wrapping
module counter #(parameter W = 16) (
    input          clk,
    input          rst_n,
    input          en,
    input          clr,
    output reg [W-1:0] cnt
);
    always @(posedge clk)
        if (!rst_n || clr)      cnt <= 0;
        else if (en && ~&cnt)   cnt <= cnt + 1'b1;
endmodule


module fsm #(
    parameter W = 16,
    parameter [W-1:0] T_GREEN_MIN = 6,
    parameter [W-1:0] T_GREEN_MAX = 15,
    parameter [W-1:0] T_YELLOW    = 4,
    parameter [W-1:0] T_RED       = 6    // ped walk time, both sides all-red
)(
    input          clk,
    input          rst_n,
    input  [W-1:0] cnt,
    input          car_sensor_ns, car_sensor_ew,
    input          ped_call_ns,   ped_call_ew,
    output reg [2:0] ns_lights,          // {R,Y,G}
    output reg [2:0] ew_lights,
    output reg       walk_ns, walk_ew,
    output           timer_clr,
    output reg       timer_en
);
    localparam S_NS_G = 3'd0, S_NS_Y = 3'd1, S_EW_W = 3'd2,
               S_EW_G = 3'd3, S_EW_Y = 3'd4, S_NS_W = 3'd5;

    localparam G = 3'b001, Y = 3'b010, R = 3'b100;

    reg [2:0] state, nxt_state;

    // inputs assumed synced/debounced upstream, no CDC handling in here
    reg ped_ns_r, ped_ew_r, car_ns_r, car_ew_r;

    wire ped_ns = ped_ns_r | ped_call_ns;
    wire ped_ew = ped_ew_r | ped_call_ew;
    wire car_ns = car_ns_r | car_sensor_ns;
    wire car_ew = car_ew_r | car_sensor_ew;

    wire ns_busy = car_ew | ped_ew | ped_ns;   // someone is waiting on NS green

    wire min_done = cnt >= T_GREEN_MIN - 1'b1;
    wire max_done = cnt >= T_GREEN_MAX - 1'b1;
    wire yel_done = cnt >= T_YELLOW    - 1'b1;
    wire red_done = cnt >= T_RED       - 1'b1;

    initial begin
        if (T_GREEN_MIN < 1 || T_GREEN_MAX < T_GREEN_MIN || T_YELLOW < 1 || T_RED < 1) begin
            $display("fsm: bad timing params min=%0d max=%0d yel=%0d red=%0d",
                     T_GREEN_MIN, T_GREEN_MAX, T_YELLOW, T_RED);
            $finish;
        end
    end

    // call latches, cleared once the matching phase is being served
    always @(posedge clk) begin
        if (!rst_n) {ped_ns_r, ped_ew_r, car_ns_r, car_ew_r} <= 4'b0;
        else begin
            ped_ns_r <= (ped_ns_r | ped_call_ns)   & (state != S_NS_W);
            ped_ew_r <= (ped_ew_r | ped_call_ew)   & (state != S_EW_W);
            car_ns_r <= (car_ns_r | car_sensor_ns) & (state != S_NS_G);
            car_ew_r <= (car_ew_r | car_sensor_ew) & (state != S_EW_G);
        end
    end

    // sync reset
    always @(posedge clk)
        if (!rst_n) state <= S_NS_G;
        else        state <= nxt_state;

    always @(*) begin
        nxt_state = state;
        case (state)
            S_NS_G: if (min_done && ns_busy && (!car_ns || max_done)) nxt_state = S_NS_Y;
            S_NS_Y: if (yel_done) nxt_state = ped_ew ? S_EW_W :
                                              car_ew ? S_EW_G :
                                              ped_ns ? S_NS_W : S_NS_G;
            S_EW_W: if (red_done) nxt_state = car_ew ? S_EW_G :
                                              ped_ns ? S_NS_W : S_NS_G;
            S_EW_G: if (min_done && (!car_ew || max_done)) nxt_state = S_EW_Y;
            S_EW_Y: if (yel_done) nxt_state = ped_ns ? S_NS_W : S_NS_G;
            S_NS_W: if (red_done) nxt_state = S_NS_G;
            default: nxt_state = S_NS_G;
        endcase
    end

    assign timer_clr = (nxt_state != state);

    // no point counting while NS rests in green with nobody waiting
    always @(*) timer_en = !(state == S_NS_G && !ns_busy && min_done);

    always @(*) begin
        ns_lights = R;
        ew_lights = R;
        walk_ns   = 1'b0;
        walk_ew   = 1'b0;
        case (state)
            S_NS_G: ns_lights = G;
            S_NS_Y: ns_lights = Y;
            S_EW_W: walk_ew   = 1'b1;
            S_EW_G: ew_lights = G;
            S_EW_Y: ew_lights = Y;
            S_NS_W: walk_ns   = 1'b1;
            default: ;
        endcase
    end
endmodule


module top #(
    parameter BITS        = 16,
    parameter T_GREEN_MIN = 16'd6,
    parameter T_GREEN     = 16'd15,      // old name, kept so existing tb overrides still work
    parameter T_GREEN_MAX = T_GREEN,
    parameter T_YELLOW    = 16'd4,
    parameter T_RED       = 16'd6
)(
    input clk,
    input rst_n,
    input car_sensor_ns,
    input car_sensor_ew,
    input ped_call_ns,
    input ped_call_ew,
    output [2:0] ns_lights,
    output [2:0] ew_lights,
    output walk_ns,
    output walk_ew
);
    wire [BITS-1:0] cnt;
    wire timer_clr, timer_en;

    fsm #(
        .W(BITS),
        .T_GREEN_MIN(T_GREEN_MIN),
        .T_GREEN_MAX(T_GREEN_MAX),
        .T_YELLOW(T_YELLOW),
        .T_RED(T_RED)
    ) controller (
        .clk(clk), .rst_n(rst_n), .cnt(cnt),
        .car_sensor_ns(car_sensor_ns), .car_sensor_ew(car_sensor_ew),
        .ped_call_ns(ped_call_ns),     .ped_call_ew(ped_call_ew),
        .ns_lights(ns_lights), .ew_lights(ew_lights),
        .walk_ns(walk_ns),     .walk_ew(walk_ew),
        .timer_clr(timer_clr), .timer_en(timer_en)
    );

    counter #(.W(BITS)) timer (
        .clk(clk), .rst_n(rst_n),
        .en(timer_en), .clr(timer_clr),
        .cnt(cnt)
    );
endmodule
