// Traffic Light Controller (Verilog)
// 4-Way Intersection FSM with Pedestrian Crossing Phases

module counter #(
    parameter BITS = 16
)(
    input clk,
    input rst_n,                // Synchronous active-low reset
    input en,                   // Count enable
    input clr,                  // Synchronous clear
    input [BITS-1:0] target,    // Target cycle count
    output pulse                // Terminal count pulse (high in terminal cycle)
);

    reg [BITS-1:0] count_reg;

    // Pulse is asserted during the target-th clock cycle
    assign pulse = (en && (count_reg >= target - 1'b1));

    always @(posedge clk) begin
        if (!rst_n || clr) begin
            count_reg <= {BITS{1'b0}};
        end else if (en) begin
            if (count_reg >= target - 1'b1) begin
                count_reg <= {BITS{1'b0}};
            end else begin
                count_reg <= count_reg + 1'b1;
            end
        end
    end

endmodule

module fsm #(
    parameter W = 16
)(
    input clk,
    input rst_n,                    // Synchronous active-low reset
    input timer_done,               // Signal from counter indicating terminal cycle
    input [W-1:0] T_GREEN,          // Duration for Green state (clock cycles)
    input [W-1:0] T_YELLOW,         // Duration for Yellow state (clock cycles)
    input [W-1:0] T_RED,            // Duration for Pedestrian Walk state (clock cycles)
    output reg [2:0] ns_lights,     // {Red, Yellow, Green}
    output reg [2:0] ew_lights,     // {Red, Yellow, Green}
    output reg walk_ns,             // Active-high pedestrian walk signal for NS
    output reg walk_ew,             // Active-high pedestrian walk signal for EW
    output reg [W-1:0] load_val     // Target count provided to counter
);

    // State encoding (Verilog 2001 localparam)
    localparam S0_NS_G = 3'd0; // North-South Green, East-West Red
    localparam S1_NS_Y = 3'd1; // North-South Yellow, East-West Red
    localparam S2_EW_W = 3'd2; // Pedestrian Walk EW, vehicular all-Red
    localparam S3_EW_G = 3'd3; // East-West Green, North-South Red
    localparam S4_EW_Y = 3'd4; // East-West Yellow, North-South Red
    localparam S5_NS_W = 3'd5; // Pedestrian Walk NS, vehicular all-Red

    reg [2:0] curr, next;

    // State register with synchronous active-low reset
    always @(posedge clk) begin
        if (!rst_n)
            curr <= S0_NS_G;
        else
            curr <= next;
    end

    // Next-state logic
    always @(*) begin
        next = curr;
        if (timer_done) begin
            case (curr)
                S0_NS_G: next = S1_NS_Y;
                S1_NS_Y: next = S2_EW_W;
                S2_EW_W: next = S3_EW_G;
                S3_EW_G: next = S4_EW_Y;
                S4_EW_Y: next = S5_NS_W;
                S5_NS_W: next = S0_NS_G;
                default: next = S0_NS_G;
            endcase
        end
    end

    // Output logic: Lights encoded as 3'b[RED, YELLOW, GREEN]
    always @(*) begin
        // Default safe values (All Red, no walk)
        ns_lights = 3'b100;
        ew_lights = 3'b100;
        walk_ns   = 1'b0;
        walk_ew   = 1'b0;
        load_val  = T_GREEN;

        case (curr)
            S0_NS_G: begin
                ns_lights = 3'b001; // NS Green
                ew_lights = 3'b100; // EW Red
                load_val  = T_GREEN;
            end
            S1_NS_Y: begin
                ns_lights = 3'b010; // NS Yellow
                ew_lights = 3'b100; // EW Red
                load_val  = T_YELLOW;
            end
            S2_EW_W: begin
                ns_lights = 3'b100; // NS Red
                ew_lights = 3'b100; // EW Red
                walk_ew   = 1'b1;   // EW Pedestrian Walk
                load_val  = T_RED;
            end
            S3_EW_G: begin
                ns_lights = 3'b100; // NS Red
                ew_lights = 3'b001; // EW Green
                load_val  = T_GREEN;
            end
            S4_EW_Y: begin
                ns_lights = 3'b100; // NS Red
                ew_lights = 3'b010; // EW Yellow
                load_val  = T_YELLOW;
            end
            S5_NS_W: begin
                ns_lights = 3'b100; // NS Red
                ew_lights = 3'b100; // EW Red
                walk_ns   = 1'b1;   // NS Pedestrian Walk
                load_val  = T_RED;
            end
            default: begin
                ns_lights = 3'b100;
                ew_lights = 3'b100;
                walk_ns   = 1'b0;
                walk_ew   = 1'b0;
                load_val  = T_GREEN;
            end
        endcase
    end

endmodule

module top #(
    parameter BITS     = 16,
    parameter T_GREEN  = 16'd10,
    parameter T_YELLOW = 16'd4,
    parameter T_RED    = 16'd6
)(
    input clk,
    input rst_n,

    output [2:0] ns_lights,
    output [2:0] ew_lights,
    output walk_ns,
    output walk_ew
);

    wire pulse_w;
    wire [BITS-1:0] timer_target;

    // FSM Controller Instance
    fsm #(
        .W(BITS)
    ) controller (
        .clk(clk),
        .rst_n(rst_n),
        .timer_done(pulse_w),
        .T_GREEN(T_GREEN),
        .T_YELLOW(T_YELLOW),
        .T_RED(T_RED),
        .ns_lights(ns_lights),
        .ew_lights(ew_lights),
        .walk_ns(walk_ns),
        .walk_ew(walk_ew),
        .load_val(timer_target)
    );

    // Timer Instance
    counter #(
        .BITS(BITS)
    ) timer_inst (
        .clk(clk),
        .rst_n(rst_n),
        .en(1'b1),
        .clr(1'b0),
        .target(timer_target),
        .pulse(pulse_w)
    );

endmodule
