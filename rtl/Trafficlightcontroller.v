module counter #(
    parameter BITS = 16
)(
    input clk,
    input rst_n,     // using active low reset convention
    input en,
    input [BITS-1:0] target,
    output reg pulse
);

    reg [BITS-1:0] count_reg;

    always @(posedge clk) begin
        if (!rst_n) begin
            count_reg <= 0;
            pulse <= 0;
        end else if (en) begin
            if (count_reg == target) begin
                count_reg <= 0; // reset once we hit the limit
                pulse <= 1;
            end else begin
                count_reg <= count_reg + 1;
                pulse <= 0;
            end
        end else begin
            pulse <= 0;
        end
    end
endmodule

module fsm #(
    parameter W = 16
)(
    input clk,
    input rst_n,
    input timer_done,
    input [W-1:0] T_GREEN,
    input [W-1:0] T_YELLOW,
    input [W-1:0] T_RED,
    output reg [2:0] ns_lights,
    output reg [2:0] ew_lights,
    output reg walk_ns, walk_ew,
    output reg [W-1:0] load_val
);

    // State encoding using localparam (Verilog 2001 style)
    localparam S0_NS_G = 3'd0;
    localparam S1_NS_Y = 3'd1;
    localparam S2_EW_W = 3'd2;
    localparam S3_EW_G = 3'd3;
    localparam S4_EW_Y = 3'd4;
    localparam S5_NS_W = 3'd5;

    reg [2:0] curr, next;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) curr <= S0_NS_G;
        else curr <= next;
    end

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

    always @(*) begin
        ns_lights = 3'b100;
        ew_lights = 3'b100;
        walk_ns = 0;
        walk_ew = 0;
        load_val = T_GREEN;
        case (curr)
            S0_NS_G: begin ns_lights = 3'b001; load_val = T_GREEN; end
            S1_NS_Y: begin ns_lights = 3'b010; load_val = T_YELLOW; end
            S2_EW_W: begin walk_ew = 1; load_val = T_RED; end
            S3_EW_G: begin ew_lights = 3'b001; load_val = T_GREEN; end
            S4_EW_Y: begin ew_lights = 3'b010; load_val = T_YELLOW; end
            S5_NS_W: begin walk_ns = 1; load_val = T_RED; end
        endcase
    end
endmodule

module top (
    input clk,
    input rst_n,

    output [2:0] ns_lights,
    output [2:0] ew_lights,
    output walk_ns,
    output walk_ew
);

    wire pulse_w;
    wire [15:0] timer_target;

    // Controller instance
    fsm #(.W(16)) controller (
        .clk(clk),
        .rst_n(rst_n),
        .timer_done(pulse_w),
        .T_GREEN(16'd10),  // Hardcoded these for now, maybe add ports later
        .T_YELLOW(16'd4),
        .T_RED(16'd6),
        .ns_lights(ns_lights),
        .ew_lights(ew_lights),
        .walk_ns(walk_ns),
        .walk_ew(walk_ew),
        .load_val(timer_target)
    );

    // Physical timer instance
    counter #(.BITS(16)) timer_inst (
        .clk(clk),
        .rst_n(rst_n),
        .en(1'b1), // always run
        .target(timer_target),
        .pulse(pulse_w)
    );

endmodule

