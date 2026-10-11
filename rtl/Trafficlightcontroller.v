`timescale 1ns/1ps

// Traffic Light Controller (Verilog)
// Smart Actuated 4-Way Intersection FSM with Dynamic Vehicle Detection,
// Pedestrian Call Push-Buttons, Gap-Out Timers, and Conditional Phase Skipping.

module counter #(
    parameter BITS = 16
)(
    input clk,
    input rst_n,                // Synchronous active-low reset
    input en,                   // Count enable
    input clr,                  // Synchronous clear
    input [BITS-1:0] target,    // Target cycle count (for pulse generation)
    output reg [BITS-1:0] count,// Current elapsed count value
    output pulse                // Terminal count pulse
);

    assign pulse = (en && (count >= target - 1'b1));

    always @(posedge clk) begin
        if (!rst_n || clr) begin
            count <= {BITS{1'b0}};
        end else if (en) begin
            if (count != {BITS{1'b1}}) begin
                count <= count + 1'b1;
            end
        end
    end

endmodule

module fsm #(
    parameter W = 16
)(
    input clk,
    input rst_n,                    // Synchronous active-low reset
    input [W-1:0] timer_count,      // Current cycle count from counter
    input [W-1:0] T_GREEN_MIN,      // Minimum Green duration (clock cycles)
    input [W-1:0] T_GREEN_MAX,      // Maximum Green duration (clock cycles)
    input [W-1:0] T_YELLOW,         // Yellow duration (clock cycles)
    input [W-1:0] T_RED,            // Pedestrian Walk duration (clock cycles)
    input car_sensor_ns,            // Vehicle sensor North-South
    input car_sensor_ew,            // Vehicle sensor East-West
    input ped_call_ns,              // Pedestrian call button North-South
    input ped_call_ew,              // Pedestrian call button East-West
    output reg [2:0] ns_lights,     // {Red, Yellow, Green}
    output reg [2:0] ew_lights,     // {Red, Yellow, Green}
    output reg walk_ns,             // Active-high pedestrian walk signal for NS
    output reg walk_ew,             // Active-high pedestrian walk signal for EW
    output reg [W-1:0] load_val,    // Target count provided to counter
    output timer_clr,               // Clear signal to counter on state transition
    output reg timer_en             // Count enable signal
);

    // State encoding (Verilog 2001 localparam)
    localparam S0_NS_G = 3'd0; // North-South Green, East-West Red
    localparam S1_NS_Y = 3'd1; // North-South Yellow, East-West Red
    localparam S2_EW_W = 3'd2; // Pedestrian Walk EW, vehicular all-Red
    localparam S3_EW_G = 3'd3; // East-West Green, North-South Red
    localparam S4_EW_Y = 3'd4; // East-West Yellow, North-South Red
    localparam S5_NS_W = 3'd5; // Pedestrian Walk NS, vehicular all-Red

    reg [2:0] curr, next;

    // Latched call registers to capture brief pulses
    reg ped_call_ns_reg;
    reg ped_call_ew_reg;
    reg car_call_ns_reg;
    reg car_call_ew_reg;

    // Combined effective demand signals (latched register OR active sensor)
    wire ped_ns_pending = ped_call_ns_reg | ped_call_ns;
    wire ped_ew_pending = ped_call_ew_reg | ped_call_ew;
    wire car_ns_pending = car_call_ns_reg | car_sensor_ns;
    wire car_ew_pending = car_call_ew_reg | car_sensor_ew;

    wire ew_any_demand  = car_ew_pending | ped_ew_pending;
    wire ns_conflicting_demand = ew_any_demand | ped_ns_pending;

    // Latch management: capture calls when not in service, clear when served
    always @(posedge clk) begin
        if (!rst_n) begin
            ped_call_ns_reg <= 1'b0;
            ped_call_ew_reg <= 1'b0;
            car_call_ns_reg <= 1'b0;
            car_call_ew_reg <= 1'b0;
        end else begin
            // Pedestrian NS Call
            if (curr == S5_NS_W)
                ped_call_ns_reg <= 1'b0;
            else if (ped_call_ns)
                ped_call_ns_reg <= 1'b1;

            // Pedestrian EW Call
            if (curr == S2_EW_W)
                ped_call_ew_reg <= 1'b0;
            else if (ped_call_ew)
                ped_call_ew_reg <= 1'b1;

            // Vehicle NS Call (registered when NS is not green)
            if (curr == S0_NS_G)
                car_call_ns_reg <= 1'b0;
            else if (car_sensor_ns)
                car_call_ns_reg <= 1'b1;

            // Vehicle EW Call (registered when EW is not green)
            if (curr == S3_EW_G)
                car_call_ew_reg <= 1'b0;
            else if (car_sensor_ew)
                car_call_ew_reg <= 1'b1;
        end
    end

    // State register with synchronous active-low reset
    always @(posedge clk) begin
        if (!rst_n)
            curr <= S0_NS_G;
        else
            curr <= next;
    end

    // Parameter validation checks during elaboration / simulation
    initial begin
        if (T_GREEN_MIN < 16'd1) begin
            $display("[ERROR] Invalid parameter: T_GREEN_MIN (%0d) must be >= 1", T_GREEN_MIN);
            $finish;
        end
        if (T_GREEN_MAX < T_GREEN_MIN) begin
            $display("[ERROR] Invalid parameter: T_GREEN_MAX (%0d) cannot be less than T_GREEN_MIN (%0d)", T_GREEN_MAX, T_GREEN_MIN);
            $finish;
        end
        if (T_YELLOW < 16'd1) begin
            $display("[ERROR] Invalid parameter: T_YELLOW (%0d) must be >= 1", T_YELLOW);
            $finish;
        end
        if (T_RED < 16'd1) begin
            $display("[ERROR] Invalid parameter: T_RED (%0d) must be >= 1", T_RED);
            $finish;
        end
    end

    // Combinational next-state logic with gap-out and phase skipping
    always @(*) begin
        next = curr;

        case (curr)
            S0_NS_G: begin
                // Check if minimum green requirement is met
                if (timer_count >= T_GREEN_MIN - 1'b1) begin
                    // Only transition if there is conflicting demand (EW traffic or pedestrian call)
                    if (ns_conflicting_demand) begin
                        // Transition if traffic gaps out (!car_ns_pending) OR max green expired
                        if (!car_ns_pending || (timer_count >= T_GREEN_MAX - 1'b1)) begin
                            next = S1_NS_Y;
                        end
                    end
                end
            end

            S1_NS_Y: begin
                if (timer_count >= T_YELLOW - 1'b1) begin
                    // Conditional phase skipping logic:
                    if (ped_ew_pending) begin
                        next = S2_EW_W; // Service EW pedestrian
                    end else if (car_ew_pending) begin
                        next = S3_EW_G; // Skip EW walk, service EW vehicular green
                    end else if (ped_ns_pending) begin
                        next = S5_NS_W; // Skip EW side street entirely, service NS walk
                    end else begin
                        next = S0_NS_G; // Skip EW entirely, return to main street green
                    end
                end
            end

            S2_EW_W: begin
                if (timer_count >= T_RED - 1'b1) begin
                    if (car_ew_pending) begin
                        next = S3_EW_G; // EW vehicular demand present
                    end else if (ped_ns_pending) begin
                        next = S5_NS_W; // Skip EW vehicular green, service NS walk
                    end else begin
                        next = S0_NS_G; // Skip EW vehicular green, return to NS green
                    end
                end
            end

            S3_EW_G: begin
                // Check if minimum green requirement is met for side street
                if (timer_count >= T_GREEN_MIN - 1'b1) begin
                    // Gap-out early if no EW traffic, or max-out when T_GREEN_MAX reached
                    if (!car_ew_pending || (timer_count >= T_GREEN_MAX - 1'b1)) begin
                        next = S4_EW_Y;
                    end
                end
            end

            S4_EW_Y: begin
                if (timer_count >= T_YELLOW - 1'b1) begin
                    // Conditional phase skipping:
                    if (ped_ns_pending) begin
                        next = S5_NS_W; // Service NS pedestrian
                    end else begin
                        next = S0_NS_G; // Skip NS walk, return to main street green
                    end
                end
            end

            S5_NS_W: begin
                if (timer_count >= T_RED - 1'b1) begin
                    next = S0_NS_G; // Walk interval completed, return to NS green
                end
            end

            default: next = S0_NS_G;
        endcase
    end

    // Timer enable logic: holds at T_GREEN_MIN-1 when resting in NS Green without demand
    always @(*) begin
        timer_en = 1'b1;
        if (curr == S0_NS_G && !ns_conflicting_demand && (timer_count >= T_GREEN_MIN - 1'b1)) begin
            timer_en = 1'b0;
        end
    end

    // Synchronous timer clear asserted whenever state transitions
    assign timer_clr = (next != curr);

    // Output logic: Lights encoded as 3'b[RED, YELLOW, GREEN]
    always @(*) begin
        // Default safe values (All Red, no walk)
        ns_lights = 3'b100;
        ew_lights = 3'b100;
        walk_ns   = 1'b0;
        walk_ew   = 1'b0;
        load_val  = T_GREEN_MAX;

        case (curr)
            S0_NS_G: begin
                ns_lights = 3'b001; // NS Green
                ew_lights = 3'b100; // EW Red
                walk_ns   = 1'b0;
                walk_ew   = 1'b0;
                load_val  = T_GREEN_MAX;
            end
            S1_NS_Y: begin
                ns_lights = 3'b010; // NS Yellow
                ew_lights = 3'b100; // EW Red
                walk_ns   = 1'b0;
                walk_ew   = 1'b0;
                load_val  = T_YELLOW;
            end
            S2_EW_W: begin
                ns_lights = 3'b100; // NS Red
                ew_lights = 3'b100; // EW Red
                walk_ns   = 1'b0;
                walk_ew   = 1'b1;   // EW Pedestrian Walk
                load_val  = T_RED;
            end
            S3_EW_G: begin
                ns_lights = 3'b100; // NS Red
                ew_lights = 3'b001; // EW Green
                walk_ns   = 1'b0;
                walk_ew   = 1'b0;
                load_val  = T_GREEN_MAX;
            end
            S4_EW_Y: begin
                ns_lights = 3'b100; // NS Red
                ew_lights = 3'b010; // EW Yellow
                walk_ns   = 1'b0;
                walk_ew   = 1'b0;
                load_val  = T_YELLOW;
            end
            S5_NS_W: begin
                ns_lights = 3'b100; // NS Red
                ew_lights = 3'b100; // EW Red
                walk_ns   = 1'b1;   // NS Pedestrian Walk
                walk_ew   = 1'b0;
                load_val  = T_RED;
            end
            default: begin
                ns_lights = 3'b100;
                ew_lights = 3'b100;
                walk_ns   = 1'b0;
                walk_ew   = 1'b0;
                load_val  = T_GREEN_MAX;
            end
        endcase
    end

endmodule

module top #(
    parameter BITS        = 16,
    parameter T_GREEN_MIN = 16'd6,
    parameter T_GREEN     = 16'd15,  // Backward compatibility alias for green duration
    parameter T_GREEN_MAX = T_GREEN, // Defaults to T_GREEN unless explicitly overridden
    parameter T_YELLOW    = 16'd4,
    parameter T_RED       = 16'd6
)(
    input clk,
    input rst_n,

    // Smart actuation sensor & button inputs
    input car_sensor_ns,
    input car_sensor_ew,
    input ped_call_ns,
    input ped_call_ew,

    // Vehicular and pedestrian outputs
    output [2:0] ns_lights,
    output [2:0] ew_lights,
    output walk_ns,
    output walk_ew
);

    wire timer_pulse_w;
    wire [BITS-1:0] timer_count_w;
    wire [BITS-1:0] timer_target_w;
    wire timer_clr_w;
    wire timer_en_w;

    // FSM Controller Instance
    fsm #(
        .W(BITS)
    ) controller (
        .clk(clk),
        .rst_n(rst_n),
        .timer_count(timer_count_w),
        .T_GREEN_MIN(T_GREEN_MIN),
        .T_GREEN_MAX(T_GREEN_MAX),
        .T_YELLOW(T_YELLOW),
        .T_RED(T_RED),
        .car_sensor_ns(car_sensor_ns),
        .car_sensor_ew(car_sensor_ew),
        .ped_call_ns(ped_call_ns),
        .ped_call_ew(ped_call_ew),
        .ns_lights(ns_lights),
        .ew_lights(ew_lights),
        .walk_ns(walk_ns),
        .walk_ew(walk_ew),
        .load_val(timer_target_w),
        .timer_clr(timer_clr_w),
        .timer_en(timer_en_w)
    );

    // Timer Instance
    counter #(
        .BITS(BITS)
    ) timer_inst (
        .clk(clk),
        .rst_n(rst_n),
        .en(timer_en_w),
        .clr(timer_clr_w),
        .target(timer_target_w),
        .count(timer_count_w),
        .pulse(timer_pulse_w)
    );

endmodule
