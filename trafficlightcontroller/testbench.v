module toptb();
    reg clk;
    reg rst_n;
    wire [2:0] ns, ew;
    wire w_ns, w_ew;

    top uut (
        .clk(clk),
        .rst_n(rst_n),
        .ns_lights(ns),
        .ew_lights(ew),
        .walk_ns(w_ns),
        .walk_ew(w_ew)
    );

    always #5 clk = ~clk;

    initial begin
        // Setup wave dumping for GTKWave
        $dumpfile("simulation.vcd");
        $dumpvars(0, toptb);

        clk = 0;
        rst_n = 0;
        #20 rst_n = 1;
        #1000;
        $display("Simulation finished.");
        $finish;
    end
endmodule
