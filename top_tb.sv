`include "sm83.sv"

module top_tb;
    reg clk, rst, write;
    wire [15:0] addr;
    reg [7:0] mem [0:'hffff];
    reg [7:0] d_in, d_out;

    sm83 cpu (
        .clk(clk),
        .rst(rst),
        .addr(addr),
        .d_in(d_in),
        .d_out(d_out),
        .write(write)
    );

    always_ff @(negedge clk) begin
        if (write) mem[addr] <= d_out;
        d_in <= mem[addr];
    end

    initial begin
        $readmemh("rom.hex", mem, 0, 5);
        clk = 1; rst = 0;       // cpu's comb logic sets addr = 0 and write = 0.
        #1; clk = 0;            // Falling edge loads d_in from addr 0.
        clk = 1; #1;            // Raise clock. cpu latches ir from d_in and zeros regs, since ~rst.
        rst = 1; clk = 0; #1;   // Reset is no longer asserted. cpu executes first op (comb), and its memory req is serviced by falling edge.
        forever #1 clk = ~clk;  // Should be up and running now!
    end

    initial begin
        $dumpfile("top_tb.vcd");
        $dumpvars(0, cpu);
        #20;
        $finish;
    end

endmodule