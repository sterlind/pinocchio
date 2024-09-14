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
        clk = 0;
        $readmemh("rom.hex", mem);
        rst = 0;
        #2;
        rst = 1;
        forever #1 clk = ~clk;
    end

    initial begin
        $dumpfile("top_tb.vcd");
        $dumpvars(0, cpu);
        #20;
        $finish;
    end

endmodule