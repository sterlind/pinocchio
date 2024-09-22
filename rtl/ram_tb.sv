`include "ram.sv"
module ram_tb;
    reg clk;
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end

    reg [7:0] addr, d_rw;
    reg write;
    ram #(.WORDS(256), .WIDTH(8)) mem (
        .clk(clk),
        .addr(addr),
        .write(write),
        .d_in(d_rw)
    );

    initial begin
        $dumpvars(0, mem);
        #1;
        addr = 8'h0; d_rw = 8'h02; write = 1;
        #1;
        addr = 8'h1; d_rw = 8'h03; write = 1;
        #1;
        addr = 8'h2; d_rw = 8'h05; write = 1;
        #1;
        write = 0; addr = 8'h0;
        #1;
        write = 0; addr = 8'h1;
        #1;
        write = 0; addr = 8'h2;
        #1;
        $finish;
    end
endmodule