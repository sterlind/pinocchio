`timescale 1ns / 1ps
module top_tb;
    initial
        $display("Hello world");

    reg clk;
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end

    top t (
        .clk_27m(clk)
    );

    initial begin
        $dumpvars(0, t);
        #1024;
        $display("Done");
        $finish;
    end
endmodule