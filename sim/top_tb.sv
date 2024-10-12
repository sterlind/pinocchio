`timescale 1ns / 1ps
module top_tb;
    initial
        $display("Hello world");

    reg tclk;
    reg nrst;
    initial begin
        tclk = 0; nrst = 0;
        forever #1 tclk = ~tclk;
    end

    initial #4 nrst = 1;

    wire [14:0] rom_addr;
    wire [7:0] rom_data;
    cart_prom cart (
        .clk(tclk),
        .oce('x),
        .ce(1'b1),
        .reset(1'b0),
        .ad(rom_addr),
        .dout(rom_data)
    );

    dmg_main dmg(
        .clk(tclk),
        .rst(nrst),
        .rom_addr(rom_addr),
        .rom_data(rom_data)
    );

    initial begin
        $dumpvars(0, dmg);
        #(1024 * 1024 * 4);
        $display("Done");
        $finish;
    end
endmodule