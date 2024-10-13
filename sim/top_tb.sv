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
    cart_test_prom cart (
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

    reg [15:0] r_af, r_bc, r_de, r_hl, r_sp, r_pc, r_zw;
    assign r_af = {dmg.cpu.rf[A], dmg.cpu.flags};
    assign r_bc = {dmg.cpu.rf[B], dmg.cpu.rf[C]};
    assign r_de = {dmg.cpu.rf[D], dmg.cpu.rf[E]};
    assign r_hl = {dmg.cpu.rf[H], dmg.cpu.rf[L]};
    assign r_sp = {dmg.cpu.rf[SPH], dmg.cpu.rf[SPL]};
    assign r_pc = {dmg.cpu.rf[PCH], dmg.cpu.rf[PCL]};
    assign r_zw = {dmg.cpu.rf[Z], dmg.cpu.rf[W]};

    initial begin
        $dumpvars(0, dmg);
        $dumpvars(0, r_af, r_bc, r_de, r_hl, r_sp, r_pc, r_zw);
        #(600 * 1024 * 4);
        $display("Done");
        $finish;
    end
endmodule