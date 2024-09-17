`include "top.sv"

module top_tb;
    reg clk, rst, write;


    wire [14:0] rom_addr;
    rom tetris(.clk(clk), .addr(rom_addr));

    top t(
        .rst(rst),
        .clk(clk),
        .rom_addr(rom_addr),
        .rom_data(tetris.data)
    );

    initial begin
        clk = 0; rst = 0;
        forever #1 clk = ~clk;
    end
    initial begin
        #4 rst = 1;
    end

    genvar k;
    reg [15:0] bc, de, hl, sp, pc, af, wz;
    always_comb begin
        pc = {t.cpu.rf[PCH], t.cpu.rf[PCL]};
        af = {t.cpu.rf[A], t.cpu.flags, 4'b0};
        bc = {t.cpu.rf[B], t.cpu.rf[C]};
        de = {t.cpu.rf[D], t.cpu.rf[E]};
        hl = {t.cpu.rf[H], t.cpu.rf[L]};
        sp = {t.cpu.rf[SPH], t.cpu.rf[SPL]};
        wz = {t.cpu.rf[W], t.cpu.rf[Z]};
    end

    initial begin
        $dumpvars(0, t);
        $dumpvars(0, tetris);
        $dumpvars(0, bc);
        $dumpvars(0, de);
        $dumpvars(0, hl);
        $dumpvars(0, af);
        $dumpvars(0, sp);
        $dumpvars(0, pc);
        $dumpvars(0, wz);
    end

    // 20874
    initial #('h208f4*4) $finish;
endmodule

module rom(
    input wire clk,
    input wire [14:0] addr,
    output reg [7:0] data
);
    reg [7:0] mem [0:'h7fff];
    int fd, code;
    initial begin
        fd = $fopen("tetris.gb", "rb");
        code = $fread(mem, fd, 0, 'h7fff);
        $fclose(fd);
        // Boot rom overlaps cartridge rom.
        $readmemh("dmg_rom.hex", mem, 0, 255);
    end
    always_ff @(posedge clk) data <= mem[addr];
endmodule