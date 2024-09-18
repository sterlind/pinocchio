`include "sm83.sv"
`include "ppu.sv"

typedef enum logic [15:0] {
    ROM     = 16'b0xxx_xxxx_xxxx_xxxx,
    VRAM    = 16'b100x_xxxx_xxxx_xxxx,
    WRAM    = 16'b110x_xxxx_xxxx_xxxx,
    HRAM    = 16'b1111_1111_1xxx_xxxx,
    PPU_REG = 16'hff4x
} mem_mask;

module top(
    input wire rst,
    input wire clk,
    output reg [14:0] rom_addr,
    input wire [7:0] rom_data
);
    wire clk_cpu;
    clkdiv4 clk_cpu_div (
        .clk(clk),
        .div_clk(clk_cpu)
    );

    reg [7:0] bus_in;
    sm83 cpu (
        .clk(clk_cpu),
        .rst(rst),
        .d_in(bus_in)
    );

    reg ppu_reg_write, vram_write;
    ppu_m ppu (
        .clk(clk),
        .reg_addr(cpu.addr[3:0]),
        .reg_write(ppu_reg_write),
        .reg_d_wr(cpu.d_out),
        .vram_addr_in(cpu.addr[12:0]),
        .vram_write_in(vram_write),
        .vram_d_wr(cpu.d_out)
    );

    reg clk_ram;
    assign clk_ram = ~clk_cpu;

    reg hram_write;
    reg [7:0] hram_in;
    ram #(.WORDS(128)) hram (
        .clk(clk_ram),
        .addr(cpu.addr[6:0]),
        .d_in(cpu.d_out),
        .write(hram_write)
    );

    always_comb begin
        hram_write = 0; ppu_reg_write = 0; vram_write = 0; ppu_reg_write = 0;
        bus_in = 'x;
        rom_addr = 'x;
        casex (cpu.addr)
            VRAM: begin bus_in = ppu.vram_d_rd; vram_write = cpu.write; end
            HRAM: begin bus_in = hram.d_out; hram_write = cpu.write; end
            ROM: begin rom_addr = cpu.addr; bus_in = rom_data; end
            PPU_REG: begin bus_in = ppu.reg_d_rd; ppu_reg_write = cpu.write; end
        endcase
    end
endmodule

module clkdiv4(
    input wire clk,
    output wire div_clk
);
    reg [1:0] ctr;
    initial ctr = 0;
    always_ff @(posedge clk) ctr <= ctr + 1'b1;
    assign div_clk = ctr[1];
endmodule