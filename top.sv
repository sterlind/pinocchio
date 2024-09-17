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
    reg [12:0] vram_addr, wram_addr;
    reg [7:0] wram_in, vram_in;
    reg wram_write, vram_write, hram_write;
    ram #(.D(13)) vram (.clk(clk), .addr(vram_addr), .d_in(vram_in), .write(vram_write));
    ram #(.D(13)) wram (.clk(clk), .addr(wram_addr), .d_in(wram_in), .write(wram_write));
    ram #(.D(7)) hram (.clk(~clk_cpu), .addr(cpu.addr[6:0]), .d_in(cpu.d_out), .write(hram_write));

    wire clk_cpu;
    clkdiv4 clk_cpu_div (
        .clk(clk),
        .div_clk(clk_cpu)
    );

    reg [7:0] bus_out;
    sm83 cpu (
        .clk(clk_cpu),
        .rst(rst),
        .d_in(bus_out)
    );

    reg ppu_reg_write;
    ppu_ctrl ppu_reg (
        .clk(clk),
        .reg_addr(cpu.addr[3:0]),
        .reg_in(cpu.d_out),
        .reg_write(ppu_reg_write),
        .ly(ppu.ly)
    );

    ppu_engine ppu (
        .clk(clk),
        .lcdc(ppu_reg.lcdc)
    );

    always_comb begin
        bus_out = 'x;
        hram_write = 0; wram_write = 0; vram_write = 0;
        rom_addr = 'x;
        casex (cpu.addr)
            HRAM: begin hram_write = cpu.write; bus_out = hram.d_out; end
            ROM: begin rom_addr = cpu.addr; bus_out = rom_data; end
            VRAM: begin vram_addr = cpu.addr; vram_write = cpu.write; vram_in = cpu.d_out; bus_out = vram.d_out; end
            WRAM: begin wram_addr = cpu.addr; wram_write = cpu.write; wram_in = cpu.d_out; bus_out = wram.d_out; end
            PPU_REG: begin bus_out = ppu_reg.reg_out; ppu_reg_write = cpu.write; end
        endcase
    end
endmodule

module ram #(
    parameter D = 12
) (
    input wire clk,
    input wire [D-1:0] addr,
    input wire [7:0] d_in,
    output wire [7:0] d_out,
    input wire write
);
    reg mem [0:2**D-1];
    always_ff @(posedge clk)
        if (write) mem[addr] <= d_in;
    assign d_out = mem[addr];
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