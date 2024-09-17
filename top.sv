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

    reg clk_ram;
    assign clk_ram = ~clk_cpu;

    reg hram_write;
    reg [7:0] hram_in;
    ram #(.D(7)) hram (
        .clk(clk_ram),
        .addr(cpu.addr[6:0]),
        .d_in(hram_in),
        .write(hram_write)
    );

    always_comb begin
        bus_out = 'x;
        rom_addr = 'x;
        hram_write = 0;
        hram_in = 'x;
        ppu_reg_write = 0;
        casex (cpu.addr)
            HRAM: begin bus_out = hram.d_out; hram_write = cpu.write; hram_in = cpu.d_out; end
            ROM: begin rom_addr = cpu.addr; bus_out = rom_data; end
            PPU_REG: begin bus_out = ppu_reg.reg_out; ppu_reg_write = cpu.write; end
        endcase
    end
endmodule

module ram #(parameter D = 8) (
    input wire clk,
    input wire [(D-1):0] addr,
    input wire write,
    input byte d_in,
    output byte d_out
);
    localparam BYTES = 1 << D;
    reg [7:0] mem [0:BYTES-1];
    always_ff @(posedge clk) begin
        d_out <= mem[addr];
        if (write) mem[addr] <= d_in;
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