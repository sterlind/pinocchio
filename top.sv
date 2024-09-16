`include "sm83.sv"

typedef enum logic [15:0] {
    ROM     = 16'b0xxx_xxxx_xxxx_xxxx,
    VRAM    = 16'b100x_xxxx_xxxx_xxxx,
    WRAM    = 16'b110x_xxxx_xxxx_xxxx
} mem_mask;

module top(
    input wire rst,
    input wire clk,
    output reg [14:0] rom_addr,
    input wire [7:0] rom_data
);
    reg [12:0] vram_addr, wram_addr;
    reg [7:0] wram_in, vram_in;
    reg wram_write, vram_write;
    ram #(.D(13)) vram (.clk(clk), .addr(vram_addr), .d_in(vram_in), .write(vram_write));
    ram #(.D(13)) wram (.clk(clk), .addr(wram_addr), .d_in(wram_in), .write(wram_write));

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

    always_comb begin
        bus_out = 'x;
        rom_addr = 'x;
        casex (cpu.addr)
            ROM: begin rom_addr = cpu.addr; bus_out = rom_data; end
            VRAM: begin vram_addr = cpu.addr; vram_write = cpu.write; vram_in = cpu.d_out; bus_out = vram.d_out; end
            WRAM: begin wram_addr = cpu.addr; wram_write = cpu.write; wram_in = cpu.d_out; bus_out = wram.d_out; end
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
    always_ff @(posedge clk) ctr <= ctr + 1'b1;
    assign div_clk = ~|ctr;
endmodule