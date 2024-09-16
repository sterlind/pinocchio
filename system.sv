`include "sm83.sv"
`include "ppu.sv"

typedef enum logic [15:0] {
    ROM:    16'b0xxx_xxxx_xxxx_xxxx,
    VRAM:   16'b100x_xxxx_xxxx_xxxx,
    ERAM:   16'b101x_xxxx_xxxx_xxxx,
    WRAM:   16'b110x_xxxx_xxxx_xxxx,
    OAM:    16'hfexx,
    PPU:    16'hff4x
} bus_addr_t;

module soc(
    input wire clk,
    input wire rst,
    output wire [14:0] rom_addr,
    input wire [7:0] rom_data
);
    wire cpu_clk;
    clkdiv4 div4 (
        .clk(clk),
        .div(cpu_clk)
    );

    ram_8k vram (.clk(clk));
    always_comb begin
        vram.write = 0;
        if (ppu.read) casex (ppu.addr)
            VRAM: begin vram.addr = ppu.addr; ppu.data = vram.data; end
        endcase
    end
    ram_8k wram (.clk(clk));

    sm83 cpu (
        .clk(cpu_clk),
        .rst(rst)
    );

    ppu_vdp ppu (
        .clk(clk)
    );

endmodule

module ram_8k(
    input wire clk,
    input wire [12:0] addr,
    input wire [7:0] d_in,
    input wire write,
    output reg [7:0] d_out
);
endmodule

module clkdiv4(
    input wire clk,
    output reg div
);
    logic [1:0] ctr;
    initial ctr = 2'b0;
    always_ff @(posedge clk) ctr <= ctr + 1'b1;
    assign div = ~|ctr;
endmodule