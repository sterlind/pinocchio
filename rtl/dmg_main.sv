typedef enum logic [15:0] {
    ADDR_ROM     = 16'b0xxx_xxxx_xxxx_xxxx,
    VRAM    = 16'b100x_xxxx_xxxx_xxxx,
    WRAM    = 16'b110x_xxxx_xxxx_xxxx,
    HRAM    = 16'b1111_1111_1xxx_xxxx,
    PPU_REG = 16'hff4x
} mem_mask;

module dmg_main(
    input wire rst,
    input wire clk,
    output reg [14:0] rom_addr,
    input wire [7:0] rom_data,
    output wire lcd_vsync, lcd_hsync, lcd_pixel,
    output wire [1:0] lcd_color
);
    /*
    wire clk_cpu;
    gclkdiv4 clk_cpu_div (
        .hclkin(clk),
        .clkout(clk_cpu),
        .resetn(1'b1)
    );
    */

    reg [1:0] mcycle_ctr;
    reg cpu_ce;
    always_ff @(posedge clk) mcycle_ctr <= mcycle_ctr + 1'b1;
    assign cpu_ce = &mcycle_ctr;

    reg [7:0] bus_in, cpu_d_out;
    wire [15:0] cpu_addr /* synthesis syn_keep=1 */;
    wire cpu_write;
    sm83 cpu (
        .clk(clk),
        .ce(cpu_ce),
        .rst(rst),
        .d_in(bus_in),
        .addr(cpu_addr),
        .d_out(cpu_d_out),
        .write(cpu_write)
    );

    reg ppu_reg_write, vram_write;
    wire [7:0] reg_d_rd, vram_d_rd;
    ppu_m ppu (
        .clk(clk),
        .reg_addr(cpu_addr[3:0]),
        .reg_write(ppu_reg_write),
        .reg_d_wr(cpu_d_out),
        .reg_d_rd(reg_d_rd),
        .vram_addr_in(cpu_addr[12:0]),
        .vram_write_in(vram_write),
        .vram_d_wr(cpu_d_out),
        .vram_d_rd(vram_d_rd),
        .lcd_hsync(lcd_hsync),
        .lcd_vsync(lcd_vsync),
        .lcd_pixel(lcd_pixel),
        .lcd_color(lcd_color)
    );

    reg hram_write;
    reg [7:0] hram_in;
    wire [7:0] hram_d_out;
    ram_128words_8bit hram (
        .clk(~clk),
        .ce(cpu_ce),
        .addr(cpu_addr[6:0]),
        .d_in(cpu_d_out),
        .d_out(hram_d_out),
        .write(hram_write)
    );

    assign rom_addr = cpu_addr[14:0];
    always_comb begin
        hram_write = 0; ppu_reg_write = 0; vram_write = 0; ppu_reg_write = 0;
        bus_in = 'x;
        casex (cpu_addr)
            VRAM: begin bus_in = vram_d_rd; vram_write = cpu_write; end
            HRAM: begin bus_in = hram_d_out; hram_write = cpu_write; end
            ADDR_ROM: begin bus_in = rom_data; end
            PPU_REG: begin bus_in = reg_d_rd; ppu_reg_write = cpu_write; end
        endcase
    end
endmodule

module ram_128words_8bit (
    input wire clk,
    input wire ce,
    input wire [6:0] addr,
    input wire write,
    input wire [7:0] d_in,
    output reg [7:0] d_out
);
    reg [7:0] mem[0:127];
    always_ff @(posedge clk) if (ce) begin
        d_out <= mem[addr];
        if (write) mem[addr] <= d_in;
    end
endmodule