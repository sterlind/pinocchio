typedef enum logic [15:0] {
    ADDR_ROM     = 16'b0xxx_xxxx_xxxx_xxxx,
    VRAM    = 16'b100x_xxxx_xxxx_xxxx,
    WRAM    = 16'b110x_xxxx_xxxx_xxxx,
    HIDEROM = 16'hff50,
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
    always_ff @(posedge clk)
        if (~rst) mcycle_ctr <= 2'b0;
        else mcycle_ctr <= mcycle_ctr + 1'b1;
    assign cpu_ce = &mcycle_ctr;

    wire irq_vblank;
    reg [7:0] irq;
    assign irq = {7'b0, irq_vblank};

    reg [7:0] bus_in, cpu_d_out;
    wire [15:0] cpu_addr /* synthesis syn_keep=1 */;
    wire cpu_write;
    sm83 cpu (
        .clk(clk),
        .ce(cpu_ce),
        .rst(rst),
        .d_in_ext(bus_in),
        .addr(cpu_addr),
        .d_out(cpu_d_out),
        .write(cpu_write),
        .irq_in(irq)
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
        .lcd_color(lcd_color),
        .irq_vblank(irq_vblank)
    );

    wire [7:0] wram_d_rd;
    reg wram_write;
    work_spram wram(
        .dout(wram_d_rd),
        .clk(~clk),
        .oce(1'b0),
        .ce(cpu_ce),
        .reset(~rst),
        .wre(wram_write),
        .ad(cpu_addr[12:0]),
        .din(cpu_d_out)
    );

    reg hide_boot;
    wire [7:0] boot_data;
    boot_prom boot (
        .clk(~clk),
        .oce('x),
        .ce(1'b1),
        .reset(1'b0),
        .ad(rom_addr[7:0]),
        .dout(boot_data)
    );

    always_ff @(negedge clk)
        if (~rst) hide_boot <= 0;
        else if (cpu_addr == HIDEROM && cpu_write && |cpu_d_out) hide_boot <= 1'b1;

    assign rom_addr = cpu_addr[14:0];
    always_comb begin
        ppu_reg_write = 0; vram_write = 0; ppu_reg_write = 0; wram_write = 0;
        bus_in = 8'hff;
        casex (cpu_addr)
            VRAM: begin bus_in = vram_d_rd; vram_write = cpu_write; end
            WRAM: begin bus_in = wram_d_rd; wram_write = cpu_write; end
            ADDR_ROM: begin bus_in = (hide_boot || |rom_addr[14:8]) ? rom_data : boot_data; end
            PPU_REG: begin bus_in = reg_d_rd; ppu_reg_write = cpu_write; end
        endcase
    end
endmodule