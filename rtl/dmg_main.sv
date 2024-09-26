typedef enum logic [15:0] {
    ADDR_ROM    = 16'b0xxx_xxxx_xxxx_xxxx,
    VRAM        = 16'b100x_xxxx_xxxx_xxxx,
    WRAM        = 16'b110x_xxxx_xxxx_xxxx,
    OAM         = 16'hfexx,
    HIDEROM     = 16'hff50,
    OAM_DMA     = 16'hff46,
    PPU_REG     = 16'hff4x
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
    wire [15:0] cpu_addr, bus_addr , dma_src_addr;
    reg bus_write, cpu_write;
    reg dma_active;
    assign bus_addr = dma_active ? dma_src_addr : cpu_addr;
    assign bus_write = dma_active ? 0 : cpu_write;
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

    reg ppu_reg_write, vram_write, oam_write;
    wire [7:0] reg_d_rd, vram_d_rd, oam_d_rd;
    ppu_m ppu (
        .clk(clk),
        .d_wr(cpu_d_out),
        .reg_addr(bus_addr[3:0]),
        .reg_d_rd(reg_d_rd),
        .reg_write(ppu_reg_write),
        .oam_addr_in(bus_addr[7:0]),
        .oam_d_rd(oam_d_rd),
        .oam_write(oam_write),
        .vram_addr_in(bus_addr[12:0]),
        .vram_d_rd(vram_d_rd),
        .vram_write_in(vram_write),
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
        .ad(bus_addr[12:0]),
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

    reg [7:0] dma_base;
    reg [7:0] dma_idx;
    assign dma_active = dma_idx != 8'h9f;
    assign dma_src_addr = {dma_base, dma_idx};

    always_ff @(negedge clk)
        if (~rst) begin hide_boot <= 0; dma_base <= 8'hff; end
        else begin
            if (bus_write) case (bus_addr)
                HIDEROM: hide_boot <= hide_boot | (|cpu_d_out);
                OAM_DMA: begin dma_base <= cpu_d_out; dma_idx <= 8'b0; end
            endcase
            if (dma_active) dma_idx <= dma_idx + 1'b1;
        end

    assign rom_addr = bus_addr[14:0];
    always_comb begin
        ppu_reg_write = 0; vram_write = 0; ppu_reg_write = 0; wram_write = 0; oam_write = 0;
        bus_in = 8'hff;
        casex (bus_addr)
            16'hff00: bus_in = 8'h0f;
            VRAM: begin bus_in = vram_d_rd; vram_write = bus_write; end
            WRAM: begin bus_in = wram_d_rd; wram_write = bus_write; end
            OAM: begin bus_in = oam_d_rd; oam_write = bus_write; end
            ADDR_ROM: begin bus_in = (hide_boot || |rom_addr[14:8]) ? rom_data : boot_data; end
            OAM_DMA: bus_in = dma_base;
            PPU_REG: begin bus_in = reg_d_rd; ppu_reg_write = bus_write; end
        endcase
        if (dma_active) oam_write = 1;
    end
endmodule