`include "ram.sv"

typedef enum logic [3:0] {
    LCDC = 4'h0,
    LY = 4'h4
} ppu_reg_t;

typedef enum logic [1:0] {
    PHASE_HBLANK,
    PHASE_VBLANK,
    PHASE_OAM_SCAN,
    PHASE_DRAW
} ppu_phase_t;

typedef struct packed {
    bit ena;
    bit win_tile_map;
    bit win_ena;
    bit bg_win_tile_data;
    bit bg_tile_map;
    bit obj_size;
    bit obj_ena;
    bit bg_ena;
} lcdc_t;

module ppu_m (
    input wire clk,
    // DMA:
    // When DMA is active, PPU will request data in (and take over the CPU's bus.)
    output wire [15:0] dma_src_addr,
    input wire [7:0] dma_d_in,
    input wire dma_active,
    // Regs:
    input wire [3:0] reg_addr,
    input wire [7:0] reg_d_wr,
    output logic [7:0] reg_d_rd,
    input wire reg_write,
    // VRAM:
    input wire [12:0] vram_addr_in,
    input wire [7:0] vram_d_wr,
    output logic [7:0] vram_d_rd,
    input wire vram_write_in,
    // OAM:
    input wire [7:0] oam_addr_in,
    input wire [7:0] oam_d_wr,
    output logic [7:0] oam_d_rd,
    input wire oam_write_in,
    // Display:
    output wire lcd_clk, lcd_ena,
    output wire lcd_hsync, lcd_vsync,
    output wire [1:0] lcd_color
);
    ppu_phase_t phase;

    reg [12:0] vram_addr;
    reg vram_write;
    ram #(WORDS = 8192) vram (
        .clk(clk),
        .addr(vram_addr),
        .write(vram_write),
        .d_in(vram_d_wr),
        .d_out(vram_d_rd)
    );
    always_comb case (phase)
        PHASE_DRAW: begin vram_addr = renderer.vram_addr; vram_write = 0; end
        default: begin vram_addr = vram_addr_in; vram_write = vram_write_in; end
    endcase

    reg [6:0] oam_addr;
    reg [5:0] oam_dma_src_base;
    ram #(WORDS = 80, WIDTH = 16) oam (
        .clk(clk),
        .addr(oam_addr),
        .write(oam_write)
    );
    always_comb 

    ppu_renderer renderer (
        .clk(clk),
        .vram_in(vram.d_out)
    );
endmodule

module ppu_renderer(
    input wire clk,
    output wire [12:0] vram_addr,
    input wire [7:0] vram_in,
    input wire [5:0]
);
endmodule

module ppu_ctrl(
    // Bus <-> PPU Regs:
    input wire clk,
    input wire [3:0] reg_addr,
    input wire [7:0] reg_in,
    output logic [7:0] reg_out,
    input wire reg_write,
    // Control signals:
    output lcdc_t lcdc,
    input byte ly
);
    always_ff @(posedge clk)
        if (reg_write)
            case (reg_addr)
                LCDC: lcdc <= reg_in;
            endcase

    always_comb begin
        reg_out = 'x;
        case (reg_addr)
            LY: reg_out = ly;
        endcase
    end
endmodule

module ppu_engine(
    input wire clk,
    input lcdc_t lcdc,
    output byte ly
);
    reg [8:0] lx;

    // Fixme: placeholder.
    always_ff @(posedge clk) begin
        if (~lcdc.ena) begin
            lx <= 0;
            ly <= 0;
        end else begin
            lx <= lx + 1'b1;
            if (lx == 9'd455) begin
                lx <= 0;
                if (ly == 8'd153) ly <= 0;
                else ly <= ly + 1'b1;
            end
        end
    end
endmodule