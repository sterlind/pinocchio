`include "ram.sv"

typedef enum logic [3:0] {
    LCDC    = 4'h0,
    STAT    = 4'h1,
    SCY     = 4'h2,
    SCX     = 4'h3,
    LY      = 4'h4
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
    input ppu_reg_t reg_addr,
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
    // VRAM:
    reg [12:0] vram_addr;
    reg vram_write;
    ram #(.WORDS(8192)) vram (
        .clk(clk),
        .addr(vram_addr),
        .write(vram_write),
        .d_in(vram_d_wr),
        .d_out(vram_d_rd)
    );
    always_comb case (renderer.phase)
        PHASE_DRAW: begin vram_addr = renderer.vram_addr; vram_write = 0; end
        default: begin vram_addr = vram_addr_in; vram_write = vram_write_in; end
    endcase

    // OAM:
    reg [6:0] oam_addr;
    reg [5:0] oam_dma_src_base;
    reg oam_write;
    ram #(.WORDS(80), .WIDTH(16)) oam (
        .clk(clk),
        .addr(oam_addr),
        .write(oam_write)
    );
    always_comb case (renderer.phase)
        PHASE_HBLANK, PHASE_VBLANK: begin oam_addr = oam_addr_in; oam_write = oam_write_in; end
        default: begin oam_write = 0; oam_addr = renderer.oam_addr; end
    endcase

    // Regs:
    lcdc_t lcdc;
    reg [7:0] scy, scx;
    always_comb case (reg_addr)
        LCDC: reg_d_rd = lcdc;
        SCY: reg_d_rd = scy;
        SCX: reg_d_rd = scx;
        LY: reg_d_rd = renderer.ly;
        default: reg_d_rd = 'x;
    endcase
    always_ff @(posedge clk) if (reg_write) case (reg_addr)
        LCDC: lcdc = reg_d_wr;
        SCY: scy = reg_d_wr;
        SCX: scx = reg_d_wr;
    endcase

    ppu_renderer renderer (
        .clk(clk),
        .vram_in(vram.d_out),
        .oam_in(oam.d_out)
    );
endmodule

module ppu_renderer(
    input wire clk,
    // Bus -> VRAM, OAM:
    output wire [12:0] vram_addr,
    input wire [7:0] vram_in,
    input wire [6:0] oam_addr,
    input wire [15:0] oam_in,
    // Regs:
    input lcdc_t lcdc,
    input byte scy, scx,
    output byte ly,
    // Display:
    output reg vblank,
    output reg draw
);
    reg [8:0] dot_ctr;
    reg fetch_clk;
    always_ff @(posedge clk)
        if (~lcdc.ena) begin
            draw <= 0;
            dot_ctr <= 0;
            lx <= 0;
            ly <= 0;
            vblank <= 0;
        end else if (dot_ctr == 9'd455) begin
            dot_ctr <= 0;
            lx <= 0;
            draw <= 0;
            case (ly)
                8'd143: begin vblank <= 1; ly <= ly + 1'b1; end
                8'd153: begin vblank <= 0; ly <= 0; end
                default: ly <= ly + 1'b1;
            endcase
        end else begin
            dot_ctr <= dot_ctr + 1'b1;
            if (dot_ctr == 9'd79) draw <= 1;
        end

    pixel_fetcher fetcher (
        .clk(fetch_clk),

    )
endmodule

typedef enum logic [2:0] {
    FETCH_TILE,
    FETCH_DATA_LOW,
    FETCH_DATA_HIGH,
    FETCH_PUSH
} fetcher_state_t;

// Addresses:
// Tile maps are at 9800-9BFF and 9C00-9FFF.
// Tile data is in three blocks:
// 0: 8000-87FF
// 1: 8800-8FFF
// 2: 9000-97FF
// Addresses are x_xxxx_xxxx_xxxx (8000-9FFF).
// Blocks:
// 0: 0_0000_0000_0000 - 0_0111_1111_1111
// 1: 0_1000_0000_0000 - 0_1111_1111_1111
// 2: 1_0000_0000_0000 - 1_0111_1111_1111
// Tile maps:
// A: 1_1000_0000_0000 - 1_1011_1111_1111
// B: 1_1100_0000_0000 - 1_1111_1111_1111
// So 1_1xaa_aaab_bbbb where x = bit, a = tx, b = ty.
// NOTE: Clock this at half speed.
module pixel_fetcher (
    input wire clk,
    input wire rst,
    input wire [7:0] ly, lx,
    input lcdc_t lcdc,
    input byte scx, scy,
    output reg [12:0] vram_addr,
    input wire [7:0] vram_in,
    output reg [1:0] pixels [7:0],
    output reg full
);
    reg [7:0] px, py;
    assign py = ly + scy, px = lx + scx;

    reg [7:0] tile_id, data_low, data_high;

    reg [12:0] tile_addr;
    assign tile_addr = {2'b11, lcdc.bg_tile_map, py[7:3], px[7:3]};

    reg [11:0] data_addr;
    assign data_addr = {2'b00, tile_id, py[2:0]}; // Todo: fix to use lcdc.4

    assign full = state == FETCH_PUSH;
    genvar k;
    generate
        for (k = 0; k < 8; k = k + 1)
            assign pixels[k] = {data_high[k], data_low[k]};
    endgenerate

    fetcher_state_t state;
    always_comb case (state)
        FETCH_TILE: vram_addr = tile_addr;
        FETCH_DATA_LOW: vram_addr = {data_addr, 1'b0};
        FETCH_DATA_HIGH: vram_addr = {data_addr, 1'b1};
    endcase

    always_ff @(posedge clk)
        if (rst) state <= FETCH_TILE;
        else case (state)
            FETCH_TILE: begin state <= FETCH_DATA_LOW; tile_id <= vram_in; end
            FETCH_DATA_LOW: begin state <= FETCH_DATA_HIGH; data_low <= vram_in; end
            FETCH_DATA_HIGH: begin state <= FETCH_PUSH; data_high <= vram_in; end
        endcase
endmodule