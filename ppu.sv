`include "ram.sv"

typedef enum logic [3:0] {
    LCDC    = 4'h0,
    STAT    = 4'h1,
    SCY     = 4'h2,
    SCX     = 4'h3,
    LY      = 4'h4
} ppu_reg_t;

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
    output wire lcd_hsync, lcd_vsync, lcd_pixel,
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
        .oam_in(oam.d_out),
        .lcdc(lcdc),
        .scy(scy),
        .scx(scx)
    );

    assign lcd_hsync = renderer.phase == PHASE_HBLANK;
    assign lcd_vsync = renderer.phase == PHASE_VBLANK;
    assign lcd_pixel = renderer.pixel_valid;
    assign lcd_color = renderer.pixel;
endmodule

typedef enum logic [1:0] {
    PHASE_HBLANK,
    PHASE_VBLANK,
    PHASE_OAM_SCAN,
    PHASE_DRAW
} ppu_phase_t;

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
    output reg phase,
    output reg pixel_valid,
    output reg [1:0] pixel
);
    assign pixel_valid = fifo_pull;
    assign pixel = bg_fifo.color;

    reg [8:0] dot_ctr;
    reg [7:0] lx;
    always_ff @(posedge clk)
        if (~lcdc.ena) begin
            // On disable:
            dot_ctr <= 0;
            ly <= 0;
            lx <= 0;
            phase <= PHASE_OAM_SCAN;
        end else if (dot_ctr == 9'd455) begin
            // On end of scanline:
            dot_ctr <= 0;
            lx <= 0;
            case (ly)
                8'd143: begin phase <= PHASE_VBLANK; ly <= 8'd144; end
                8'd153: begin phase <= PHASE_OAM_SCAN; ly <= 0; end
                default: begin phase <= PHASE_OAM_SCAN; ly <= ly + 1'b1; end
            endcase
        end else begin
            // Within scanline:
            if (phase != PHASE_VBLANK) begin
                // Handle dot tick within scan line:
                if (dot_ctr == 9'd79) phase <= PHASE_DRAW;
                if (lx == 8'd159) phase <= PHASE_HBLANK;
            end
            dot_ctr <= dot_ctr + 1;
        end

    reg fetch_clk;
    always_ff @(posedge clk)
        if (~lcdc.ena) fetch_clk <= 0;
        else fetch_clk <= ~fetch_clk;

    reg transfer_pixels;
    assign transfer_pixels = fetcher.full && bg_fifo.empty;
    pixel_fetcher fetcher (
        .clk(fetch_clk),
        .vram_addr(vram_addr),
        .vram_in(vram_in),
        .ly(ly),
        .lx(lx),
        .lcdc(lcdc),
        .scx(scx),
        .scy(scy),
        .rst(transfer_pixels)
    );

    reg fifo_rst;
    reg fifo_pull;
    assign fifo_pull = phase == PHASE_DRAW && ~bg_fifo.empty;
    bg_fifo_m bg_fifo (
        .clk(clk),
        .rst(fifo_rst),
        .load(transfer_pixels),
        .pull(fifo_pull)
    );
endmodule

module bg_fifo_m (
    input wire clk,
    input wire rst,
    input wire pull,
    input wire [1:0] pixels_in [7:0],
    input wire load,
    output reg [1:0] color,
    output reg empty
);
    reg [7:0] filled;
    reg [1:0] buffer[7:0];
    genvar k;
    generate
        for (k = 0; k < 7; k = k + 1)
            always @(posedge clk)
                if (load) buffer[k] <= pixels_in[k];
                else if (pull) buffer[k] <= buffer[k + 1];
    endgenerate
    always @(posedge clk) if (load) buffer[7] <= pixels_in[7]; else if (pull) buffer[7] <= 0;

    always @(posedge clk)
        if (rst) filled <= 0;
        else if (pull) filled <= {1'b0, filled[6:0]};
    
    assign color = buffer[filled], empty = ~filled[0];
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