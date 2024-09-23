`default_nettype none

typedef enum logic [1:0] {
    PHASE_HBLANK,
    PHASE_VBLANK,
    PHASE_OAM_SCAN,
    PHASE_DRAW
} ppu_phase_t;

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
    // Display:
    output wire lcd_hsync, lcd_vsync, lcd_pixel,
    output wire [1:0] lcd_color,
    // IRQ:
    output logic irq_vblank
);
    reg mem_clk;
    assign mem_clk = ~clk;
    
    // Regs:
    lcdc_t lcdc;
    reg [7:0] scy, scx;
    always_comb case (reg_addr)
        LCDC: reg_d_rd = lcdc;
        SCY: reg_d_rd = scy;
        SCX: reg_d_rd = scx;
        LY: reg_d_rd = renderer_ly;
        default: reg_d_rd = 'x;
    endcase
    always_ff @(posedge clk) if (reg_write) case (ppu_reg_t'(reg_addr))
        LCDC: lcdc = reg_d_wr;
        SCY: scy = reg_d_wr;
        SCX: scx = reg_d_wr;
    endcase

    ppu_phase_t phase, phase_prev;
    wire [12:0] renderer_vram_addr;
    wire [7:0] renderer_ly;
    wire renderer_pixel_valid;
    wire [1:0] renderer_pixel;
    ppu_renderer renderer (
        .clk(clk),
        .vram_addr(renderer_vram_addr),
        .vram_in(vram_d_rd),
        .lcdc(lcdc),
        .scy(scy),
        .scx(scx),
        .ly(renderer_ly),
        .phase(phase),
        .pixel_valid(renderer_pixel_valid),
        .pixel(renderer_pixel)
    );

    // VRAM:
    reg [12:0] vram_addr;
    reg vram_write;
/*
    ram_8192words_8bit vram (
        .clk(mem_clk),
        .addr(vram_addr),
        .write(vram_write),
        .d_in(vram_d_wr),
        .d_out(vram_d_rd)
    );
*/
    sp_8192w_8b your_instance_name(
        .dout(vram_d_rd),
        .clk(~clk),
        .oce(1'b0),
        .ce(1'b1),
        .reset(~lcdc.ena),
        .wre(vram_write),
        .ad(vram_addr),
        .din(vram_d_wr)
    );

    always_ff @(posedge clk) phase_prev <= phase;
    assign irq_vblank = phase_prev != phase && phase == PHASE_VBLANK;

    always_comb case (phase)
        PHASE_DRAW: begin vram_addr = renderer_vram_addr; vram_write = 0; end
        default: begin vram_addr = vram_addr_in; vram_write = vram_write_in; end
    endcase

    assign lcd_hsync = phase == PHASE_HBLANK;
    assign lcd_vsync = phase == PHASE_VBLANK;
    assign lcd_pixel = renderer_pixel_valid;
    assign lcd_color = renderer_pixel;
endmodule

module ppu_renderer(
    input wire clk,
    // Bus -> VRAM, OAM:
    output wire [12:0] vram_addr,
    input wire [7:0] vram_in,
    // Regs:
    input lcdc_t lcdc,
    input byte scy, scx,
    output byte ly,
    // Display:
    output ppu_phase_t phase,
    output reg pixel_valid,
    output reg [1:0] pixel
);
    reg fifo_pop;
    assign pixel_valid = fifo_pop;

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
                default: begin
                    ly <= ly + 1'b1;
                    if (phase != PHASE_VBLANK) phase <= PHASE_OAM_SCAN;
                end
            endcase
        end else begin
            // Within scanline:
            if (phase != PHASE_VBLANK) begin
                // Handle dot tick within scan line:
                if (dot_ctr == 9'd79) phase <= PHASE_DRAW;
                if (lx == 8'd159) phase <= PHASE_HBLANK;
                if (pixel_valid) lx <= lx + 1'b1;
            end
            dot_ctr <= dot_ctr + 1'b1;
        end

    reg transfer_pixels;
    reg fetcher_full;
    wire [7:0] pix_hi, pix_lo;
    reg fetcher_rst;
    pixel_fetcher fetcher (
        .clk(clk),
        .vram_addr(vram_addr),
        .vram_in(vram_in),
        .ly(ly),
        .lx(lx),
        .lcdc(lcdc),
        .scx(scx),
        .scy(scy),
        .rst(fetcher_rst),
        .pix_hi(pix_hi),
        .pix_lo(pix_lo),
        .full(fetcher_full)
    );

    reg fifo_rst;
    reg bg_fifo_empty;
    reg [1:0] fifo_pixel;
    bg_shr shr1 (
        .clk(clk),
        .rst(fifo_rst),
        .load(transfer_pixels),
        .pop(fifo_pop),
        .load_hi(pix_hi),
        .load_lo(pix_lo),
        .color(fifo_pixel),
        .empty(bg_fifo_empty)
    );
    assign pixel = fifo_pixel;
    assign fifo_rst = phase == PHASE_OAM_SCAN;
    assign fetcher_rst = transfer_pixels || (phase != PHASE_DRAW);
    assign fifo_pop = phase == PHASE_DRAW && ~bg_fifo_empty;
    assign transfer_pixels = fetcher_full && bg_fifo_empty;
endmodule

module bg_shr (
    input wire clk,
    input wire rst,
    input wire pop,
    input wire [7:0] load_hi, load_lo,
    input wire load,
    output reg [1:0] color,
    output reg empty
);
    reg [7:0] filled, buf_hi, buf_lo;
    always_ff @(posedge clk)
        if (rst) filled <= 0;
        else if (load) begin
            filled <= 'hff;
            buf_hi <= load_hi; buf_lo <= load_lo;
        end else if (pop) begin
            filled <= filled >> 1;
            buf_hi <= buf_hi << 1;
            buf_lo <= buf_lo << 1;
        end

    assign color = {buf_hi[7], buf_lo[7]};
    assign empty = ~|filled;
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
    output reg [7:0] pix_hi, pix_lo,
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
    assign pix_hi = data_high, pix_lo = data_low;

    fetcher_state_t state;
    always_comb case (state)
        FETCH_TILE: vram_addr = tile_addr;
        FETCH_DATA_LOW: vram_addr = {data_addr, 1'b0};
        FETCH_DATA_HIGH: vram_addr = {data_addr, 1'b1};
        default: vram_addr = 'x;
    endcase

    always_ff @(posedge clk)
        if (rst) state <= FETCH_TILE;
        else case (state)
            FETCH_TILE: begin state <= FETCH_DATA_LOW; tile_id <= vram_in; end
            FETCH_DATA_LOW: begin state <= FETCH_DATA_HIGH; data_low <= vram_in; end
            FETCH_DATA_HIGH: begin state <= FETCH_PUSH; data_high <= vram_in; end
        endcase
endmodule

`default_nettype wire