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
    LY      = 4'h4,
    BGP     = 4'h7
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
    input wire [7:0] d_wr,
    // Regs:
    input wire [3:0] reg_addr,
    output logic [7:0] reg_d_rd,
    input wire reg_write,
    // OAM:
    input wire [7:0] oam_addr_in,
    output logic [7:0] oam_d_rd,
    input wire oam_write,
    // VRAM:
    input wire [12:0] vram_addr_in,
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
    reg [7:0] scy, scx, bgp;
    always_comb case (reg_addr)
        LCDC: reg_d_rd = lcdc;
        SCY: reg_d_rd = scy;
        SCX: reg_d_rd = scx;
        LY: reg_d_rd = renderer_ly;
        BGP: reg_d_rd = bgp;
        default: reg_d_rd = 'x;
    endcase
    always_ff @(posedge clk) if (reg_write) case (ppu_reg_t'(reg_addr))
        LCDC: lcdc = d_wr;
        SCY: scy = d_wr;
        SCX: scx = d_wr;
        BGP: bgp = d_wr;
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
        .bgp(bgp),
        .phase(phase),
        .pixel_valid(renderer_pixel_valid),
        .pixel(renderer_pixel)
    );

    // VRAM:
    reg [12:0] vram_addr;
    reg vram_write;

    sp_8192w_8b vram_block (
        .dout(vram_d_rd),
        .clk(~clk),
        .oce(1'b0),
        .ce(1'b1),
        .reset(~lcdc.ena),
        .wre(vram_write),
        .ad(vram_addr),
        .din(d_wr)
    );

    // OAM:
    reg [7:0] oam_write_addr;
    reg [6:0] oam_read_addr;
    reg [15:0] oam_db;
    assign oam_d_rd = oam_addr_in[0] ? oam_db[15:8] : oam_db[7:0];
    oam_sdpb oam_block (
        .clka(~clk),
        .cea(oam_write),
        .reseta(~lcdc.ena),
        .clkb(~clk),
        .ceb(1'b1),
        .resetb(~lcdc.ena),
        .oce(1'b0),
        .ada(oam_addr_in),
        .din(d_wr),
        .adb(oam_addr_in[7:1]),
        .dout(oam_db)
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
    output wire [6:0] oam_addr,
    input wire [15:0] oam_in,
    // Regs:
    input lcdc_t lcdc,
    input byte scy, scx,
    input byte bgp,
    output byte ly,
    // Display:
    output ppu_phase_t phase,
    output reg pixel_valid,
    output reg [1:0] pixel
);
    reg fifo_pop;
    reg [8:0] dot_ctr;
    reg [7:0] lx;
    assign pixel_valid = fifo_pop;

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
        .next_block(|lx | pixel_valid),
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
        .palette(bgp),
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
    input wire [7:0] palette,
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

    always_comb case ({buf_hi[7], buf_lo[7]})
        2'b00: color = palette[1:0];
        2'b01: color = palette[3:2];
        2'b10: color = palette[5:4];
        2'b11: color = palette[7:6];
    endcase
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
    input wire next_block,
    output reg [12:0] vram_addr,
    input wire [7:0] vram_in,
    output reg [7:0] pix_hi, pix_lo,
    output reg full
);
    reg [7:0] px, py;
    assign py = ly + scy, px = lx + scx + (next_block ? 8'd8 : 8'd0);

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

typedef enum logic [1:0] {
    SPRITE_ATTRS,
    SPRITE_TILE_ID,
    SPRITE_Y
} s_sprite_data_t;

module sprite_chain(
    input wire clk, rst, load, hi,
    input wire [15:0] d_load,
    input wire [7:0] lx, ly,
    input wire s_sprite_data_t s_data,
    output logic [7:0] d_out,
    output logic d_valid
);
    reg valid, in_range;
    reg [7:0] dy;
    assign dy = ly - d_load[7:0];

    always_ff @(posedge clk)
        if (~rst) valid <= 0;
        else if (load) begin
            if (hi) valid <= 0; // Reset on hi.
            else valid <= ~|dy[7:3]; // Not negative and within 8 pixels. Todo: handle tall sprites.
        end

    reg [15:0] d_in;
    assign d_in = hi ? d_load : {d_load[15:8], dy};
    wire [3:0] bus_pri [0:9];
    wire [7:0] bus_data [0:9];
    wire bus_valid [0:9];

    sprite_slot slot[0:9] (
        .clk(clk), .rst(rst),
        .load(load & valid), .hi(hi),
        .s_data(s_data),
        .d_load(d_in),
        .lx(lx),
        .pri_in('{4'bx, bus_pri}),
        .pri_out('{bus_pri, 4'bx}),
        .d_in({8'bx, bus_data}),
        .d_out({8'bx, d_out}),
        .valid_in('{1'b0, bus_valid}),
        .valid_out('{bus_valid, d_valid})
    );
endmodule

module sprite_slot(
    input wire clk, rst,
    input wire load, hi,
    input wire s_sprite_data_t s_data,
    input wire [15:0] d_load,
    input wire [7:0] lx,
    // Sprite bus:
    input wire [3:0] pri_in,    // [Pri]:   Priority of current candidate. 
    output logic [3:0] pri_out, //          Composed of sprite's x coord and priority bit.
    input wire [7:0] d_in,      // [Data]:  Sprite data (tile idx, y offset, attrs.)
    output logic [7:0] d_out,
    input wire valid_in,        // [Valid]: During draw, signals that the bus contains some eligible sprite.
    output logic valid_out      //          During load, signals that a previous slot took the data.
);
    reg filled, eligible, chosen;
    reg [7:0] dy, x, y, tile_idx, attrs, dx;
    reg [3:0] pri;

    assign dx = lx - x;
    assign eligible = ~|dx[7:3]; // Not negative and within 8 pixels.
    assign pri = {attrs[7], dx[2:0]};
    assign chosen = eligible && (~valid_in || pri < pri_in);

    assign valid_out = load ? ~filled : (valid_in || chosen);
    assign pri_out = chosen ? pri : pri_in;

    always_comb
        if (chosen)
            case (s_data)
                SPRITE_ATTRS: d_out = attrs;
                SPRITE_TILE_ID: d_out = tile_idx;
                SPRITE_Y: d_out = y;
            endcase
        else d_out = d_in;

    always_ff @(posedge clk)
        if (~rst) filled <= 0;
        else if (load && ~filled && ~valid_in) begin
            if (hi) {attrs, tile_idx} <= d_load;
            else {x, y} <= d_load;
        end
endmodule

`default_nettype wire