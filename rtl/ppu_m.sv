`default_nettype none

typedef struct packed {
    bit [2:0] dy;    // Y offset within the sprite block (for current ly.)
    bit [7:0] tile;  // Tile index.
    bit [3:0] attrs; // Sprite attributes.
} sprite_data_t;

typedef struct packed {
    bit [7:0] x;
    sprite_data_t data;
    bit bypass;
} sprite_bus_t;

typedef struct packed {
    byte y;
    byte x;
    byte tile;
    byte attrs;
} oam_entry_t;

module sprite_slot(
    input wire clk, rst, load,
    input sprite_bus_t prev_in,
    output sprite_bus_t next_out
);
    reg ready;              // If slot is filled but hasn't been read yet.
    reg [7:0] x;            // X position of the sprite to match against during queries.
    sprite_data_t data;     // Data to return, once it's our turn.

    // We're chosen for a query if we're the first valid slot with a matching x in the chain.
    // If chosen, we push our data the bus and set bypass.
    reg chosen_query, chosen_load;
    assign chosen_load = load & ~ready & ~prev_in.bypass;
    assign chosen_query = ~load & ready & ~prev_in.bypass & (x == prev_in.x);
    assign next_out = (chosen_load | chosen_query) ? {8'bx, data, 1'b1} : prev_in;

    always_ff @(posedge clk)
        // Reset is simple: just mark slot as empty.
        if (~rst) ready <= 0;
        // It's our turn to load if we're not yet ready but all previous slots were.
        else if (chosen_load) begin
            ready <= 1;
            x <= prev_in.x;
            data <= prev_in.data;
        end
        // After being chosen in query mode, mark slot as empty to give others a turn.
        else if (chosen_query) ready <= 0;

endmodule

module sprite_chain(
    input wire clk, rst, load, query,
    input wire [7:0] ly, lx,
    output logic [6:0] oam_addr,
    input wire [15:0] oam_d_in,
    input wire cfg_tall_sprites,
    output sprite_data_t d_out,
    output logic d_valid
);
    sprite_bus_t bus_in, bus_out;
    sprite_bus_t [0:8] bus;
    sprite_slot slots [0:9] (
        .clk(clk), .rst(rst), .load(load),
        .prev_in({bus_in, bus}),
        .next_out({bus, bus_out})
    );

    assign d_out = bus_out.data, d_valid = ~load && query && bus_out.bypass;

    // Buffer the first word of OAM, so we can assemble the whole entry before storing it in a slot.
    reg [15:0] oam_buf; 
    oam_entry_t entry;
    assign entry = {oam_buf, oam_d_in};

    // Calculate y offset within sprite, and check if it's visible on this scan line.
    reg [7:0] dy, tile;
    //reg [3:0] dy_corr;
    assign dy = ly - (entry.y - 8'd16);
    //assign dy_corr = entry.attrs[6] ? ~dy[3:0] : dy[3:0]; 
    reg visible;
    assign visible = cfg_tall_sprites ? ~|dy[7:4] : ~|dy[7:3];
    assign tile = cfg_tall_sprites ? {entry.tile[7:1], dy[3]} : entry.tile;

    assign bus_in = load
        // During load, fill slots with OAM data. Bypass all slots if sprite is out of range or not yet buffered.
        ? {entry.x - 8'd8, {dy[2:0], tile, entry.attrs[7:4]}, ~(visible & oam_addr[0])}
        // Otherwise, send a query through.
        : {lx, sprite_data_t'('0), ~query};

    always_ff @(posedge clk)
        if (~rst) oam_addr <= 0;
        else if (load) begin
            oam_addr <= oam_addr + 1'b1;
            if (~oam_addr[0]) oam_buf <= oam_d_in;
        end
endmodule

typedef enum logic [1:0] {
    S_FETCH_TILE,
    S_FETCH_DATA_LO,
    S_FETCH_DATA_HI,
    S_PUSH_FIFO
} renderer_state_t;

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

typedef struct packed {
    bit lyc_int_sel;
    bit mode2_int_sel;
    bit mode1_int_sel;
    bit mode0_int_sel;
} int_sels_t;

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
    LYC     = 4'h5,
    BGP     = 4'h7,
    WY      = 4'ha,
    WX      = 4'hb
} ppu_reg_t;

module fifo_m(
    input wire clk, rst, load, pop,
    input wire [7:0] hi_in, lo_in,
    output logic [1:0] color,
    output logic empty
);
    reg [7:0] hi, lo;
    reg [7:0] filled;

    assign empty = ~filled[7];
    assign color = {hi[7], lo[7]};
    always_ff @(posedge clk)
        if (~rst) filled <= 8'b0;
        else if (load /* temp: */ && empty) begin
            {hi, lo} <= {hi_in, lo_in};
            filled <= 8'hff;
        end else if (pop) begin
            filled <= {filled[6:0], 1'b0};
            hi <= {hi[6:0], 1'b0};
            lo <= {lo[6:0], 1'b0};
        end
endmodule

module scanline_renderer(
    input wire clk, rst,
    input wire [7:0] ly, scx, scy, wx, wy, wlc,
    input wire wlc_valid,
    input lcdc_t lcdc,
    output logic [6:0] oam_addr,
    input wire [15:0] oam_in,
    output logic [12:0] vram_addr,
    input wire [7:0] vram_in,
    output logic draw_pixel,
    output logic [1:0] pixel_color,
    output logic sprites_loaded, done
);
    // ~ Internal registers and state ~
    renderer_state_t state;
    reg [7:0] lx;
    reg in_window, in_sprite;
    reg reset_fetcher, push_obj_fifo, push_bg_fifo;
    reg first_tile;

    // Buffers:
    reg [7:0] tile_id, data_lo, data_hi;

    // Calculations:
    reg [7:0] bg_x, bg_y, win_x;
    assign bg_y = ly + scy, bg_x = lx + scx + (first_tile ? 8'd0 : 8'd8);
    assign win_x = lx - (wx - 8'd7);

    reg [12:0] bg_map_addr, win_map_addr;
    assign bg_map_addr = {2'b11, lcdc.bg_tile_map, bg_y[7:3], bg_x[7:3]};
    assign win_map_addr = {2'b11, lcdc.win_tile_map, ly[7:3], lx[7:3]};

    reg [1:0] bg_block;
    reg [11:0] obj_data_addr, bg_data_addr, data_addr;
    assign obj_data_addr = {2'b00, sprite.tile, sprite.dy};
    assign bg_block = tile_id[7] ? 2'b01 : (lcdc.bg_win_tile_data ? 2'b00 : 2'b10);
    assign bg_data_addr = {bg_block, tile_id[6:0], in_window ? wlc[2:0] : bg_y[2:0]};
    assign data_addr = in_sprite ? obj_data_addr : bg_data_addr;

    reg window_start;
    assign window_start = lcdc.win_ena && wlc_valid && ~|win_x;

    // FIFOs:
    wire [1:0] obj_color;
    wire no_obj;
    fifo_m obj_fifo (
        .clk(clk), .rst(rst), .load(push_obj_fifo), .pop(~no_obj && draw_pixel),
        .hi_in(data_hi), .lo_in(data_lo),
        .color(obj_color),
        .empty(no_obj)
    );

    wire [1:0] bg_color;
    wire no_bg;
    fifo_m bg_fifo (
        .clk(clk), .rst(rst), .load(push_bg_fifo), .pop(draw_pixel),
        .hi_in(data_hi), .lo_in(data_lo),
        .color(bg_color),
        .empty(no_bg)
    );

    assign draw_pixel = ~done & ~has_sprite & ~in_sprite & (~no_bg | ~lcdc.bg_ena);
    always_comb case ({no_obj, no_bg})
        2'b00: pixel_color = ~|obj_color ? bg_color : obj_color;
        2'b01: pixel_color = obj_color;
        2'b10: pixel_color = bg_color;
        2'b11: pixel_color = 2'b00;
    endcase

    // ~ Sprites ~
    wire has_sprite;
    sprite_data_t sprite, sprite_out;
    sprite_chain sprites (
        .clk(clk), .rst(rst), .load(~sprites_loaded), .query(~in_sprite | push_obj_fifo),
        .ly(ly), .lx(lx),
        .oam_addr(oam_addr),
        .oam_d_in(oam_in),
        .cfg_tall_sprites(lcdc.obj_size),
        .d_out(sprite_out),
        .d_valid(has_sprite)
    );

    // ~ Fetcher ~
    assign reset_fetcher = (window_start && ~in_window) || push_obj_fifo || push_bg_fifo || (has_sprite && ~in_sprite) || ~sprites_loaded;
    always_comb begin
        vram_addr = 0;
        push_obj_fifo = 0; push_bg_fifo = 0;
        case (state)
            S_FETCH_TILE: vram_addr = in_window ? win_map_addr : bg_map_addr;
            S_FETCH_DATA_LO: vram_addr = {data_addr, 1'b0};
            S_FETCH_DATA_HI: vram_addr = {data_addr, 1'b1};
            S_PUSH_FIFO: begin
                push_obj_fifo = in_sprite;
                push_bg_fifo = ~in_sprite && no_bg;
            end
        endcase
    end

    reg ce;
    always_ff @(posedge clk)
        if (~rst) begin lx <= 0; sprites_loaded <= 0; in_window <= 0; state <= S_FETCH_TILE; done <= 0; first_tile <= 1; in_sprite <= 0; ce <= 0; end
        else if (~done) begin
            ce <= ~ce;
            if (ce) case (state)
                S_FETCH_TILE: tile_id <= vram_in;
                S_FETCH_DATA_LO: data_lo <= vram_in;
                S_FETCH_DATA_HI: data_hi <= vram_in;
            endcase

            if (push_obj_fifo || push_bg_fifo) first_tile <= 0;
            if (push_obj_fifo) in_sprite <= 0;
            else if (has_sprite) begin in_sprite <= 1; sprite <= sprite_out; end

            if (reset_fetcher) state <= S_FETCH_TILE;
            else if (ce && state != S_PUSH_FIFO) state <= renderer_state_t'(state + 1'b1);

            if (~sprites_loaded && oam_addr == 7'd79) sprites_loaded <= 1;

            if (draw_pixel) lx <= lx + 1'b1;
            if (lx == 8'd159) done <= 1;
        end
endmodule

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
    output logic irq_vblank, irq_stat
);
    // ~ Regs ~
    // Internal:
    reg [7:0] ly, lyc, wlc;
    reg wlc_valid;
    reg rst;
    reg frame_done, restart_frame;
    reg [8:0] dot_ctr;
    ppu_phase_t phase, phase_prev;

    // External:
    lcdc_t lcdc;
    int_sels_t int_sels;
    reg [7:0] scy, scx, bgp, wx, wy;
    always_comb case (reg_addr)
        LCDC: reg_d_rd = lcdc;
        STAT: reg_d_rd = {1'b0, int_sels, lyc == ly, phase};
        SCY: reg_d_rd = scy;
        SCX: reg_d_rd = scx;
        LY: reg_d_rd = ly;
        LYC: reg_d_rd = lyc;
        BGP: reg_d_rd = bgp;
        WX: reg_d_rd = wx;
        WY: reg_d_rd = wy;
        default: reg_d_rd = 'x;
    endcase
    always_ff @(posedge clk) if (reg_write) case (ppu_reg_t'(reg_addr))
        LCDC: lcdc <= d_wr;
        STAT: int_sels <= d_wr[6:3];
        LYC: lyc <= d_wr;
        SCY: scy <= d_wr;
        SCX: scx <= d_wr;
        BGP: bgp <= d_wr;
        WX: wx <= d_wr;
        WY: wy <= d_wr;
    endcase

    assign irq_stat =
        (int_sels.mode2_int_sel && phase == PHASE_OAM_SCAN) ||
        (int_sels.mode1_int_sel && phase == PHASE_VBLANK) ||
        (int_sels.mode0_int_sel && phase == PHASE_HBLANK) ||
        (int_sels.lyc_int_sel && lyc == ly);

    wire [6:0] renderer_oam_addr;
    wire [12:0] renderer_vram_addr;
    wire scanline_loaded, scanline_done;
    scanline_renderer renderer(
        .clk(clk), .rst(|dot_ctr & ~frame_done),
        .ly(ly), .scx(scx), .scy(scy), .wx(wx), .wy(wy), .wlc(wlc),
        .wlc_valid(wlc_valid),
        .lcdc(lcdc),
        .oam_addr(renderer_oam_addr),
        .oam_in(oam_db),
        .vram_addr(renderer_vram_addr),
        .vram_in(vram_d_rd),
        .draw_pixel(lcd_pixel),
        .pixel_color(lcd_color),
        .sprites_loaded(scanline_loaded), .done(scanline_done)
    );

    // VRAM:
    reg [12:0] vram_addr;
    reg vram_write;

    sp_8192w_8b vram_block (
        .dout(vram_d_rd),
        .clk(~clk),
        .oce(1'b0),
        .ce(1'b1),
        .reset(1'b0),
        .wre(vram_write),
        .ad(vram_addr),
        .din(d_wr)
    );

    // OAM:
    wire [15:0] oam_db, oam_dout;
    assign oam_db = {oam_dout[7:0], oam_dout[15:8]};
    reg [6:0] oam_addr_rd;
    assign oam_d_rd = oam_addr_in[0] ? oam_db[15:8] : oam_db[7:0];
    assign oam_addr_rd = phase == PHASE_OAM_SCAN ? renderer_oam_addr : oam_addr_in[7:1];
    oam_sdpb oam_block (
        .clka(clk),
        .cea(oam_write && (frame_done || scanline_done || ~rst)),
        .reseta(1'b0),
        .clkb(~clk),
        .ceb(1'b1),
        .resetb(1'b0),
        .oce(1'b0),
        .ada(oam_addr_in),
        .din(d_wr),
        .adb(oam_addr_rd),
        .dout(oam_dout)
    );

/*
    oam_dpb oam_block(
        .doutb(oam_db), //output [15:0] doutb
        .clka(~clk), //input clka
        .ocea(1'b0), //input ocea
        .cea(1'b1), //input cea
        .reseta(1'b0), //input reseta
        .wrea(oam_write), //input wrea
        .clkb(~clk), //input clkb
        .oceb(1'b0), //input oceb
        .ceb(1'b1), //input ceb
        .resetb(1'b0), //input resetb
        .wreb(1'b0), //input wreb
        .ada(oam_addr_in), //input [7:0] ada
        .dina(d_wr), //input [7:0] dina
        .adb(oam_addr_rd), //input [6:0] adb
        .dinb(16'b0) //input [15:0] dinb
    );
*/

    assign rst = lcdc.ena;
    assign restart_frame = ~rst | ly == 8'd154;
    always_ff @(posedge clk) begin
        phase_prev <= phase;
        if (restart_frame) begin
            dot_ctr <= 0;
            ly <= 0;
            wlc <= 0; wlc_valid <= 0;
            frame_done = 0;
        end else if (dot_ctr == 9'd455) begin
            dot_ctr <= 0;
            ly <= ly + 1'b1;
            frame_done <= frame_done | (ly == 8'd143);
        end else dot_ctr <= dot_ctr + 1'b1;
    end

    always_comb
        if (frame_done) phase = PHASE_VBLANK;
        else if (scanline_done) phase = PHASE_HBLANK;
        else if (scanline_loaded) phase = PHASE_DRAW;
        else phase = PHASE_OAM_SCAN;

    assign lcd_hsync = phase == PHASE_HBLANK;
    assign lcd_vsync = phase == PHASE_VBLANK;
    assign irq_vblank = phase_prev != phase && phase == PHASE_VBLANK;

    always_comb case (phase)
        PHASE_DRAW: begin vram_addr = renderer_vram_addr; vram_write = 0; end
        default: begin vram_addr = vram_addr_in; vram_write = vram_write_in; end
    endcase
endmodule

`default_nettype wire
