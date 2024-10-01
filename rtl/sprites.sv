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
    input wire clk, rst, load, next,
    input sprite_bus_t prev_in,
    output sprite_bus_t next_out
);
    reg ready;              // If slot is filled but hasn't been read yet.
    reg [7:0] x;            // X position of the sprite to match against during queries.
    sprite_data_t data;     // Data to return, once it's our turn.

    // We're chosen for a query if we're the first valid slot with a matching x in the chain.
    // If chosen, we push our data the bus and set bypass.
    reg chosen;
    assign chosen = ~load & ~rst & ~prev_in.bypass & ready & (x == prev_in.x);
    assign next_out = prev_in.bypass ? prev_in : {8'bx, data, 1'b1};

    always_ff @(posedge clk)
        // Reset is simple: just mark slot as empty.
        if (~rst) ready <= 0;
        // It's our turn to load if we're not yet ready but all previous slots were.
        else if (load & ~ready & ~prev_in.bypass) begin
            ready <= 1;
            x <= prev_in.x;
            data <= prev_in.data;
        end
        // After being chosen in query mode, mark slot as empty to give others a turn.
        else if (chosen && next) ready <= 0;

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
    wire sprite_bus_t bus_in, bus_out;
    wire sprite_bus_t [0:8] bus;
    sprite_slot slots [0:9] (
        .clk(clk), .rst(rst), .load(load), .next(next),
        .prev_in({bus_in, bus}),
        .next_out({bus, bus_out})
    );

    assign d_out = bus_out.data, d_valid = ~load && query && bus_out.bypass;

    // Buffer the first word of OAM, so we can assemble the whole entry before storing it in a slot.
    reg [15:0] oam_buf; 
    oam_entry_t entry;
    assign entry = {oam_d_in, oam_buf};

    // Calculate y offset within sprite, and check if it's visible on this scan line.
    reg [7:0] dy, tile;
    reg [3:0] dy_corr;
    assign dy = ly - (entry.y - 8'd16);
    assign dy_corr = entry.attrs[6] ? ~dy[3:0] : dy[3:0]; 
    reg visible;
    assign visible = cfg_tall_sprites ? ~dy[7:4] : ~dy[7:3];
    assign tile = cfg_tall_sprites ? {entry.tile[7:1], dy_corr[3]} : entry.tile;

    assign bus_in = load
        // During load, fill slots with OAM data. Bypass all slots if sprite is out of range or not yet buffered.
        ? {entry.x, {dy_corr[2:0], tile, entry.attrs[7:4]}, ~(visible & oam_addr[0])}
        // Otherwise, send a query through.
        : {lx, sprite_data_t'('x), ~query};

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
        else if (load) begin
            {hi, lo} <= {hi_in, lo_in};
            filled <= 8'hff;
        end else if (pop) begin
            filled <= {filled[6:0], 1'b0};
            hi <= {hi[6:0], 1'b0};
            lo <= {lo[6:0], 1'b0};
        end
endmodule

module scanline_renderer(
    input wire clk, ce, rst,
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

    // Buffers:
    reg [7:0] tile_id, data_lo, data_hi;

    // Calculations:
    reg [7:0] bg_x, bg_y, win_x;
    assign bg_y = ly + scy, bg_x = lx + scx;
    assign win_x = lx - (wx - 8'd7);

    reg [12:0] bg_map_addr, win_map_addr;
    assign bg_map_addr = {2'b11, lcdc.bg_tile_map, bg_y[7:3], bg_x[7:3]};
    assign win_map_addr = {2'b11, lcdc.win_tile_map, ly[7:3], lx[7:3]};

    reg [1:0] bg_block;
    reg [11:0] obj_data_addr, bg_data_addr, data_addr;
    assign obj_data_addr = {2'b00, sprite.tile, sprite.dy};
    assign bg_block = tile_id[7] ? 2'b01 : (lcdc.bg_win_tile_data ? 2'b00 : 2'b10);
    assign bg_data_addr = {bg_block, tile_id, in_window ? wlc[2:0] : bg_y[2:0]};
    assign data_addr = in_sprite ? obj_data_addr : bg_data_addr;

    reg window_start;
    assign window_start = lcdc.win_ena && wlc_valid && ~|win_x;

    // FIFOs:
    wire [1:0] obj_color;
    wire no_obj;
    fifo_m obj_fifo (
        .clk(clk), .rst(rst), .load(push_obj_fifo), .pop(draw_pixel),
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

    assign draw_pixel = (~no_obj | ~lcdc.obj_ena | (~in_sprite & ~has_sprite)) & (~no_bg | ~lcdc.bg_ena);
    always_comb case ({no_obj, no_bg})
        2'b00: pixel_color = ~|obj_color ? bg_color : obj_color;
        2'b01: pixel_color = obj_color;
        2'b10: pixel_color = bg_color;
        2'b11: pixel_color = 'x;
    endcase

    // ~ Sprites ~
    reg has_sprite;
    sprite_data_t sprite;
    sprite_chain sprites (
        .clk(clk), .rst(rst), .load(~sprites_loaded), .query(~has_sprite | push_obj_fifo),
        .ly(ly), .lx(lx),
        .oam_addr(oam_addr),
        .oam_d_in(oam_in),
        .cfg_tall_sprites(lcdc.obj_size),
        .d_out(sprite),
        .d_valid(has_sprite)
    );

    // ~ Fetcher ~
    assign reset_fetcher = (window_start && ~in_window) || push_obj_fifo || push_bg_fifo || (has_sprite && ~in_sprite);
    always_comb begin
        vram_addr = 'x;
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

    always_ff @(posedge clk)
        if (~rst) begin lx <= 0; sprites_loaded <= 0; in_window <= 0; state <= S_FETCH_TILE; done <= 0; end
        else if (ce) begin
            case (state)
                S_FETCH_TILE: tile_id <= vram_in;
                S_FETCH_DATA_LO: data_lo <= vram_in;
                S_FETCH_DATA_HI: data_hi <= vram_in;
            endcase

            in_sprite <= has_sprite;

            if (reset_fetcher) state <= S_FETCH_TILE;
            else if (state != S_PUSH_FIFO) state <= state + 1'b1;

            if (~sprites_loaded && oam_addr == 7'd79) sprites_loaded <= 1;

            if (draw_pixel) lx <= lx + 1'b1;
            if (lx == 8'd159) done <= 1;
        end
endmodule