typedef struct packed {
    bit [2:0] dy;    // Y offset within the sprite block (for current ly.)
    reg [7:0] tile;  // Tile index.
    reg [3:0] attrs; // Sprite attributes.
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
        else if (chosen) ready <= 0;

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
        .clk(clk), .rst(rst), .load(load),
        .prev_in({bus_in, bus}),
        .next_out({bus, bus_out})
    );

    assign d_out = bus_out.data, d_valid = bus_out.bypass;

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
        // Otherwise, send a query through, bypassing if query mode isn't active.
        : {lx, sprite_data_t'('x), ~query};

    always_ff @(posedge clk)
        if (~rst) oam_addr <= 0;
        else if (load) begin
            oam_addr <= oam_addr + 1'b1;
            if (~oam_addr[0]) oam_buf <= oam_d_in;
        end
endmodule