typedef enum logic [3:0] {
    LCDC,
    STAT,
    SCY,
    SCX,
    LY,
    LYC,
    DMA,
    BGP,
    OBP0,
    OBP1,
    WY,
    WX
} ppu_ctrl_reg_t;

typedef struct packed {
    bit lcd_ena;
    bit win_tile_map;
    bit win_ena;
    bit tile_data;
    bit bg_tile_map;
    bit obj_size;
    bit obj_ena;
    bit bg_win_ena;
} lcdc_t;

typedef struct packed {
    bit lyc_int_ena;
    bit [2:0] mode_int_ena;
    bit lyc_match;
    bit [1:0] mode;
} stat_t;

module ppu_ctrl(
    input wire clk,
    // Ctrl <-> System Bus
    input wire [3:0] reg_addr,
    input wire [7:0] d_in,
    input wire write,
    output wire [7:0] d_out,
    // PPU <-> Ctrl
    input byte ly,
    output lcdc_t lcdc,
    input stat_t stat,
    output byte scy, scx
);
endmodule

module ppu(
    input wire clk,
    output wire [15:0] addr,
    input wire [7:0] d_in,
    output wire draw,
    output wire [7:0] draw_x, draw_y,
    output wire [1:0] draw_c
);
endmodule

typedef struct packed {
    bit pri;
    bit y_flip;
    bit x_flip;
    bit pal;
    bit [2:0] reserved;
} obj_attr_t;

typedef struct packed {
    bit [7:0] y, x, idx;
    obj_attr_t attr;
} obj_t;

module obj_entry(
    input wire clk,
    input wire load,    // Load signal. Shifts stored obj right.
    input obj_t o_in,   // Object in (from entry to the left.)
    output obj_t o_out  // Object out (to entry to the right.)
);
    always_ff @(posedge clk) if (load) o_out <= o_in;
endmodule
