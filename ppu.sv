typedef enum logic [3:0] {
    LCDC = 4'h0,
    LY = 4'h4
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
    input wire [12:0] vram_addr,
    input wire [7:0] vram_d_wr,
    output logic [7:0] vram_d_rd,
    input wire vram_write,
    // OAM:
    input wire [7:0] oam_addr,
    input wire [7:0] oam_d_wr,
    output logic [7:0] oam_d_rd,
    input wire oam_write,
    // Display:
    output wire lcd_clk, lcd_ena,
    output wire lcd_hsync, lcd_vsync,
    output wire [1:0] lcd_color
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