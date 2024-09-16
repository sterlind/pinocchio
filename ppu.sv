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
    always_ff @(posedge clk) case (reg_addr)
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
            if (lx == 9'd456) begin
                lx <= 0;
                if (ly == 8'd153) ly <= 0;
                else ly <= ly + 1'b1;
            end
        end
    end
endmodule