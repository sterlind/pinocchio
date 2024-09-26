module gb_display(
    input wire tclk, dclk, rst,
    input wire ppu_vs, ppu_hs, ppu_de,
    input wire [1:0] ppu_color,
    output logic vs, hs, de,
    output logic [4:0] r, b,
    output logic [5:0] g
);
    wire [9:0] vga_x, vga_y, ppu_x, ppu_y;

    ppu_timer ppu_time (
        .clk(tclk),
        .rst(rst),
        .vs(ppu_vs),
        .hs(ppu_hs),
        .de(ppu_de),
        .x(ppu_x),
        .y(ppu_y)
    );

    vga_timer vga_time (
        .clk(dclk),
        .rst(rst),
        .vs(vs),
        .hs(hs),
        .de(de),
        .x(vga_x),
        .y(vga_y)
    );

    wire [14:0] ppu_addr_write, vga_addr_read;
    wire ppu_addr_valid, vga_addr_valid;
    fb_addr_encoder ppu_addr_enc (
        .x(ppu_x),
        .y(ppu_y),
        .valid(ppu_addr_valid),
        .addr(ppu_addr_write)
    );
    fb_addr_encoder vga_addr_enc (
        .x(vga_x >> 1),
        .y(vga_y >> 1),
        .valid(vga_addr_valid),
        .addr(vga_addr_read)
    );

    wire [1:0] fb_out;
    wire [1:0] any = 2'bx;
    dpb_fbram fb (
        .douta(any),
        .doutb(fb_out),
        .clka(tclk),
        .ocea(1'b0),
        .cea(1'b1),
        .reseta(~rst),
        .wrea(ppu_de),
        .clkb(dclk),
        .oceb(1'b0),
        .ceb(1'b1),
        .resetb(~rst),
        .wreb(1'b0),
        .ada(ppu_addr_write),
        .dina(ppu_color),
        .adb(vga_addr_read),
        .dinb(any)
    );

    localparam C00 = 24'h0f380f;
    localparam C01 = 24'h306230;
    localparam C10 = 24'h8bac0f;
    localparam C11 = 24'h9bbc0f;

    reg [15:0] vga_color;
    always_comb case (~fb_out)
        2'b00: vga_color = {C00[23-:5], C00[15-:6], C00[7-:5]};
        2'b01: vga_color = {C01[23-:5], C01[15-:6], C01[7-:5]};
        2'b10: vga_color = {C10[23-:5], C10[15-:6], C10[7-:5]};
        2'b11: vga_color = {C11[23-:5], C11[15-:6], C11[7-:5]};
    endcase

    assign {r, g, b} = vga_addr_valid ? vga_color : 16'b0;
endmodule

module ppu_timer (
    input wire clk, rst, vs, hs, de,
    output logic [9:0] x, y
);
    reg delta;
    always_ff @(posedge clk or negedge rst)
        if (~rst || vs) begin x <= 0; y <= 0; delta <= 0; end
        else if (hs) begin x <= 0; y <= y + delta; delta <= 0; end
        else if (de) begin x <= x + 1'b1; delta <= 1'b1; end
endmodule

module fb_addr_encoder(
    input wire [9:0] x, y,
    output logic valid,
    output logic [14:0] addr
);
    localparam WIDTH = 10'd160, HEIGHT = 10'd144;
    assign valid = x < WIDTH && y < HEIGHT;
    assign addr = y * WIDTH + x;
endmodule

typedef enum logic [1:0] {
    S_SYNC, S_FP, S_ACTIVE, S_BP
} state_t;

module tracker (
    input wire clk, ce, rst,
    output logic start, sync, active,
    output logic [9:0] pos
);
    parameter T_SYNC = 16'd0, T_FP = 16'd0, T_ACTIVE = 16'd0, T_BP = 16'd0;
    state_t curr;
    reg [9:0] dur;
    always_comb begin
        sync = 1; active = 0; start = 0;
        case (curr)
            S_SYNC: begin dur = T_SYNC - 1'b1; sync = 0; start = ~|pos; end
            S_FP: dur = T_FP - 1'b1;
            S_ACTIVE: begin dur = T_ACTIVE - 1'b1; active = 1; end
            S_BP: dur = T_BP - 1'b1;
        endcase
    end
    always_ff @(posedge clk or negedge rst)
        if (~rst) begin curr <= S_BP; pos <= 10'd0; end
        else if (ce) begin
            if (dur == pos) begin curr <= state_t'(curr + 1'b1); pos <= 10'd0; end
            else pos <= pos + 10'd1;
        end
endmodule

module vga_timer (
    input wire clk, rst,
    output logic hs, vs, de,
    output logic [9:0] x, y
);
    parameter H_ACTIVE = 16'd800; 
    parameter H_BP = 16'd40;      
    parameter H_SYNC = 16'd128;   
    parameter H_FP = 16'd88;      
    parameter V_ACTIVE = 16'd480; 
    parameter V_BP = 16'd1;     
    parameter V_SYNC = 16'd3;    
    parameter V_FP = 16'd21;    

    reg h_start, h_active;
    tracker #(.T_SYNC(H_SYNC), .T_FP(H_FP), .T_ACTIVE(H_ACTIVE), .T_BP(H_BP)) h_tracker (
        .clk(clk), .ce(1'b1), .rst(rst),
        .start(h_start), .sync(hs), .active(h_active),
        .pos(x)
    );

    reg v_active;
    tracker #(.T_SYNC(V_SYNC), .T_FP(V_FP), .T_ACTIVE(V_ACTIVE), .T_BP(V_BP)) v_tracker (
        .clk(clk), .ce(h_start), .rst(rst),
        .sync(vs), .active(v_active),
        .pos(y)
    );

    assign de = h_active & v_active;
endmodule