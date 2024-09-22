module top(
    input wire clk_27m,
    input wire btn_rst,
    output wire [4:0] lcd_r, lcd_b,
    output wire [5:0] lcd_g,
    output wire lcd_de, lcd_hsync, lcd_vsync, lcd_dclk
);
    reg tclk;
    tclk_pll pll(.clkin(clk_27m), .clkout(tclk));

    reg led_sig;
    assign led_sig = btn_rst;
    assign status_led = led_sig;
    reg rst;
    reg nrst;
    reg [3:0] rst_ctr;
    always_ff @(posedge tclk) begin
        if (rst) rst_ctr <= 4'b0000;
        if (~&rst_ctr) rst_ctr <= rst_ctr + 1'b1;
    end
    assign nrst = &rst_ctr;

    debounce rst_debounce (
        .clk(tclk),
        .d(btn_rst),
        .q_fall(rst)
    );

    wire [14:0] rom_addr;
    wire [7:0] rom_data;
    cart_prom cart (
        .clk(tclk),
        .oce('x),
        .ce(1'b1),
        .reset(1'b0),
        .ad(rom_addr),
        .dout(rom_data)
    );

    assign lcd_dclk = clk_27m;
    wire ppu_vs, ppu_hs, ppu_draw;
    wire [1:0] ppu_color;
    gb_display disp(
        .tclk(tclk),
        .dclk(lcd_dclk),
        .rst(nrst),
        .ppu_vs(ppu_vs),
        .ppu_hs(ppu_hs),
        .ppu_de(ppu_draw),
        .ppu_color(ppu_color),
        .vs(lcd_vsync),
        .hs(lcd_hsync),
        .de(lcd_de),
        .r(lcd_r),
        .g(lcd_g),
        .b(lcd_b)
    );
    
    dmg_main dmg(
        .clk(tclk),
        .rst(nrst),
        .rom_addr(rom_addr),
        .rom_data(rom_data),
        .lcd_vsync(ppu_vs),
        .lcd_hsync(ppu_hs),
        .lcd_pixel(ppu_draw),
        .lcd_color(ppu_color)
    );
endmodule