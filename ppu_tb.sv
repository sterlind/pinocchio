`include "ppu.sv"

module ppu_tb;
    reg clk;

    reg [12:0] vram_addr;
    reg vram_write;
    reg [7:0] vram_d_wr;

    reg reg_write;
    reg [7:0] reg_d_wr;
    ppu_reg_t reg_addr;
    ppu_m ppu (
        .clk(clk),
        .reg_addr(reg_addr),
        .reg_write(reg_write),
        .reg_d_wr(reg_d_wr),

        .vram_addr_in(vram_addr),
        .vram_write_in(vram_write),
        .vram_d_wr(vram_d_wr)
    );

    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end

    reg need_zero;
    always @(posedge clk)
        if (need_zero) begin
            vram_addr <= vram_addr + 1'b1;
            vram_d_wr <= 0;
            vram_write <= 1;
            if (vram_addr == 13'h1fff) begin
                need_zero <= 0;
                vram_write <= 0;
            end
        end


    initial begin
        vram_addr <= 0;
        need_zero = 0;
        reg_d_wr = 8'd0; reg_addr = LCDC; reg_write = 1;
        #2 reg_addr = SCX;
        #2 reg_addr = SCY;
        #2 need_zero = 1;
        #('h1fff*2);
        #2 reg_d_wr = 8'b10000001; reg_addr = LCDC; reg_write = 1;
        #2 reg_write = 0;
    end

    initial begin
        $dumpvars(0, ppu);
        #(456 * 154 + 'h1ffff*2 + 10) $finish;
    end
endmodule