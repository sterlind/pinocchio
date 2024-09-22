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

    reg zeroed;
    always_ff @(posedge clk) if (~zeroed) begin
        vram_addr <= vram_addr + 1'b1;
        if (vram_addr == 13'h1fff) zeroed = 1;
    end

    initial begin
        zeroed = 1;
        vram_write = 0;
        vram_addr = 0;
        reg_addr = LCDC; reg_d_wr = 8'h00; reg_write = 1;
        #2;
        reg_addr = SCX;
        #2;
        reg_addr = SCY;
        #2;
        reg_write = 0;
        zeroed = 0; vram_write = 1; vram_d_wr = 8'h00;
        #('h1fff * 2);
        vram_write = 0; reg_write = 1; reg_d_wr = 8'h81; reg_addr = LCDC;
        #2;
    end

    initial begin
        $dumpvars(0, ppu);
        #(2*(3 + 'h1fff + 1 + 456*154)) $finish;
    end
endmodule