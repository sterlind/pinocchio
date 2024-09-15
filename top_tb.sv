`include "sm83.sv"

module top_tb;
    reg clk, rst, write;
    wire [15:0] addr;
    reg [7:0] mem [0:'hffff];
    reg [7:0] d_in, d_out;

    sm83 cpu (
        .clk(clk),
        .rst(rst),
        .addr(addr),
        .d_in(d_in),
        .d_out(d_out),
        .write(write)
    );

    always_ff @(negedge clk) begin
        if (write) mem[addr] <= d_out;
        d_in <= mem[addr];
    end

    int fd, code;
    initial begin
        //$readmemh("rom.hex", mem, 0, 5);
        // For now:
        fd = $fopen("tetris.gb", "rb");
        code = $fread(mem, fd, 0, 'h7fff);
        $fclose(fd);
        // Boot rom overlaps cartridge rom.
        $readmemh("dmg_rom.hex", mem, 0, 255);
        clk = 1; rst = 0;       // cpu's comb logic sets addr = 0 and write = 0.
        #1; clk = 0;            // Falling edge loads d_in from addr 0.
        clk = 1; #1;            // Raise clock. cpu latches ir from d_in and zeros regs, since ~rst.
        rst = 1; clk = 0; #1;   // Reset is no longer asserted. cpu executes first op (comb), and its memory req is serviced by falling edge.
        forever #1 clk = ~clk;  // Should be up and running now!
    end

    genvar k;
    reg [15:0] bc, de, hl, sp, pc, af, wz;
    always_comb begin
        pc = {cpu.rf[PCH], cpu.rf[PCL]};
        af = {cpu.rf[A], cpu.flags, 4'b0};
        bc = {cpu.rf[B], cpu.rf[C]};
        de = {cpu.rf[D], cpu.rf[E]};
        hl = {cpu.rf[H], cpu.rf[L]};
        sp = {cpu.rf[SPH], cpu.rf[SPL]};
        wz = {cpu.rf[W], cpu.rf[Z]};
    end

    initial begin
        $dumpvars(0, cpu);
        $dumpvars(0, bc);
        $dumpvars(0, de);
        $dumpvars(0, hl);
        $dumpvars(0, af);
        $dumpvars(0, sp);
        $dumpvars(0, pc);
        $dumpvars(0, wz);
    end

    initial #128 $finish;
endmodule