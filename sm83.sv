`include "decoder.sv"

module sm83(
    input wire clk,
    input wire rst,
    output wire [15:0] addr,
    input wire [7:0] d_in,
    output wire [7:0] d_out,
    output reg write
);
    reg [15:0] ab;
    reg [7:0] db;
    reg [7:0] rf [`MIN_REG8:`MAX_REG8];
    reg flags_t flags;

    assign addr = ab, d_out = db;

    wire [7:0] ir;
    wire [2:0] step;

    decoder ctrl (
        .opcode(ir),
        .step(step)
    );

    // Sequencer:
    sequencer seq (
        .clk(clk),
        .rst(rst),
        .done(ctrl.done),
        .cond(ctrl.cond),
        .flags(flags),
        .is_cond(ctrl.is_cond),
        .next_cond(ctrl.next_cond),
        .d_in(d_in),
        .ir(ir),
        .step(step)
    );

    // IDU:
    idu_m idu (
        .ab(ab),
        .mode(ctrl.idu)
    );

    // ALU:
    reg [7:0] arg, acc;
    alu_m alu(
        .op(ctrl.alu_op),
        .acc(acc),
        .arg(arg),
        .f_in(flags)
    );
    always_comb begin
        case (ctrl.s_acc)
            ACC_DB: acc = db;
            ACC_SPL: acc = rf[SPL];
            ACC_SPH: acc = rf[SPH];
            ACC_PCL: acc = rf[PCL];
        endcase
        case (ctrl.s_arg)
            ARG_DB: arg = db;
            ARG_ONE: arg = 8'd1;
        endcase
    end

    // Register file:
    reg [15:0] rr_wb;
    always_ff @(posedge clk or negedge rst) begin
        if (!rst) begin
            for (int k = `MIN_REG8; k < `MAX_REG8; k = k + 1)
                rf[k] <= 0;
            flags <= 4'h0;
        end else begin
            if (ctrl.wr_pc) {rf[PCH], rf[PCL]} <= ab;
            if (ctrl.s_rr_wb != RR_WB_NONE)
                case (ctrl.t_rr_wb)
                    WZ: {rf[W], rf[Z]} <= rr_wb;
                    BC: {rf[B], rf[C]} <= rr_wb;
                    DE: {rf[D], rf[E]} <= rr_wb;
                    HL: {rf[H], rf[L]} <= rr_wb;
                    SP: rf[SP] <= rr_wb;
                    AF: {rf[A], flags} <= rr_wb[15:4];
                endcase
            write <= ctrl.t_db == MEM;
            if (ctrl.t_db != F && ctrl.t_db != MEM) rf[ctrl.t_db] <= ctrl.use_alu ? alu.res : db;
        end
    end

    always_comb begin
        case (ctrl.s_db)
            MEM: db = d_in;
            F: db = {flags, 4'd0};
            default: db = rf[ctrl.s_db];
        endcase

        case (ctrl.s_ab)
            AF: ab = {rf[A], flags, 4'b0};
            PCH_ZERO: ab = {rf[PCH], 8'b0};
            WZ: ab = {rf[W], rf[Z]};
            BC: ab = {rf[B], rf[C]};
            DE: ab = {rf[D], rf[E]};
            HL: ab = {rf[H], rf[L]};
            SP: ab = {rf[SPH], rf[SPL]};
            PC: ab = {rf[PCH], rf[PCL]};
        endcase

        case (ctrl.s_rr_wb)
            RR_WB_IDU: rr_wb = idu.res;
            RR_WB_WZ: rr_wb = {rf[W], rf[Z]};
            default: rr_wb = 16'hxx;
        endcase
    end
endmodule

module idu_m (
    input wire [15:0] ab,
    input idu_mode_t mode,
    output wire [15:0] res
);
endmodule

module alu_m (
    input alu_op_t op,
    input wire [7:0] acc,
    input wire [7:0] arg,
    input flags_t f_in,
    output flags_t f_out,
    output wire [7:0] res
);
endmodule