`default_nettype none
module sm83(
    input wire clk,
    input wire ce,
    input wire rst,
    output reg [15:0] addr,
    input wire [7:0] d_in /* synthesis syn_keep=1 */,
    output wire [7:0] d_out /* synthesis syn_keep=1 */,
    output reg write
);
    reg [15:0] ab;
    reg [7:0] db;
    reg [7:0] rf /* synthesis syn_keep=1 */ [`MIN_REG8:`MAX_REG8];
    flags_t flags;

    reg [15:0] pc2 /* synthesis syn_noprune syn_keep=1 syn_preserve=1 */;
    assign pc2 = {rf[PCH], rf[PCL]};

    assign d_out = db;

    wire [7:0] ir;
    wire [2:0] step;

    wire in_prefix;
    logic [2:0] c_next_cond;
    logic c_done;
    logic c_is_cond;
    cond_t c_cond;
    reg16_t c_s_ab;
    ab_mask_t c_ab_mask;
    reg8_t c_s_db /* synthesis syn_keep=1 */;
    reg8_t c_t_db /* synthesis syn_keep=1 */;
    s_r_wb_t c_s_r_wb;
    logic [2:0] c_alu_op;
    logic [1:0] c_sru_mode;
    logic [2:0] c_idx;
    s_acc_t c_s_acc;
    s_arg_t c_s_arg;
    idu_mode_t c_idu;
    s_rr_wb_t c_s_rr_wb;
    reg16_t c_t_rr_wb;
    reg c_wr_pc;
    decoder ctrl (
        .opcode(ir),
        .step(step),
        .in_prefix(in_prefix),
        .next_cond(c_next_cond),
        .done(c_done),
        .is_cond(c_is_cond),
        .cond(c_cond),
        .s_ab(c_s_ab),
        .ab_mask(c_ab_mask),
        .s_db(c_s_db),
        .t_db(c_t_db),
        .s_r_wb(c_s_r_wb),
        .alu_op(c_alu_op),
        .sru_mode(c_sru_mode),
        .idx(c_idx),
        .s_acc(c_s_acc),
        .s_arg(c_s_arg),
        .idu(c_idu),
        .s_rr_wb(c_s_rr_wb),
        .t_rr_wb(c_t_rr_wb),
        .wr_pc(c_wr_pc)
    );

    always_comb
        if (!rst) begin write = 0; addr = 16'b0; end
        else begin
            case (c_ab_mask)
                AB_ZERO: addr = 16'b0;
                AB_NO_MASK: addr = ab;
                AB_MASK_OR_FF00: addr = ab | 16'hff00;
                AB_MASK_AND_FF00: addr = ab & 16'hff00;
            endcase
            write = c_t_db == MEM;
        end

    // Sequencer:
    sequencer seq (
        .clk(clk),
        .ce(ce),
        .rst(rst),
        .done(c_done),
        .cond(c_cond),
        .flags(flags),
        .is_cond(c_is_cond),
        .next_cond(c_next_cond),
        .d_in(d_in),
        .ir(ir),
        .step(step),
        .in_prefix(in_prefix)
    );

    // IDU:
    reg idu_inc, idu_bypass;
    reg [1:0] adj_bits;
    assign adj_bits = {f_out.f_z, f_out.f_c};
    wire [15:0] idu_res;
    idu_m idu (
        .ab(addr),
        .inc(idu_inc),
        .bypass(idu_bypass),
        .res(idu_res)
    );
    always_comb begin
        idu_bypass = 0; idu_inc = 1;
        case (c_idu)
            INC: idu_inc = 1;
            DEC: idu_inc = 0;
            ADJ: begin
                case (adj_bits)
                    2'b01: idu_inc = 1;
                    2'b10: idu_inc = 0;
                    default: idu_bypass = 1;
                endcase
            end
        endcase
    end

    // ALU:
    reg [7:0] arg, acc, alu_res, sru_res;
    flags_t alu_f_out, sru_f_out;
    alu_m alu(
        .op(alu_op_t'(c_alu_op)),
        .acc(acc),
        .arg(arg),
        .c_in(flags.f_c),
        .f_out(alu_f_out),
        .res(alu_res)
    );
    sru_m sru(
        .op(sru_op_t'(c_alu_op)),
        .mode(sru_mode_t'(c_sru_mode)),
        .idx(c_idx),
        .in(db),
        .f_in(flags),
        .f_out(sru_f_out),
        .res(sru_res)
    );

    always_comb begin
        case (c_s_acc)
            ACC_A: acc = rf[A];
            ACC_DB: acc = db;
            ACC_SPL: acc = rf[SPL];
            ACC_SPH: acc = rf[SPH];
            ACC_PCL: acc = rf[PCL];
            default: acc = 'x;
        endcase
        case (c_s_arg)
            ARG_DB: arg = db;
            ARG_ONE: arg = 8'd1;
        endcase
    end

    // Register file:
    reg [15:0] rr_wb;
    reg [7:0] r_wb;
    flags_t f_out;

    always_ff @(posedge clk or negedge rst) begin
        if (~rst) begin
            for (int k = `MIN_REG8; k < `MAX_REG8 + 1; k = k + 1)
                rf[k] <= 0;
            {rf[SPH], rf[SPL]} <= 'hffff;
            flags <= 4'h0;
        end else if (ce) begin
            if (c_wr_pc) {rf[PCH], rf[PCL]} <= idu_res; // Todo: do we always pull from idu?
            if (c_s_rr_wb != RR_WB_NONE)
                case (c_t_rr_wb)
                    WZ: {rf[W], rf[Z]} <= rr_wb;
                    BC: {rf[B], rf[C]} <= rr_wb;
                    DE: {rf[D], rf[E]} <= rr_wb;
                    HL: {rf[H], rf[L]} <= rr_wb;
                    SP: {rf[SPH], rf[SPL]} <= rr_wb;
                    AF: {rf[A], flags} <= rr_wb[15:4];
                    PC: {rf[PCH], rf[PCL]} <= rr_wb;
                endcase
            flags <= f_out;
            if (c_t_db != F && c_t_db != MEM && c_t_db != NONE)
                rf[c_t_db] <= r_wb;
        end
    end

    always_comb begin
        case (c_s_db)
            MEM: db = d_in;
            F: db = {flags, 4'd0};
            default: db = rf[c_s_db];
        endcase

        case (c_s_ab)
            AF: ab = {rf[A], flags, 4'b0};
            WZ: ab = {rf[W], rf[Z]};
            BC: ab = {rf[B], rf[C]};
            DE: ab = {rf[D], rf[E]};
            HL: ab = {rf[H], rf[L]};
            SP: ab = {rf[SPH], rf[SPL]};
            PC: ab = {rf[PCH], rf[PCL]};
            default: ab = 'x;
        endcase

        case (c_s_rr_wb)
            RR_WB_IDU: rr_wb = idu_res;
            RR_WB_WZ: rr_wb = {rf[W], rf[Z]};
            default: rr_wb = 16'hxx;
        endcase

        case (c_s_r_wb)
            R_WB_ALU: begin r_wb = alu_res; f_out = alu_f_out; end
            R_WB_SRU: begin r_wb = sru_res; f_out = sru_f_out; end
            default: begin r_wb = db; f_out = flags; end
        endcase
    end
endmodule

module idu_m (
    input wire [15:0] ab,
    input wire inc,
    input wire bypass,
    output logic [15:0] res
);
    always_comb
        if (bypass) res = ab;
        else if (inc) res = ab + 1'b1; else res = ab - 1'b1;
endmodule

module sru_m (
    input wire sru_op_t op,
    input wire [7:0] in,
    input flags_t f_in,
    input wire sru_mode_t mode,
    input bit [2:0] idx,
    output logic [7:0] res,
    output flags_t f_out
);
    wire c_in = f_in.f_c;
    reg c_out, bit_set;
    assign bit_set = in[idx];
    always_comb case (mode)
        SRU_OP: f_out = {3'b0, c_out};
        SRU_BIT: f_out = {~bit_set, 3'b0};
        default: f_out = f_in;
    endcase
    always_comb case (mode)
        SRU_OP: begin
            case (op)
                SRU_RL: {c_out, res} = {in, c_in};
                SRU_RR: {c_out, res} = {c_in, in};
                default: {c_out, res} = {1'bx, in};
            endcase
        end
        default: {c_out, res} = 'x;
    endcase
endmodule

module alu_m (
    input wire alu_op_t op,
    input wire [7:0] acc,
    input wire [7:0] arg,
    input bit c_in,
    output flags_t f_out,
    output logic [7:0] res
);
    reg z, n, h, c, carry;
    reg [7:0] added, sum;
    assign {h, sum[3:0]} = acc[3:0] + added[3:0] + carry;
    assign {c, sum[7:4]} = acc[7:4] + added[7:4] + h;
    assign z = res == 0;

    always_comb begin
        case (op)
            ALU_ADC, ALU_SBC: carry = c_in;
            ALU_SUB, ALU_CP: carry = 1'b1;
            default: carry = 1'b0;
        endcase
        case (op)
            ALU_SUB, ALU_SBC, ALU_CP: begin added = ~arg; n = 1; end
            default: begin added = arg; n = 0; end
        endcase
        f_out = {z, 3'b0};
        case (op)
            ALU_XOR: res = acc ^ arg;
            ALU_OR: res = acc | arg;
            ALU_AND: res = acc & arg;
            default: begin res = sum; f_out = {z, n, h, c}; end
        endcase
    end

endmodule

`default_nettype wire