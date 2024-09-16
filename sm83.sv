`include "decoder.sv"

module sm83(
    input wire clk,
    input wire rst,
    output reg [15:0] addr,
    input wire [7:0] d_in,
    output wire [7:0] d_out,
    output reg write
);
    reg [15:0] ab;
    reg [7:0] db;
    reg [7:0] rf [`MIN_REG8:`MAX_REG8];
    reg flags_t flags;

    assign d_out = db;

    wire [7:0] ir;
    wire [2:0] step;

    decoder ctrl (
        .opcode(ir),
        .step(step)
    );

    always_comb
        if (!rst) begin write = 0; addr = 16'b0; end
        else begin
            case (ctrl.ab_mask)
                AB_ZERO: addr = 16'b0;
                AB_NO_MASK: addr = ab;
                AB_MASK_OR_FF00: addr = ab | 16'hff00;
                AB_MASK_AND_FF00: addr = ab & 16'hff00;
            endcase
            write = ctrl.t_db == MEM;
        end

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
    reg idu_inc, idu_bypass;
    reg [1:0] adj_bits;
    assign adj_bits = {alu.f_out.f_z, alu.f_out.f_c};
    idu_m idu (
        .ab(addr),
        .inc(idu_inc),
        .bypass(idu_bypass)
    );
    always_comb begin
        idu_bypass = 0; idu_inc = 1;
        case (ctrl.idu)
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
    reg [7:0] arg, acc;
    alu_m alu(
        .op(ctrl.alu_op),
        .acc(acc),
        .arg(arg),
        .c_in(flags.f_c)
    );
    sru_m sru(
        .op(ctrl.alu_op),
        .mode(ctrl.sru_mode),
        .idx(ctrl.idx),
        .in(db),
        .f_in(flags)
    );

    always_comb begin
        case (ctrl.s_acc)
            ACC_A: acc = rf[A];
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
    reg [7:0] r_wb;
    reg flags_t f_out;

    always_ff @(posedge clk) begin
        if (!rst) begin
            for (int k = `MIN_REG8; k < `MAX_REG8 + 1; k = k + 1)
                rf[k] <= 0;
            {rf[SPH], rf[SPL]} <= 'hffff;
            flags <= 4'h0;
        end else begin
            if (ctrl.wr_pc) {rf[PCH], rf[PCL]} <= idu.res; // Todo: do we always pull from idu?
            if (ctrl.s_rr_wb != RR_WB_NONE)
                case (ctrl.t_rr_wb)
                    WZ: {rf[W], rf[Z]} <= rr_wb;
                    BC: {rf[B], rf[C]} <= rr_wb;
                    DE: {rf[D], rf[E]} <= rr_wb;
                    HL: {rf[H], rf[L]} <= rr_wb;
                    SP: {rf[SPH], rf[SPL]} <= rr_wb;
                    AF: {rf[A], flags} <= rr_wb[15:4];
                endcase
            if (ctrl.t_db != F && ctrl.t_db != MEM && ctrl.t_db != NONE) begin
                flags <= f_out;
                rf[ctrl.t_db] <= r_wb;
            end
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

        case (ctrl.s_r_wb)
            R_WB_ALU: begin r_wb = alu.res; f_out = alu.f_out; end
            R_WB_SRU: begin r_wb = sru.res; f_out = sru.f_out; end
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
        else if (inc) res = ab + 1; else res = ab - 1;
endmodule

module sru_m (
    input sru_op_t op,
    input wire [7:0] in,
    input flags_t f_in,
    input sru_mode_t mode,
    input bit [2:0] idx,
    output logic [7:0] res,
    output flags_t f_out
);
    wire c_in = f_in.f_c;
    reg c_out, bit_set;
    assign bit_set = in[idx];
    always_comb case (mode)
        SRU_OP: f_out = {3'b0, c_out};
        SRU_BIT: f_out = {bit_set, 3'b0};
        default: f_out = f_in;
    endcase
    always_comb case (mode)
        SRU_OP: begin
            case (op)
                SRU_RL: {c_out, res} = {in, c_in};
                SRU_RR: {c_out, res} = {c_in, in};
                default: {c_out, res} = 9'bx; // Todo
            endcase
        end
    endcase
endmodule

module alu_m (
    input alu_op_t op,
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