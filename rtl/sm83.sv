`default_nettype none
typedef enum logic [7:0] {
    // Block 0:
    NOP         = 8'b00000000,

    LD_RR_NN    = 8'b00xx0001,
    LD_R16M_A   = 8'b00xx0010,
    LD_A_R16M   = 8'b00xx1010,
    LD_NN_SP    = 8'b00001000,

    INCDEC_RR   = 8'b00xxx011,
    ADD_HL_RR   = 8'b00xx1001,

    INCDEC_HL   = 8'b0011010x,
    INCDEC_R    = 8'b00xxx10x,

    LD_HL_N     = 8'b00110110,
    LD_R_N      = 8'b00xxx110,

    RLCA        = 8'b00000111,
    RRCA        = 8'b00001111,
    RLA         = 8'b00010111,
    RRA         = 8'b00011111,
    CPL         = 8'b00101111,

    JR_E        = 8'b00011000,
    JR_CC_E     = 8'b001xx000,
    
    // Block 1:
    LD_HL_R     = 8'b01110xxx,
    LD_R_HL     = 8'b01xxx110,
    LD_R_R      = 8'b01xxxxxx,

    // Block 2:
    ALU_A_HL    = 8'b10xxx110,
    ALU_A_R     = 8'b10xxxxxx,

    // Block 3:
    ALU_A_N     = 8'b11xxx110,

    RET_CC      = 8'b110xx000,
    RET         = 8'b11001001,
    RETI        = 8'b11011001,
    JP_CC_NN    = 8'b110xx010,
    JP_NN       = 8'b11000011,
    JP_HL       = 8'b11101001,
    CALL_CC_NN  = 8'b110xx100,
    CALL_NN     = 8'b11001101,
    RST_N       = 8'b11xxx111,

    POP_R16S    = 8'b11xx0001,
    PUSH_R16S   = 8'b11xx0101,

    PREFIX      = 8'b11001011,

    LDH_N_A     = 8'b11100000,
    LDH_A_N     = 8'b11110000,
    LDH_C_A     = 8'b11100010,
    LD_A_NN     = 8'b11111010,
    LD_NN_A     = 8'b11101010,

    DI          = 8'b11110011,
    EI          = 8'b11111011,

    // Internal:
    IRQ         = 8'hd3
} opcode_t;

typedef enum logic [7:0] {
    SRU_R_HL    = 8'b00xxx110,
    SRU_R       = 8'b00xxxxxx,
    BIT_B_HL    = 8'b01xxx110,
    BIT_B_R     = 8'b01xxxxxx,
    RES_B_HL    = 8'b10xxx110,
    RES_B_R     = 8'b10xxxxxx,
    SET_B_HL    = 8'b11xxx110,
    SET_B_R     = 8'b11xxxxxx
} prefix_opcode_t;

typedef enum logic [2:0] {
    ALU_ADD,
    ALU_ADC,
    ALU_SUB,
    ALU_SBC,
    ALU_AND,
    ALU_XOR,
    ALU_OR,
    ALU_CP
} alu_op_t;

typedef enum logic [1:0] {
    SRU_OP,
    SRU_BIT,
    SRU_SET,
    SRU_RES
} sru_mode_t;

typedef enum logic [2:0] {
    SRU_RLC,
    SRU_RRC,
    SRU_RL,
    SRU_RR,
    SRU_SLA,
    SRU_SRA,
    SRU_SWAP,
    SRU_SRL
} sru_op_t;

typedef struct packed {
    bit f_z;
    bit f_n;
    bit f_h;
    bit f_c;
} flags_t;

typedef enum logic [1:0] {
    COND_NZ, COND_Z, COND_NC, COND_C
} cond_t;

typedef enum logic [2:0] {
    WZ, BC, DE, HL, AF, R16_SP, PC, REG16_ANY = 3'bxxx
} reg16_t;

typedef enum logic [1:0] {
    AB_NO_MASK,
    AB_MASK_OR_FF00,
    AB_MASK_AND_FF00,
    AB_ZERO
} ab_mask_t;

typedef enum logic [3:0] {
    NONE = 4'b0000 /* i.e. NONE */,
    Z, W,
    B, C, D, E, H, L,
    SPH, SPL,
    PCH, PCL,
    A,
    F = 4'b1110,
    MEM = 4'b1111,
    REG8_ANY = 4'bxxxx
} reg8_t;
`define MIN_REG8 Z
`define MAX_REG8 A 

typedef enum logic [2:0] {
    ACC_A, ACC_DB, ACC_SPL, ACC_SPH, ACC_PCL, ACC_H, ACC_L
} s_acc_t;

typedef enum logic [1:0] {
    ARG_DB, ARG_ONE, ARG_FF
} s_arg_t;

typedef enum logic [1:0] {
    R_WB_DB, R_WB_ALU, R_WB_SRU
} s_r_wb_t;

typedef enum logic [2:0] {
    RR_WB_NONE, RR_WB_IDU, RR_WB_WZ, RR_WB_IRQ, RR_WB_TGT
} s_rr_wb_t;

typedef enum logic [1:0] {
    INC, DEC, ADJ
} idu_mode_t;

module r8_decoder(
    input wire [2:0] r8,
    output reg8_t r_idx
);
    always_comb case (r8)
        3'd0: r_idx = B;
        3'd1: r_idx = C;
        3'd2: r_idx = D;
        3'd3: r_idx = E;
        3'd4: r_idx = H;
        3'd5: r_idx = L;
        3'd6: r_idx = REG8_ANY;
        3'd7: r_idx = A;
    endcase
endmodule

module r16_decoder(
    input wire [1:0] r16,
    input wire is_stk,
    output reg16_t r_idx,
    output reg8_t rh_idx,
    output reg8_t rl_idx
);
    always @(*) case (r16)
        2'd0: {r_idx, rh_idx, rl_idx} = {BC, B, C};
        2'd1: {r_idx, rh_idx, rl_idx} = {DE, D, E};
        2'd2: {r_idx, rh_idx, rl_idx} = {HL, H, L};
        2'd3:
            if (is_stk) {r_idx, rh_idx, rl_idx} = {AF, A, F};
            else {r_idx, rh_idx, rl_idx} = {R16_SP, SPH, SPL};
    endcase
endmodule

module r16m_decoder(
    input wire [1:0] r16,
    output reg16_t s_ab,
    output idu_mode_t idu,
    output s_rr_wb_t s_rr_wb
);
    always @(*) case (r16)
        2'd0: {s_ab, idu, s_rr_wb} = {BC, INC, RR_WB_NONE};
        2'd1: {s_ab, idu, s_rr_wb} = {DE, INC, RR_WB_NONE};
        2'd2: {s_ab, idu, s_rr_wb} = {HL, INC, RR_WB_IDU};
        2'd3: {s_ab, idu, s_rr_wb} = {HL, DEC, RR_WB_IDU};
    endcase
endmodule

module sequencer(
    input wire clk,
    input wire ce,
    input wire rst,
    input wire done,
    input wire cond_t cond,
    input flags_t flags,
    input wire is_cond,
    input wire [2:0] next_cond,
    input wire [7:0] d_in,
    output reg [7:0] ir /* synthesis syn_keep=1 */,
    output reg in_prefix,
    output reg [2:0] step /* synthesis syn_keep=1 */,
    input wire int_req,
    output reg int_ack
);
    reg matched;
    always @(*) case(cond)
        COND_NZ: matched = !flags.f_z;
        COND_Z: matched = flags.f_z;
        COND_NC: matched = !flags.f_c;
        COND_C: matched = flags.f_c;
    endcase

    always_ff @(posedge clk) begin
        if (~rst) begin step <= 0; ir = 8'b0; in_prefix <= 0; end
        else if (ce)
            if (done) begin
                step <= 0;
                ir <= int_req ? IRQ : d_in;
                in_prefix <= 0;
                int_ack <= int_req;
            end
            else if (ir == PREFIX) begin ir <= d_in; in_prefix <= 1; end
            else if (is_cond && !matched) step <= next_cond;
            else step <= step + 1'b1;
    end
endmodule

module decoder(
    input wire [7:0] opcode,
    input wire in_prefix,
    input wire [2:0] step,          // Current step in the opcode (starts at 0, auto-increments until `done` or if `next_cond`.)
    output logic [2:0] next_cond,   // Overrides step auto-increment and sets to this step *if* `is_cond` and cc is unsatisfied.
    output logic done,              // Latches IR, resets step, begins next opcode.
    output logic is_cond,           // If true, branch on cc rather than auto-incrementing `step`.
    output cond_t cond,             // Which condition to branch on?
    output reg16_t s_ab,            // Which r16 to use for the address bus?
    output ab_mask_t ab_mask,       // How to mask the address bus?
    output reg8_t s_db,             // R8 to source db from (or MEM.)
    output reg8_t t_db,             // R8 (or MEM) load from db out.
    output s_r_wb_t s_r_wb,         // R8 writeback source?
    output logic [2:0] alu_op,      // ALU/SRU operation to perform? 
    output logic [1:0] sru_mode,    // SRU mode?
    output logic [2:0] idx,         // Index argument (for bit/res/set and rst.)
    output s_acc_t s_acc,           // Where to source ALU accumulator from?
    output s_arg_t s_arg,           // Where to source AU argument from?
    output idu_mode_t idu,          // Increment or decrement?
    output s_rr_wb_t s_rr_wb,       // R16 writeback source?
    output reg16_t t_rr_wb,         // R16 writeback target?
    output logic wr_pc,             // Latch PC from IDU out?
    output logic set_ime, rst_ime   // Set or reset IME?
);
    reg8_t r8_dst;
    reg8_t r8_src;
    r8_decoder dc_r8_dst(.r8(opcode[5:3]), .r_idx(r8_dst));
    r8_decoder dc_r8_src(.r8(opcode[2:0]), .r_idx(r8_src));

    reg16_t r16;
    reg8_t r16h, r16l;
    reg is_stk;
    r16_decoder dc_r16(.is_stk(is_stk), .r16(opcode[5:4]), .r_idx(r16), .rh_idx(r16h), .rl_idx(r16l));

    s_rr_wb_t r16m_s_rr_wb;
    reg16_t r16m_s_ab;
    idu_mode_t r16m_idu;
    r16m_decoder dc_r16m (.r16(opcode[5:4]), .s_rr_wb(r16m_s_rr_wb), .s_ab(r16m_s_ab), .idu(r16m_idu));

    assign cond = cond_t'(opcode[4:3]);
    assign idx = opcode[5:3];

    wire f_inc_rr = opcode[3];
    wire f_inc_r = opcode[0];
    wire [2:0] f_alu_op = opcode[5:3];

    always_comb begin
        // Defaults:
        is_stk = 0;
        done = 0; is_cond = 0; wr_pc = 0;
        s_r_wb = R_WB_DB;
        sru_mode = SRU_OP;
        ab_mask = AB_NO_MASK;
        s_arg = ARG_DB; idu = INC; alu_op = f_alu_op; s_acc = ACC_A; s_rr_wb = RR_WB_NONE; t_rr_wb = REG16_ANY; t_db = NONE;
        s_ab = REG16_ANY;
        s_db = NONE;
        next_cond = 0;
        set_ime = 0; rst_ime = 0;

        if (in_prefix) casex ({opcode, step})
            {SRU_R_HL,  3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {SRU_R_HL,  3'd1}: /* [hl] <- sru z */                      begin s_ab = HL; s_db = Z; t_db = MEM; s_r_wb = R_WB_SRU; sru_mode = SRU_OP; end
            {SRU_R_HL,  3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end
            {SRU_R,     3'd0}: /* r <- sru r; inc pc; done */           begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_r_wb = R_WB_SRU; s_db = r8_src; t_db = r8_src; end
            {BIT_B_HL,  3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {BIT_B_HL,  3'd1}: /* f_z <- bit(b, z); inc pc; done */     begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_r_wb = R_WB_SRU; sru_mode = SRU_BIT; s_db = Z; end
            {BIT_B_R,   3'd0}: /* f_z <- bit(b, r); inc pc; done */     begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_r_wb = R_WB_SRU; sru_mode = SRU_BIT; s_db = r8_src; end
            {RES_B_HL,  3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {RES_B_HL,  3'd1}: /* [hl] <- res(b, z); */                 begin s_ab = HL; s_r_wb = R_WB_SRU; sru_mode = SRU_RES; s_db = Z; t_db = MEM; end
            {RES_B_HL,  3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end
            {RES_B_R,   3'd0}: /* r <- rst(b, r); inc pc; done */       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_r_wb = R_WB_SRU; sru_mode = SRU_RES; s_db = r8_src; t_db = r8_src; end
            {SET_B_HL,  3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {SET_B_HL,  3'd1}: /* [hl] <- set(b, z); */                 begin s_ab = HL; s_r_wb = R_WB_SRU; sru_mode = SRU_SET; s_db = Z; t_db = MEM; end
            {SET_B_HL,  3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end
            {SET_B_R,   3'd0}: /* r <- set(b, r); inc pc; done */       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_r_wb = R_WB_SRU; sru_mode = SRU_SET; s_db = r8_src; t_db = r8_src; end
        endcase
        else casex ({opcode, step})
            {NOP,       3'd0}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_RR_NN,  3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_RR_NN,  3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_RR_NN,  3'd2}: /* r16 <- wz; inc pc; done */            begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_rr_wb = RR_WB_WZ; t_rr_wb = r16; end

            {LD_A_R16M, 3'd0}: /* z <- [r16m]; maybe inc/dec hl */      begin idu = r16m_idu; s_rr_wb = r16m_s_rr_wb; t_rr_wb = HL; s_ab = r16m_s_ab; s_db = MEM; t_db = Z; end
            {LD_A_R16M, 3'd1}: /* a <- z; inc pc; done */               begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = A; end

            {LD_R16M_A, 3'd0}: /* [r16m] <- a; maybe inc/dec hl */      begin idu = r16m_idu; s_rr_wb = r16m_s_rr_wb; t_rr_wb = HL; s_ab = r16m_s_ab; s_db = A; t_db = MEM; end
            {LD_R16M_A, 3'd1}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_NN_SP,  3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_NN_SP,  3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_NN_SP,  3'd2}: /* [wz] <- spl; inc wz */                begin s_ab = WZ; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = WZ; s_db = SPL; t_db = MEM; end
            {LD_NN_SP,  3'd3}: /* [wz] <- sph */                        begin s_ab = WZ; s_db = SPH; t_db = MEM; end
            {LD_NN_SP,  3'd4}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {INCDEC_RR, 3'd0}: /* inc/dec r16 */                        begin s_ab = r16; if (f_inc_rr) idu = DEC; else idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = r16; end
            {INCDEC_RR, 3'd1}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {ADD_HL_RR, 3'd0}: /* l <- add(l, r16l) */                  begin s_db = r16l; t_db = L; s_acc = ACC_L; s_r_wb = R_WB_ALU; alu_op = ALU_ADD; end
            {ADD_HL_RR, 3'd1}: /* h <- adc(h, r16h); inc pc; done */    begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = r16h; t_db = H; s_r_wb = R_WB_ALU; alu_op = ALU_ADC; s_acc = ACC_H; end 

            {INCDEC_HL, 3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {INCDEC_HL, 3'd1}: /* [hl] <- z +/- 1 */                    begin s_ab = HL; s_db = Z; t_db = MEM; s_r_wb = R_WB_ALU; s_arg = ARG_ONE; s_acc = ACC_DB; if (f_inc_r) alu_op = ALU_SUB; else alu_op = ALU_ADD; end
            {INCDEC_HL, 3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {INCDEC_R,  3'd0}: /* r8_d <- r8_d +/- 1; inc pc; done */   begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_acc = ACC_DB; s_arg = ARG_ONE; s_db = r8_dst; t_db = r8_dst; s_r_wb = R_WB_ALU; if (f_inc_r) alu_op = ALU_SUB; else alu_op = ALU_ADD; end

            {LD_HL_N,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end                    
            {LD_HL_N,   3'd1}: /* [hl] <- z */                          begin s_ab = HL; s_db = Z; t_db = MEM; end                                       
            {LD_HL_N,   3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_R_N,    3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_R_N,    3'd1}: /* r8_dst <- z; inc pc; done */          begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = r8_dst; end       

            {RLCA,      3'd0}: /* a <- sru(rl, a); inc pc; done */      begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_r_wb = R_WB_SRU; s_db = A; t_db = A; alu_op = SRU_RLC; end
            {RRCA,      3'd0}: /* a <- sru(rl, a); inc pc; done */      begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_r_wb = R_WB_SRU; s_db = A; t_db = A; alu_op = SRU_RRC; end
            {RLA,       3'd0}: /* a <- sru(rl, a); inc pc; done */      begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_r_wb = R_WB_SRU; s_db = A; t_db = A; alu_op = SRU_RL; end
            {RRA,       3'd0}: /* a <- sru(rl, a); inc pc; done */      begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_r_wb = R_WB_SRU; s_db = A; t_db = A; alu_op = SRU_RR; end
            {CPL,       3'd0}: /* a <- xor(a, ff); inc pc; done */      begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_r_wb = R_WB_ALU; t_db = A; s_arg = ARG_FF; alu_op = ALU_XOR; end

            {JR_E,      3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {JR_E,      3'd1}: /* z <- pcl + z; w <- adj pch */         begin s_ab = PC; ab_mask = AB_MASK_AND_FF00; idu = ADJ; s_db = Z; t_db = Z; s_r_wb = R_WB_ALU; alu_op = ALU_ADD; s_acc = ACC_PCL; s_arg = ARG_DB; s_rr_wb = RR_WB_IDU; t_rr_wb = WZ; end
            {JR_E,      3'd2}: /* pc <- inc wz; done */                 begin done = 1; idu = INC; s_ab = WZ; wr_pc = 1; end

            {JR_CC_E,   3'd0}: /* z <- [pc]; inc pc; cc or goto 3 */    begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; is_cond = 1; next_cond = 3'd3; end
            {JR_CC_E,   3'd1}: /* z <- pcl + z; w <- adj pch */         begin s_ab = PC; ab_mask = AB_MASK_AND_FF00; idu = ADJ; s_db = Z; t_db = Z; s_r_wb = R_WB_ALU; alu_op = ALU_ADD; s_acc = ACC_PCL; s_arg = ARG_DB; s_rr_wb = RR_WB_IDU; t_rr_wb = WZ; end
            {JR_CC_E,   3'd2}: /* pc <- inc wz; done */                 begin done = 1; idu = INC; s_ab = WZ; wr_pc = 1; end
            {JR_CC_E,   3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_HL_R,   3'd0}: /* [hl] <- r8_src */                     begin s_ab = HL; s_db = r8_src; t_db = MEM; end
            {LD_HL_R,   3'd1}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_R_HL,   3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {LD_R_HL,   3'd1}: /* r8_dst <- z; inc pc; done */          begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = r8_dst; end

            {LD_R_R,    3'd0}: /* r8_dst <- r8_src; inc pc; done */     begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = r8_src; t_db = r8_dst; end

            {ALU_A_HL,  3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {ALU_A_HL,  3'd1}: /* a <- alu(a, z); inc pc; done */       begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = Z; t_db = A; s_r_wb = R_WB_ALU; end

            {ALU_A_R,   3'd0}: /* a <- alu(a, r); inc pc; done */       begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = r8_src; t_db = A; s_r_wb = R_WB_ALU; end

            {ALU_A_N,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {ALU_A_N,   3'd1}: /* a <- alu(a, z); inc pc; done */       begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = Z; t_db = A; s_r_wb = R_WB_ALU; end

            {JP_CC_NN,  3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {JP_CC_NN,  3'd1}: /* w <- [pc]; inc pc; cc or goto 3 */    begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; is_cond = 1; next_cond = 3'd3; end
            {JP_CC_NN,  3'd2}: /* pc <- wz */                           begin ab_mask = AB_ZERO; s_rr_wb = RR_WB_WZ; t_rr_wb = PC; end
            {JP_CC_NN,  3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {JP_NN,     3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {JP_NN,     3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {JP_NN,     3'd2}: /* pc <- wz */                           begin ab_mask = AB_ZERO; s_rr_wb = RR_WB_WZ; t_rr_wb = PC; end
            {JP_NN,     3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {JP_HL,     3'd0}: /* pc <- inc hl; done */                 begin done = 1; idu = INC; s_ab = HL; wr_pc = 1; end

            {CALL_CC_NN,3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {CALL_CC_NN,3'd1}: /* w <- [pc]; inc pc; cc or goto 5*/     begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; is_cond = 1; next_cond = 3'd5; end
            {CALL_CC_NN,3'd2}: /* dec sp */                             begin idu = DEC; s_ab = R16_SP; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {CALL_CC_NN,3'd3}: /* [sp] <- pch; dec sp */                begin s_ab = R16_SP; s_db = PCH; t_db = MEM; idu = DEC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {CALL_CC_NN,3'd4}: /* [sp] <- pcl; pc <- wz */              begin s_ab = R16_SP; s_db = PCL; t_db = MEM; s_rr_wb = RR_WB_WZ; t_rr_wb = PC; end
            {CALL_CC_NN,3'd5}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {CALL_NN,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {CALL_NN,   3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {CALL_NN,   3'd2}: /* dec sp */                             begin idu = DEC; s_ab = R16_SP; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {CALL_NN,   3'd3}: /* [sp] <- pch; dec sp */                begin s_ab = R16_SP; s_db = PCH; t_db = MEM; idu = DEC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {CALL_NN,   3'd4}: /* [sp] <- pcl; pc <- wz */              begin s_ab = R16_SP; s_db = PCL; t_db = MEM; s_rr_wb = RR_WB_WZ; t_rr_wb = PC; end
            {CALL_NN,   3'd5}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {RST_N,     3'd0}: /* dec sp */                             begin idu = DEC; s_ab = R16_SP; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {RST_N,     3'd1}: /* [sp] <- pch; dec sp */                begin s_ab = R16_SP; s_db = PCH; t_db = MEM; idu = DEC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {RST_N,     3'd2}: /* [sp] <- pcl; pc <- tgt */             begin s_ab = R16_SP; s_db = PCL; t_db = MEM; s_rr_wb = RR_WB_TGT; t_rr_wb = PC; end
            {RST_N,     3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {RET_CC,    3'd0}: /* cc or goto 4 */                       begin ab_mask = AB_ZERO; is_cond = 1; next_cond = 3'd4; end
            {RET_CC,    3'd1}: /* z <- [sp]; inc sp */                  begin s_ab = R16_SP; s_db = MEM; t_db = Z; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {RET_CC,    3'd2}: /* w <- [sp]; inc sp */                  begin s_ab = R16_SP; s_db = MEM; t_db = W; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {RET_CC,    3'd3}: /* pc <- wz */                           begin ab_mask = AB_ZERO; s_rr_wb = RR_WB_WZ; t_rr_wb = PC; end
            {RET_CC,    3'd4}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {RET,       3'd0}: /* z <- [sp]; inc sp */                  begin s_ab = R16_SP; s_db = MEM; t_db = Z; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {RET,       3'd1}: /* w <- [sp]; inc sp */                  begin s_ab = R16_SP; s_db = MEM; t_db = W; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {RET,       3'd2}: /* pc <- wz */                           begin ab_mask = AB_ZERO; s_rr_wb = RR_WB_WZ; t_rr_wb = PC; end
            {RET,       3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {RETI,      3'd0}: /* z <- [sp]; inc sp */                  begin s_ab = R16_SP; s_db = MEM; t_db = Z; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {RETI,      3'd1}: /* w <- [sp]; inc sp */                  begin s_ab = R16_SP; s_db = MEM; t_db = W; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {RETI,      3'd2}: /* pc <- wz; ime <- 1 */                 begin ab_mask = AB_ZERO; s_rr_wb = RR_WB_WZ; t_rr_wb = PC; set_ime = 1; end
            {RETI,      3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {PREFIX,    3'd0}: /* inc pc */                             begin idu = INC; s_ab = PC; wr_pc = 1; end

            {LDH_C_A,   3'd0}: /* [ff00 + c] <- a */                    begin s_ab = BC; ab_mask = AB_MASK_OR_FF00; s_db = A; t_db = MEM; end
            {LDH_C_A,   3'd1}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LDH_A_N,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LDH_A_N,   3'd1}: /* a <- [ff00 + z] */                    begin s_ab = WZ; ab_mask = AB_MASK_OR_FF00; s_db = MEM; t_db = A; end
            {LDH_A_N,   3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LDH_N_A,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LDH_N_A,   3'd1}: /* [ff00 + z] <- a */                    begin s_ab = WZ; ab_mask = AB_MASK_OR_FF00; s_db = A; t_db = MEM; end
            {LDH_N_A,   3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {PUSH_R16S, 3'd0}: /* dec sp */                             begin idu = DEC; s_ab = R16_SP; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {PUSH_R16S, 3'd1}: /* [sp] <- r16h (stk); dec sp */         begin is_stk = 1; s_ab = R16_SP; s_db = r16h; t_db = MEM; idu = DEC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {PUSH_R16S, 3'd2}: /* [sp] <- r16l (stk) */                 begin is_stk = 1; s_ab = R16_SP; s_db = r16l; t_db = MEM; end
            {PUSH_R16S, 3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {POP_R16S,  3'd0}: /* z <- [sp]; inc sp */                  begin s_ab = R16_SP; s_db = MEM; t_db = Z; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {POP_R16S,  3'd1}: /* w <- [sp]; inc sp */                  begin s_ab = R16_SP; s_db = MEM; t_db = W; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {POP_R16S,  3'd2}: /* rr <- wz; inc pc; done */             begin is_stk = 1; done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_rr_wb = RR_WB_WZ; t_rr_wb = r16; end

            {LD_A_NN,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_A_NN,   3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_A_NN,   3'd2}: /* z <- [wz] */                          begin s_ab = WZ; s_db = MEM; t_db = A; end
            {LD_A_NN,   3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_NN_A,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_NN_A,   3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_NN_A,   3'd2}: /* [wz] <- a */                          begin s_ab = WZ; s_db = A; t_db = MEM; end
            {LD_NN_A,   3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {DI,        3'd0}: /* ime <- 0; inc pc; done */             begin done = 1; rst_ime = 1; idu = INC; s_ab = PC; wr_pc = 1; end
            {EI,        3'd0}: /* ime <- 1; inc pc; done */             begin done = 1; set_ime = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {IRQ,       3'd0}: /* dec pc */                             begin idu = DEC; s_ab = PC; wr_pc = 1; end
            {IRQ,       3'd1}: /* dec sp */                             begin idu = DEC; s_ab = R16_SP; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {IRQ,       3'd2}: /* [sp] <- pch; dec sp */                begin s_ab = R16_SP; s_db = PCH; t_db = MEM; idu = DEC; s_rr_wb = RR_WB_IDU; t_rr_wb = R16_SP; end
            {IRQ,       3'd3}: /* [sp] <- pcl; pc <- irq */             begin s_ab = R16_SP; s_db = PCL; t_db = MEM; s_rr_wb = RR_WB_IRQ; t_rr_wb = PC; end
            {IRQ,       3'd4}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end
            //default: $error("Bad opcode, step (%h, %d)", opcode, step);
        endcase
    end
endmodule

module interrupt_controller (
    input wire clk,
    input wire rst,
    // Interrupt requests in from external:
    input wire [7:0] irq /* synthesis syn_keep=1 */,
    // IME control:
    input wire rst_ime, set_ime,
    // Should we jump to an interrupt handler? Which one?
    output logic [2:0] idx,
    output logic req,
    input wire ack,
    // Bus access to IE and IF:
    output logic [7:0] ie_out, if_out,
    input wire [7:0] ie_in, if_in,
    input wire write_ie, write_if
);
    reg [7:0] r_ie /* synthesis syn_keep=1 */, r_if /* synthesis syn_keep=1 */;
    reg ime /* synthesis syn_keep=1 */;

    assign ie_out = r_ie, if_out = r_if;
    assign req = ime && |(r_ie & r_if);
    always_comb casex (r_if)
        8'bxxxxxxx1: idx = 3'd0;
        8'bxxxxxx10: idx = 3'd1;
        8'bxxxxx100: idx = 3'd2;
        8'bxxxx1000: idx = 3'd3;
        8'bxxx10000: idx = 3'd4;
        8'bxx100000: idx = 3'd5;
        8'bx1000000: idx = 3'd6;
        8'b10000000: idx = 3'd7;
        default: idx = 3'd0;
    endcase

    always_ff @(negedge clk)
        if (~rst) begin r_ie <= 0; r_if <= 0; ime <= 0; end
        else begin
            if (rst_ime || ack) ime <= 0;
            else if (set_ime) ime <= 1'b1;

            if (write_ie) r_ie <= ie_in;
            if (write_if) r_if <= if_in;
            else r_if <= (r_if & (ack ? ~(1'b1 << idx) : 8'hff)) | irq;
        end
endmodule

typedef enum logic [15:0] {
    HRAM    = 16'b1111_1111_1xxx_xxxx,
    IE      = 16'hffff,
    IF      = 16'hff0f
} cpu_internal_addr_t;

module sm83(
    input wire clk,
    input wire ce,
    input wire rst,
    input wire [7:0] irq_in,
    output reg [15:0] addr,
    input wire [7:0] d_in_ext,
    output wire [7:0] d_out,
    output reg write
);
    reg [15:0] ab;
    reg [7:0] db;
    reg [7:0] rf [`MIN_REG8:`MAX_REG8];
    flags_t flags;

    reg [7:0] d_in;

    wire [7:0] ir;
    wire [2:0] step;

    wire in_prefix;
    logic [2:0] c_next_cond;
    logic c_done;
    logic c_is_cond;
    cond_t c_cond;
    reg16_t c_s_ab;
    ab_mask_t c_ab_mask;
    reg8_t c_s_db;
    reg8_t c_t_db;
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
    reg c_rst_ime, c_set_ime;
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
        .wr_pc(c_wr_pc),
        .set_ime(c_set_ime),
        .rst_ime(c_rst_ime)
    );

    always_comb
        if (~rst) addr = 16'b0;
        else case (c_ab_mask)
            AB_ZERO: addr = 16'b0;
            AB_NO_MASK: addr = ab;
            AB_MASK_OR_FF00: addr = ab | 16'hff00;
            AB_MASK_AND_FF00: addr = ab & 16'hff00;
        endcase

    reg hram_write;
    wire [7:0] hram_rd;
    ram_128words_8bit hram (
        .clk(~clk),
        .ce(ce),
        .addr(addr[6:0]),
        .d_in(d_out),
        .d_out(hram_rd),
        .write(hram_write)
    );

    reg write_ie, write_if;
    reg [7:0] ie_rd, if_rd;
    reg should_write;
    always_comb begin
        hram_write = 0; write_ie = 0; write_if = 0; d_in = d_in_ext; write = 0;
        if (~rst) should_write = 0;
        else begin
            should_write = c_t_db == MEM;
            casex (addr)
                IE: begin write_ie = should_write; d_in = ie_rd; end
                IF: begin write_if = should_write; d_in = if_rd; end
                HRAM: begin hram_write = should_write; d_in = hram_rd; end
                default: begin d_in = d_in_ext; write = should_write; end
            endcase
        end
    end

    wire [2:0] int_idx;
    wire int_req;
    wire int_ack;
    interrupt_controller xint (
        .clk(clk),
        .rst(rst),
        .irq(irq_in),
        .rst_ime(c_rst_ime),
        .set_ime(c_set_ime),
        .idx(int_idx),
        .req(int_req),
        .ack(int_ack),
        .ie_out(ie_rd),
        .if_out(if_rd),
        .write_ie(write_ie),
        .write_if(write_if),
        .ie_in(d_out),
        .if_in(d_out)
    );

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
        .in_prefix(in_prefix),
        .int_req(int_req),
        .int_ack(int_ack)
    );

    // IDU:
    wire [15:0] idu_res;
    idu_m idu (
        .ab(addr),
        .mode(c_idu),
        .adj_bits({rf[Z][7],f_out.f_c}),
        .res(idu_res)
    );

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
            ACC_H: acc = rf[H];
            ACC_L: acc = rf[L];
            default: acc = db;
        endcase
        case (c_s_arg)
            ARG_DB: arg = db;
            ARG_ONE: arg = 8'd1;
            ARG_FF: arg = 8'hff;
            default: arg = db;
        endcase
    end

    // Register file:
    reg [15:0] rr_wb;
    reg [7:0] r_wb;
    flags_t f_out;

    always_ff @(posedge clk) begin
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
                    R16_SP: {rf[SPH], rf[SPL]} <= rr_wb;
                    AF: {rf[A], flags} <= rr_wb[15:4];
                    PC: {rf[PCH], rf[PCL]} <= rr_wb;
                endcase
            flags <= f_out;
            if (c_t_db != F && c_t_db != MEM && c_t_db != NONE)
                rf[c_t_db] <= r_wb;
        end
    end
    assign d_out = r_wb;

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
            R16_SP: ab = {rf[SPH], rf[SPL]};
            PC: ab = {rf[PCH], rf[PCL]};
            default: ab = 16'b0;
        endcase

        case (c_s_rr_wb)
            RR_WB_IDU: rr_wb = idu_res;
            RR_WB_WZ: rr_wb = {rf[W], rf[Z]};
            RR_WB_IRQ : rr_wb = {8'b0, 2'b01, int_idx, 3'b0};
            RR_WB_TGT: rr_wb = {10'b0, c_idx, 3'b0};
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
    input wire idu_mode_t mode,
    input wire [1:0] adj_bits,
    output logic [15:0] res
);
    reg [7:0] abh, abl;
    assign {abh, abl} = ab;
    always_comb
        case (mode)
            INC: res = ab + 1'b1;
            DEC: res = ab - 1'b1;
            ADJ: case (adj_bits)
                2'b01: res = {abh + 1'b1, abl};
                2'b10: res = {abh - 1'b1, abl};
                default: res = ab;
            endcase
            default: res = ab;
        endcase
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
        SRU_RES: {c_out, res} = {1'b0, in & ~(8'd1 << idx)};
        SRU_SET: {c_out, res} = {1'b0, in | ~(8'd1 << idx)};
        default: {c_out, res} = '0;
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

module ram_128words_8bit (
    input wire clk,
    input wire ce,
    input wire [6:0] addr,
    input wire write,
    input wire [7:0] d_in,
    output reg [7:0] d_out
);
    reg [7:0] mem[0:127];
    always_ff @(posedge clk) /*if (ce)*/ begin
        d_out <= mem[addr];
        if (write) mem[addr] <= d_in;
    end
endmodule

`default_nettype wire