typedef enum logic [7:0] {
    // Block 0:
    NOP         = 8'b00000000,

    LD_RR_NN    = 8'b00xx0001,
    LD_A_R16M   = 8'b00xx0010,
    LD_R16M_A   = 8'b00xx1010,
    LD_NN_SP    = 8'b00001000,

    INCDEC_RR   = 8'b00xxx011,
    ADD_HL_RR   = 8'b00xx1001,

    INCDEC_HL   = 8'b0011010x,
    INCDEC_R    = 8'b00xxx10x,

    LD_HL_N     = 8'b00110110,
    LD_R_N      = 8'b00xxx110,

    JR_E        = 8'b00111000,
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

    POP_R16S    = 8'b11xx0001,
    PUSH_R16S   = 8'b11xx0101,

    LD_A_NN     = 8'b11111010,
    LD_NN_A     = 8'b11101010
} opcode_t;

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

typedef enum logic [2:0] {
    WZ, BC, DE, HL, AF, SP, PC, PCH_ZERO, REG16_ANY = 3'bxxx
} reg16_t;

typedef enum logic [3:0] {
    MEM /* i.e. NONE */, Z, W, B, C, D, E, H, L, A, F, PCH, PCL, SPH, SPL, REG8_ANY = 4'bxxxx
} reg8_t;

typedef enum logic [1:0] {
    ACC_DB, ACC_SPL, ACC_SPH, ACC_PCL
} s_acc_t;

typedef enum logic {
    ARG_DB, ARG_ONE
} s_arg_t;

typedef enum logic [1:0] {
    RR_WB_NONE, RR_WB_IDU, RR_WB_WZ
} s_rr_wb_t;

typedef enum logic [1:0] {
    INC, DEC, ADJ
} idu_mode_t;

module r8_decoder(
    input wire [2:0] r8,
    output reg8_t r_idx
);
    always @(*) case (r8)
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
    input logic is_stk,
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
            else {r_idx, rh_idx, rl_idx} = {SP, SPH, SPL};
    endcase
endmodule

module r16m_decoder(
    input wire [1:0] r16,
    output reg16_t s_ab,
    output idu_mode_t idu,
    output s_rr_wb_t s_rr_wb
);
    always @(*) case (r16)
        2'd0: {s_ab, idu, s_rr_wb} = {BC, idu_mode_t'('x), RR_WB_NONE};
        2'd1: {s_ab, idu, s_rr_wb} = {DE, idu_mode_t'('x), RR_WB_NONE};
        2'd2: {s_ab, idu, s_rr_wb} = {HL, INC, RR_WB_IDU};
        2'd3: {s_ab, idu, s_rr_wb} = {HL, DEC, RR_WB_IDU};
    endcase
endmodule

module decoder(
    input opcode_t opcode,
    input wire [2:0] step,          // Current step in the opcode (starts at 0, auto-increments until `done` or if `next_cond`.)
    output logic [2:0] next_cond,   // Overrides step auto-increment and sets to this step *if* `is_cond` and cc is unsatisfied.
    output logic done,              // Latches IR, resets step, begins next opcode.
    output logic is_cond,           // If true, branch on cc rather than auto-incrementing `step`.
    output reg16_t s_ab,            // Which r16 to use for the address bus?
    output reg8_t s_db,             // R8 to source db from (or MEM.)
    output reg8_t t_db,             // R8 (or MEM) load from db out.
    output logic use_alu,           // Load db out with ALU result? or just transfer db in?
    output alu_op_t alu_op,         // ALU operation to perform? 
    output s_acc_t s_acc,           // Where to source ALU accumulator from?
    output s_arg_t s_arg,           // Where to source ALU argument from?
    output idu_mode_t idu,          // Increment or decrement?
    output s_rr_wb_t s_rr_wb,       // R16 writeback source?
    output reg16_t t_rr_wb,         // R16 writeback target?
    output logic wr_pc              // Latch PC from IDU out?
);
    reg8_t r8_dst, r8_src;
    r8_decoder dc_r8_dst(.r8(opcode[5:3]), .r_idx(r8_dst));
    r8_decoder dc_r8_src(.r8(opcode[2:0]), .r_idx(r8_src));

    reg16_t r16;
    reg8_t r16h, r16l;
    reg is_stk;
    r16_decoder dc_r16(.is_stk(is_stk), .r16(opcode[5:4]), .r_idx(r16), .rh_idx(r16h), .rl_idx(r16l));

    r16m_decoder r16m (.r16(opcode[5:4]));

    always @(*) begin
        // Defaults:
        is_stk = 0;
        done = 0; is_cond = 0; wr_pc = 0;
        use_alu = 0; s_arg = ARG_DB; idu = INC; alu_op = alu_op_t'(opcode[5:3]); s_acc = ACC_DB; s_rr_wb = RR_WB_NONE; t_rr_wb = REG16_ANY;

        casez ({opcode, step})
            {LD_RR_NN,  3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_RR_NN,  3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_RR_NN,  3'd2}: /* r16 <- wz; inc pc; done */            begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_rr_wb = RR_WB_WZ; t_rr_wb = r16; end

            {LD_A_R16M, 3'd0}: /* z <- [r16m]; maybe inc/dec hl */      begin idu = r16m.idu; s_rr_wb = r16m.s_rr_wb; t_rr_wb = HL; s_ab = r16m.s_ab; s_db = MEM; t_db = Z; end
            {LD_A_R16M, 3'd1}: /* a <- z; inc pc; done */               begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = A; end

            {LD_R16M_A, 3'd0}: /* [r16m] <- a; maybe inc/dec hl */      begin idu = r16m.idu; s_rr_wb = r16m.s_rr_wb; t_rr_wb = HL; s_ab = r16m.s_ab; s_db = A; end
            {LD_R16M_A, 3'd1}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_NN_SP,  3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_NN_SP,  3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_NN_SP,  3'd2}: /* [wz] <- spl; inc wz */                begin s_ab = WZ; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = WZ; s_db = SPL; t_db = MEM; end
            {LD_NN_SP,  3'd3}: /* [wz] <- sph */                        begin s_ab = WZ; s_db = SPH; t_db = MEM; end
            {LD_NN_SP,  3'd4}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {INCDEC_RR, 3'd0}: /* inc/dec r16 */                        begin s_ab = r16; if (opcode[3]) idu = DEC; else idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = r16; end
            {INCDEC_RR, 3'd1}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {ADD_HL_RR, 3'd0}: /* l <- add(l, r16l) */                  begin s_db = r16l; t_db = L; use_alu = 1; alu_op = ALU_ADD; end
            {ADD_HL_RR, 3'd1}: /* h <- adc(h, r16h); inc pc; done */    begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = r16h; t_db = H; use_alu = 1; alu_op = ALU_ADC; end 

            {INCDEC_HL, 3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {INCDEC_HL, 3'd1}: /* [hl] <- add(z, 1) */                  begin s_ab = HL; s_db = Z; t_db = MEM; use_alu = 1; alu_op = ALU_ADD; s_arg = ARG_ONE; end
            {INCDEC_HL, 3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {INCDEC_R,  3'd0}: /* r8_d <- r8_d +/- 1; inc pc; done */   begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_arg = ARG_ONE; s_db = r8_dst; t_db = r8_dst; use_alu = 1; if (opcode[0]) alu_op = ALU_SUB; else alu_op = ALU_ADD; end

            {LD_HL_N,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end                    
            {LD_HL_N,   3'd1}: /* [hl] <- z */                          begin s_ab = HL; s_db = Z; t_db = MEM; end                                       
            {LD_HL_N,   3'd2}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_R_N,    3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_R_N,    3'd1}: /* r8_dst <- z; inc pc; done */          begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = r8_dst; end       

            {JR_E,      3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {JR_E,      3'd1}: /* z <- pcl + z; w <- adj pch */         begin s_ab = PCH_ZERO; idu = ADJ; s_db = Z; t_db = Z; use_alu = 1; alu_op = ALU_ADD; s_acc = ACC_PCL; s_arg = ARG_DB; s_rr_wb = RR_WB_IDU; t_rr_wb = WZ; end
            {JR_E,      3'd2}: /* pc <- inc wz; done */                 begin done = 1; idu = INC; s_ab = WZ; wr_pc = 1; end

            {JR_CC_E,   3'd0}: /* z <- [pc]; inc pc; cc or goto 3 */    begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; is_cond = 1; next_cond = 3'd3; end
            {JR_CC_E,   3'd1}: /* z <- pcl + z; w <- adj pch */         begin s_ab = PCH_ZERO; idu = ADJ; s_db = Z; t_db = Z; use_alu = 1; alu_op = ALU_ADD; s_acc = ACC_PCL; s_arg = ARG_DB; s_rr_wb = RR_WB_IDU; t_rr_wb = WZ; end
            {JR_CC_E,   3'd2}: /* pc <- inc wz; done */                 begin done = 1; idu = INC; s_ab = WZ; wr_pc = 1; end
            {JR_CC_E,   3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_HL_R,   3'd0}: /* [hl] <- r8_src */                     begin s_ab = HL; s_db = r8_src; t_db = MEM; end
            {LD_HL_R,   3'd1}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_R_HL,   3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {LD_R_HL,   3'd1}: /* r8_dst <- z; inc pc; done */          begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = r8_dst; end

            {LD_R_R,    3'd0}: /* r8_dst <- r8_src; inc pc; done */     begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = r8_src; t_db = r8_dst; end

            {ALU_A_HL,  3'd0}: /* z <- [hl] */                          begin s_ab = HL; s_db = MEM; t_db = Z; end
            {ALU_A_HL,  3'd1}: /* a <- alu(a, z); inc pc; done */       begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = Z; t_db = A; use_alu = 1; end

            {ALU_A_R,   3'd0}: /* a <- alu(a, r); inc pc; done */       begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = r8_src; t_db = A; use_alu = 1; end

            {ALU_A_N,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {ALU_A_N,   3'd1}: /* a <- alu(a, z); inc pc; done */       begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = Z; t_db = A; use_alu = 1; end

            {PUSH_R16S, 3'd0}: /* dec sp */                             begin idu = DEC; s_ab = SP; s_rr_wb = RR_WB_IDU; t_rr_wb = SP; end
            {PUSH_R16S, 3'd1}: /* [sp] <- r16h (stk); dec sp */         begin is_stk = 1; s_ab = SP; s_db = r16h; t_db = MEM; idu = DEC; s_rr_wb = RR_WB_IDU; t_rr_wb = SP; end
            {PUSH_R16S, 3'd2}: /* [sp] <- r16l (stk) */                 begin is_stk = 1; s_ab = SP; s_db = r16l; t_db = MEM; end
            {PUSH_R16S, 3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {POP_R16S,  3'd0}: /* z <- [sp]; inc sp */                  begin s_ab = SP; s_db = MEM; t_db = Z; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = SP; end
            {POP_R16S,  3'd1}: /* w <- [sp]; inc sp */                  begin s_ab = SP; s_db = MEM; t_db = W; idu = INC; s_rr_wb = RR_WB_IDU; t_rr_wb = SP; end
            {POP_R16S,  3'd2}: /* rr <- wz; inc pc; done */             begin is_stk = 1; done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_rr_wb = RR_WB_WZ; t_rr_wb = r16; end

            {LD_A_NN,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_A_NN,   3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_A_NN,   3'd2}: /* z <- [wz] */                          begin s_ab = WZ; s_db = MEM; t_db = Z; end
            {LD_A_NN,   3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_NN_A,   3'd0}: /* z <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_NN_A,   3'd1}: /* w <- [pc]; inc pc */                  begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_NN_A,   3'd2}: /* [wz] <- a */                          begin s_ab = WZ; s_db = A; t_db = MEM; end
            {LD_NN_A,   3'd3}: /* inc pc; done */                       begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end
        endcase
    end
endmodule