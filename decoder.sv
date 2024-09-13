typedef enum logic [7:0] {
    LD_HL_N     = 8'b00110110,
    LD_R_N      = 8'b00xxx110,
    LD_HL_R     = 8'b00110xxx,
    LD_R_HL     = 8'b01xxx110,
    LD_R_R      = 8'b01xxxxxx,
    LD_A_R16M   = 8'b00xx0010,
    LD_R16M_A   = 8'b00xx1010,
    ALU_A_HL    = 8'b10xxx110,
    ALU_A_R     = 8'b10xxxxxx,
    ALU_A_N     = 8'b11xxx110,
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
    WZ, BC, DE, HL, SP, PC
} reg16_t;

typedef enum logic [3:0] {
    MEM /* i.e. NONE */, Z, W, B, C, D, E, H, L, A, PCH, PCL, SPH, SPL, REG8_ANY = 4'bxxxx
} reg8_t;

typedef enum logic [1:0] {
    ACC_DB, ACC_SPL, ACC_SPH, ACC_PCL
} s_acc_t;

typedef enum bit { INC, DEC } idu_mode_t;

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

module r16m_decoder(
    input wire [1:0] r16m,
    output reg16_t r_idx,
    output reg8_t rh_idx,
    output reg8_t rl_idx
);
    always @(*) case (r16m)
        2'd0: {r_idx, rh_idx, rl_idx} = {BC, B, C};
        2'd1: {r_idx, rh_idx, rl_idx} = {DE, D, E};
        2'd2: {r_idx, rh_idx, rl_idx} = {HL, H, L};
        2'd3: {r_idx, rh_idx, rl_idx} = {SP, SPH, SPL};
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
    output idu_mode_t idu,          // Increment or decrement?
    output logic wr_pc,             // Latch PC from IDU out?
    output logic write_mem          // Request an external write?
);
    reg8_t r8_dst, r8_src;
    r8_decoder dc_r8_dst(.r8(opcode[5:3]), .r_idx(r8_dst));
    r8_decoder dc_r8_src(.r8(opcode[2:0]), .r_idx(r8_src));

    reg16_t r16m;
    reg8_t r16h, r16l;
    r16m_decoder dc_r16m(.r16m(opcode[5:4]), .r_idx(r16m), .rh_idx(r16h), .rl_idx(r16l));

    always @(*) begin
        // Defaults:
        done = 0; is_cond = 0; write_mem = 0; wr_pc = 0; use_alu = 0; idu = INC; alu_op = alu_op_t'(opcode[5:3]); s_acc = ACC_DB;

        casez ({opcode, step})
            {LD_HL_N,   3'd0}: /* z <- [pc]; inc pc */              begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end                    
            {LD_HL_N,   3'd1}: /* [hl] <- z */                      begin s_ab = HL; s_db = Z; write_mem = 1; end                                       
            {LD_HL_N,   3'd2}: /* inc pc; done */                   begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_R_N,    3'd0}: /* z <- [pc]; inc pc */              begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_R_N,    3'd1}: /* r8_dst <- z; inc pc; done */      begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = r8_dst; end       

            {LD_HL_R,   3'd0}: /* [hl] <- r8_src */                 begin s_ab = HL; s_db = r8_src; write_mem = 1; end
            {LD_HL_R,   3'd1}: /* inc pc; done */                   begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_R_HL,   3'd0}: /* z <- [hl] */                      begin s_ab = HL; s_db = MEM; t_db = Z; end
            {LD_R_HL,   3'd1}: /* r8_dst <- z; inc pc; done */      begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = r8_dst; end

            {LD_R_R,    3'd0}: /* r8_dst <- r8_src; inc pc; done */ begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = r8_src; t_db = r8_dst; end

            {LD_A_R16M, 3'd0}: /* z <- [r16m] */                    begin s_ab = r16m; s_db = MEM; t_db = Z; end
            {LD_A_R16M, 3'd1}: /* a <- z; inc pc; done */           begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; s_db = Z; t_db = A; end

            {LD_R16M_A, 3'd0}: /* [r16m] <- a */                    begin s_ab = r16m; s_db = A; write_mem = 1; end
            {LD_R16M_A, 3'd1}: /* inc pc; done */                   begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_A_NN,   3'd0}: /* z <- [pc]; inc pc */              begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_A_NN,   3'd1}: /* w <- [pc]; inc pc */              begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_A_NN,   3'd2}: /* z <- [wz] */                      begin s_ab = WZ; s_db = MEM; t_db = Z; end
            {LD_A_NN,   3'd3}: /* inc pc; done */                   begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {LD_NN_A,   3'd0}: /* z <- [pc]; inc pc */              begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {LD_NN_A,   3'd1}: /* w <- [pc]; inc pc */              begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = W; end
            {LD_NN_A,   3'd2}: /* [wz] <- a */                      begin s_ab = WZ; s_db = A; t_db = MEM; end
            {LD_NN_A,   3'd3}: /* inc pc; done */                   begin done = 1; idu = INC; s_ab = PC; wr_pc = 1; end

            {ALU_A_N,   3'd0}: /* z <- [pc]; inc pc */              begin s_ab = PC; idu = INC; wr_pc = 1; s_db = MEM; t_db = Z; end
            {ALU_A_N,   3'd1}: /* a <- alu(a, z); inc pc; done */   begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = Z; t_db = A; use_alu = 1; end

            {ALU_A_HL,  3'd0}: /* z <- [hl] */                      begin s_ab = HL; s_db = MEM; t_db = Z; end
            {ALU_A_HL,  3'd1}: /* a <- alu(a, z); inc pc; done */   begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = Z; t_db = A; use_alu = 1; end

            {ALU_A_R,   3'd0}: /* a <- alu(a, r); inc pc; done */   begin done = 1; s_ab = PC; idu = INC; wr_pc = 1; s_db = r8_src; t_db = A; use_alu = 1; end
        endcase
    end
endmodule