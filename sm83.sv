typedef enum logic [3:0] {
    DB_B, DB_C, DB_D, DB_E, DB_H, DB_L, DB_M, DB_A, DB_Z, DB_W, DB_SPH, DB_SPL, DB_PCH, DB_PCL
} s_db_t;

typedef enum logic [2:0] {
    AB_BC, AB_DE, AB_HL, AB_SP, AB_WZ, AB_PC
} s_ab_t;

typedef enum logic [1:0] {
    IDU_INC,
    IDU_DEC,
    IDU_NONE
} idu_op_t;

typedef enum logic [3:0] {
    ALU_NONE
} alu_op_t;

module sm83 (
    input wire clk,
    input wire [7:0] d_in,
    output wire [15:0] addr,
    output reg [7:0] d_out,
    output wire write
);
    // ~ Registers ~
    reg [7:0] r_b, r_c, r_d, r_e, r_h, r_l, r_a, r_z, r_w, r_sph, r_spl, r_pch, r_pcl;

    // ~ Buses ~
    // Input data bus. Feeds ALU.
    s_db_t s_dbi;
    logic [7:0] b_dbi;
    always @(*)
        case (s_dbi)
            DB_B: b_dbi = r_b;
            DB_C: b_dbi = r_c;
            DB_D: b_dbi = r_d;
            DB_E: b_dbi = r_e;
            DB_H: b_dbi = r_h;
            DB_L: b_dbi = r_l;
            DB_M: b_dbi = d_in;
            DB_A: b_dbi = r_a;
            DB_Z: b_dbi = r_z;
            DB_W: b_dbi = r_w;
            DB_SPH: b_dbi = r_sph;
            DB_SPL: b_dbi = r_spl;
            DB_PCH: b_dbi = r_pch;
            DB_PCL: b_dbi = r_pcl;
        endcase

    // Output data bus. Feeds reg file + memory.
    s_db_t s_dbo;
    logic [7:0] b_dbo;
    always @(posedge clk)
        case (s_dbo)
            DB_B: r_b <= b_dbo;
            DB_C: r_c <= b_dbo;
            DB_D: r_d <= b_dbo;
            DB_E: r_e <= b_dbo;
            DB_H: r_h <= b_dbo;
            DB_L: r_l <= b_dbo;
            DB_M: d_out <= b_dbo;
            DB_A: r_a <= b_dbo;
            DB_Z: r_z <= b_dbo;
            DB_W: r_w <= b_dbo;
            DB_SPH: r_sph <= b_dbo;
            DB_SPL: r_spl <= b_dbo;
            DB_PCH: r_pch <= b_dbo;
            DB_PCL: r_pcl <= b_dbo;
        endcase

    // Input data bus. Feeds IDU.
    s_ab_t s_abi;
    logic [15:0] b_abi;
    always @(*)
        case (s_abi)
            AB_BC: b_abi = {r_b, r_c};
            AB_DE: b_abi = {r_d, r_e};
            AB_HL: b_abi = {r_h, r_l};
            AB_SP: b_abi = {r_sph, r_spl};
            AB_PC: b_abi = {r_pch, r_pcl};
            AB_WZ: b_abi = {r_w, r_z};
        endcase
    
    // Output data bus.
    s_ab_t t_abo;
    logic [15:0] b_abo;
    always @(posedge clk)
        case (t_abo)
            AB_BC: {r_b, r_c} <= b_abo;
            AB_DE: {r_d, r_e} <= b_abo;
            AB_HL: {r_h, r_l} <= b_abo;
            AB_SP: {r_sph, r_spl} <= b_abo;
            AB_PC: {r_pch, r_pcl} <= b_abo;
            AB_WZ: {r_w, r_z} <= b_abo;
        endcase

endmodule

module r8_decoder (
    input wire [2:0] r8,
    output reg is_hl,
    output s_db_t s_db
);
    always @(*) begin
        is_hl = 0;
        case (r8)
            3'b000: s_db = DB_B;
            3'b001: s_db = DB_C;
            3'b010: s_db = DB_D;
            3'b011: s_db = DB_E;
            3'b100: s_db = DB_H;
            3'b101: s_db = DB_L;
            3'b110: begin s_db = DB_M; is_hl = 1; end
            3'b111: s_db = DB_A;
        endcase
    end
endmodule

typedef enum logic [7:0] {
    LD_R_R = 8'b01xxxxxx,
    LD_R_N = 8'b00xxx110
} opcode_t;

type enum logic [1:0] {
    ALU_R8_R8,
    ALU_R8_IMM8
} addr_mode_t;

typedef enum logic [1:0] {
    SEQ_READ_IMM8,
    SEQ_READ_HL,
    SEQ_WRITE_HL,
    SEQ_EXEC,
    SEQ_IDLE
} seq_state_t;

module sequencer (
    input addr_mode_t ir,
    input seq_state_t curr_s,
    output seq_state_t next_s
);
    r8_decoder r8_x (.r8(ir[5:3]));
    r8_decoder r8_y (.r8(ir[2:0]));

    always @(*)
        case (addr_mode_t)
            ALU_R8_R8:
                case (curr_s)
                    SEQ_IDLE: next_s = r8_y.is_hl ? SEQ_READ_HL : SEQ_EXEC;
                    SEQ_READ_HL: next_s = SEQ_EXEC;
                    SEQ_EXEC: next_s = r8_x.is_hl ? SEQ_WRITE_HL : SEQ_IDLE;
                    SEQ_WRITE_HL: next_s = SEQ_IDLE;
                endcase
            ALU_R8_IMM8:
                case (curr_s)
                    SEQ_IDLE: next_s = SEQ_READ_IMM8;
                    SEQ_READ_IMM8: next_s = SEQ_EXEC;
                    SEQ_EXEC: next_s = r8_x.is_hl ? SEQ_WRITE_HL : SEQ_IDLE;
                    SEQ_WRITE_HL: next_s = SEQ_IDLE;
                endcase
        endcase
endmodule

module decoder (
    input opcode_t ir,
    input seq_state_t curr_s,
    output seq_state_t next_s,
    output alu_op_t alu_op,
    output s_db_t s_dbi,
    output s_db_t t_dbo,
    output s_ab_t s_abi,
    output s_ab_t t_abo,
    output idu_op_t idu_op
);
    r8_decoder r8_5_3 (.r8(ir[5:3]));
    r8_decoder r8_2_0 (.r8(ir[2:0]));

    always @(*)
        casez ({ir, curr_s})
            // LD r, r'
            {LD_R_R, M2}: begin
                // r <- r'
                s_dbi = r8_2_0.s_db;
                t_dbo = r8_5_3.s_db;
                alu_op = ALU_NONE;
                // PC <- PC + 1
                s_abi = AB_PC;
                t_abo = AB_PC;
                idu_op = IDU_INC;
                next_s = M3;
                next_s = M1;
            end
            // LD r, n
            {LD_R_N, M2}: begin
                // Z <- M
                s_dbi = DB_M;
                t_dbo = DB_Z;
                alu_op = ALU_NONE;
                // PC <- PC + 1
                s_abi = AB_PC;
                t_abo = AB_PC;
                idu_op = IDU_INC;
                next_s = M3;
            end
            {LD_R_N, M3}: begin
                // r <- Z
                s_dbi = DB_Z;
                t_dbo = r8_5_3.s_db;
                alu_op = ALU_NONE;
                // PC <- PC + 1
                s_abi = AB_PC;
                t_abo = AB_PC;
                idu_op = IDU_INC;
                next_s = M1;
            end
        endcase
endmodule