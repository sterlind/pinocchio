typedef enum logic [3:0] {
    DB_B, DB_C, DB_D, DB_E, DB_H, DB_L, DB_M, DB_A, DB_Z, DB_W, DB_SPH, DB_SPL, DB_PCH, DB_PCL
} s_db_t;

typedef enum logic [2:0] {
    ADDR_BC, ADDR_DE, ADDR_HL, ADDR_SP, ADDR_WZ, ADDR_PC
} s_addr_t;

module sm83 (
    input wire clk,
    input wire [7:0] d_in,
    output wire [15:0] addr,
    output wire [7:0] d_out,
    output wire write
);
    // ~ Registers ~
    reg [7:0] r_b, r_c, r_d, r_e, r_h, r_l, r_a, r_z, r_w, r_sph, r_spl, r_pch, r_pcl;

    // ~ Buses ~
    // Data bus. Feeds ALU.
    s_db_t s_db;
    logic [7:0] b_db;
    always @(posedge clk)
        case (s_db)
            DB_B: b_db = r_b;
            DB_C: b_db = r_c;
            DB_D: b_db = r_d;
            DB_E: b_db = r_e;
            DB_H: b_db = r_h;
            DB_L: b_db = r_l;
            DB_M: b_db = d_in;
            DB_A: b_db = r_a;
            DB_Z: b_db = r_z;
            DB_W: b_db = r_w;
            DB_SPH: b_db = r_sph;
            DB_SPL: b_db = r_spl;
            DB_PCH: b_db = r_pch;
            DB_PCL: b_db = r_pcl;
        endcase

    // Addr bus. Feeds IDU.
    s_addr_t s_addr;
    logic [15:0] b_addr;
    always @(posedge clk)
        case (s_addr)
            ADDR_BC: b_addr = {r_b, r_c};
            ADDR_DE: b_addr = {r_d, r_e};
            ADDR_HL: b_addr = {r_h, r_l};
            ADDR_SP: b_addr = {r_sph, r_spl};
            ADDR_PC: b_addr = {r_pch, r_pcl};
            ADDR_WZ: b_addr = {r_w, r_z};
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

module decoder (
    input wire [7:0] ir,
    input logic [1:0] state,
    output s_db_t s_db,
    output s_addr_t s_addr
);
    r8_decoder r8_5_3 (.r8(ir[5:3]));
    r8_decoder r8_2_0 (.r8(ir[2:0]));

    always @(*)
        casez (ir)
            // LD r, n
            8'b00xxx110: begin
                s_db = r8_5_3.s_db;
            end
        endcase
endmodule