module ram #(parameter WORDS = 16'd512, parameter WIDTH = 8'd8) (
    input wire clk,
    input wire [D-1:0] addr,
    input wire write,
    input wire [W:0] d_in,
    output reg [W:0] d_out
);
    localparam D = $clog2(WORDS);
    localparam W = WIDTH-1;

    reg [W:0] mem[0:WORDS-1];
    always_ff @(posedge clk) begin
        d_out <= mem[addr];
        if (write) mem[addr] <= d_in;
    end
endmodule

module ram_8192words_8bit (
    input wire clk,
    input wire [12:0] addr,
    input wire write,
    input wire [7:0] d_in,
    output reg [7:0] d_out
);
    reg [7:0] mem[0:8191];
    always_ff @(posedge clk) begin
        d_out <= mem[addr];
        if (write) mem[addr] <= d_in;
    end
endmodule

module ram_80words_16bit (
    input wire clk,
    input wire [6:0] addr,
    input wire write,
    input wire [15:0] d_in,
    output reg [15:0] d_out
);
    reg [15:0] mem[0:79];
    always_ff @(posedge clk) begin
        d_out <= mem[addr];
        if (write) mem[addr] <= d_in;
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
    always_ff @(posedge clk) if (ce) begin
        d_out <= mem[addr];
        if (write) mem[addr] <= d_in;
    end
endmodule