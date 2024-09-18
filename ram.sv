module ram #(parameter WORDS = 512, parameter WIDTH = 8) (
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