module debounce
#(
    parameter COUNTER_TICKS = 675_000 // 25ms clocked @ 27MHz
)
(
    input wire clk,
    input wire d,
    output reg q,
    output reg q_fall
);
    initial begin q = 0; q_fall = 0; end

    wire d_sync;
    sync sync(clk, d, d_sync);

    localparam COUNTER_BITS = $clog2(COUNTER_TICKS);
    reg [COUNTER_BITS-1:0] counter = 0;

    always @(posedge clk) begin
        counter <= 0;
        q_fall <= 0;
        if (counter == COUNTER_TICKS) begin
            q <= d_sync;
            q_fall <= q & ~d_sync;
        end else if (q != d_sync) counter <= counter + 1'b1;
    end
endmodule

module sync(
    input wire clk,
    input wire d,
    output reg q);
    reg d_pipe;
    always @(posedge clk)
        {q, d_pipe} <= {d_pipe, d};
endmodule