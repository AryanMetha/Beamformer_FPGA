module reset_synchronizer #(
    parameter SYNC_STAGES = 2
) (
    input  wire     async_rst_n,
    input  wire     clk,
    output wire     sync_rst_n
);

    reg [SYNC_STAGES-1:0] sync_chain;

    always @(posedge clk or negedge async_rst_n) begin
        if (!async_rst_n)
            sync_chain <= 0;
        else
            sync_chain <= {sync_chain[SYNC_STAGES-2:0], 1'b1};
    end

    assign sync_rst_n = sync_chain[SYNC_STAGES-1];

endmodule
