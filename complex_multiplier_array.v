`timescale 1ns/1ps
`include "beamformer_defines.vh"

module complex_multiplier_array #(
    parameter NUM_CH       = `NUM_CH_PER_LANE,
    parameter DATA_WIDTH   = `IQ_DATA_WIDTH,
    parameter COEFF_WIDTH  = `COEFF_WIDTH,
    parameter OUT_WIDTH    = `MULT_OUT_WIDTH
) (
    input  wire clk,
    input  wire rst_n,
    
    // Packed input buses - NUM_CH channels × DATA_WIDTH bits
    input  wire [(NUM_CH*DATA_WIDTH)-1:0] i_sample_packed,
    input  wire [(NUM_CH*DATA_WIDTH)-1:0] q_sample_packed,
    input  wire [(NUM_CH*COEFF_WIDTH)-1:0] coeff_i_packed,
    input  wire [(NUM_CH*COEFF_WIDTH)-1:0] coeff_q_packed,
    input  wire inputs_valid,
    
    // Packed output - NUM_CH channels × (2*OUT_WIDTH) bits
    // Each channel: {imag[OUT_WIDTH-1:0], real[OUT_WIDTH-1:0]}
    output wire [(NUM_CH*2*OUT_WIDTH)-1:0] mult_out_packed,
    output wire mult_valid
);

    // ========== GENERATE MULTIPLIERS FOR EACH CHANNEL ==========
    genvar ch;
    generate
        for (ch = 0; ch < NUM_CH; ch = ch + 1) begin : gen_mult_ch
            // Unpack inputs for this channel
            wire signed [DATA_WIDTH-1:0]  i_in  = i_sample_packed[(ch+1)*DATA_WIDTH-1 : ch*DATA_WIDTH];
            wire signed [DATA_WIDTH-1:0]  q_in  = q_sample_packed[(ch+1)*DATA_WIDTH-1 : ch*DATA_WIDTH];
            wire signed [COEFF_WIDTH-1:0] c_i   = coeff_i_packed[(ch+1)*COEFF_WIDTH-1 : ch*COEFF_WIDTH];
            wire signed [COEFF_WIDTH-1:0] c_q   = coeff_q_packed[(ch+1)*COEFF_WIDTH-1 : ch*COEFF_WIDTH];
            
            wire signed [OUT_WIDTH-1:0] real_out, imag_out;
            
            // Instantiate complex multiplier
            complex_multiplier #(
                .DATA_WIDTH(DATA_WIDTH),
                .COEFF_WIDTH(COEFF_WIDTH),
                .OUT_WIDTH(OUT_WIDTH)
            ) u_mult (
                .clk(clk),
                .rst_n(rst_n),
                .i_in(i_in),
                .q_in(q_in),
                .coeff_real(c_i),
                .coeff_imag(c_q),
                .real_out(real_out),
                .imag_out(imag_out)
            );
            
            // Pack output: {imag, real} for each channel
            assign mult_out_packed[(ch*2*OUT_WIDTH) + OUT_WIDTH-1 : (ch*2*OUT_WIDTH)] = real_out;
            assign mult_out_packed[(ch*2*OUT_WIDTH) + (2*OUT_WIDTH)-1 : (ch*2*OUT_WIDTH) + OUT_WIDTH] = imag_out;
        end
    endgenerate
    
    // ========== VALID SIGNAL PIPELINE ==========
    // Complex multiplier has 2-stage pipeline
    reg [1:0] valid_pipe;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid_pipe <= 2'b00;
        else
            valid_pipe <= {valid_pipe[0], inputs_valid};
    end
    
    assign mult_valid = valid_pipe[1];

endmodule
