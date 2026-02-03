`timescale 1ns/1ps
`include "beamformer_defines.vh"

module lane_processor #(
    parameter NUM_CH       = `NUM_CH_PER_LANE,
    parameter IQ_WIDTH     = `IQ_DATA_WIDTH,
    parameter COEFF_WIDTH  = `COEFF_WIDTH,
    parameter MULT_WIDTH   = `MULT_OUT_WIDTH,
    parameter ACC_WIDTH    = `ACC_WIDTH
) (
    input  wire clk,
    input  wire rst_n,
    
    // ADC Interface - packed: NUM_CH channels × 2 × IQ_WIDTH bits
    // Format: {ch23_Q, ch23_I, ..., ch0_Q, ch0_I}
    input  wire [(NUM_CH*2*IQ_WIDTH)-1:0] adc_data_packed,
    input  wire adc_valid,
    
    // Coefficient Interface - packed: NUM_CH × COEFF_WIDTH bits each
    input  wire [(NUM_CH*COEFF_WIDTH)-1:0] coeff_i_packed,
    input  wire [(NUM_CH*COEFF_WIDTH)-1:0] coeff_q_packed,
    
    // Lane Output - accumulated result
    output wire signed [ACC_WIDTH-1:0] lane_sum_real,
    output wire signed [ACC_WIDTH-1:0] lane_sum_imag,
    output wire lane_valid
);

    // ========== INTERNAL SIGNALS ==========
    wire [(NUM_CH*IQ_WIDTH)-1:0] i_sample_packed;
    wire [(NUM_CH*IQ_WIDTH)-1:0] q_sample_packed;
    wire samples_valid;
    
    wire [(NUM_CH*2*MULT_WIDTH)-1:0] mult_out_packed;
    wire mult_valid;

    // ========== 1. ADC DEMUX ==========
    // Unpack IQ samples from ADC data
    genvar demux_ch;
    generate
        for (demux_ch = 0; demux_ch < NUM_CH; demux_ch = demux_ch + 1) begin : gen_demux
            // ADC packing: {Q[15:0], I[15:0]} per channel
            assign i_sample_packed[(demux_ch+1)*IQ_WIDTH-1 : demux_ch*IQ_WIDTH] = 
                   adc_data_packed[(demux_ch*2*IQ_WIDTH) + IQ_WIDTH-1 : demux_ch*2*IQ_WIDTH];
            assign q_sample_packed[(demux_ch+1)*IQ_WIDTH-1 : demux_ch*IQ_WIDTH] = 
                   adc_data_packed[(demux_ch*2*IQ_WIDTH) + (2*IQ_WIDTH)-1 : (demux_ch*2*IQ_WIDTH) + IQ_WIDTH];
        end
    endgenerate
    
    assign samples_valid = adc_valid;

    // ========== 2. COMPLEX MULTIPLIER ARRAY ==========
    complex_multiplier_array #(
        .NUM_CH(NUM_CH),
        .DATA_WIDTH(IQ_WIDTH),
        .COEFF_WIDTH(COEFF_WIDTH),
        .OUT_WIDTH(MULT_WIDTH)
    ) u_mult_array (
        .clk(clk),
        .rst_n(rst_n),
        .i_sample_packed(i_sample_packed),
        .q_sample_packed(q_sample_packed),
        .coeff_i_packed(coeff_i_packed),
        .coeff_q_packed(coeff_q_packed),
        .inputs_valid(samples_valid),
        .mult_out_packed(mult_out_packed),
        .mult_valid(mult_valid)
    );

    // ========== 3. ACCUMULATOR TREE ==========
    accumulator_tree #(
        .NUM_CH(NUM_CH),
        .IN_WIDTH(MULT_WIDTH),
        .OUT_WIDTH(ACC_WIDTH)
    ) u_acc_tree (
        .clk(clk),
        .rst_n(rst_n),
        .data_in_packed(mult_out_packed),
        .inputs_valid(mult_valid),
        .sum_real(lane_sum_real),
        .sum_imag(lane_sum_imag),
        .sum_valid(lane_valid)
    );

endmodule
