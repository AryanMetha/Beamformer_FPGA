`timescale 1ns/1ps
`include "beamformer_defines.vh"

module adc_interface #(
    parameter NUM_LANES    = `NUM_LANES,
    parameter NUM_CH       = `NUM_CH_PER_LANE,
    parameter IQ_WIDTH     = `IQ_DATA_WIDTH,
    parameter SAMPLE_RATE  = 500_000_000  // 500 MSPS
) (
    input  wire                                           adc_clk,
    input  wire                                           rst_n,
    
    output wire [(NUM_LANES*NUM_CH*2*IQ_WIDTH)-1:0]      adc_data_out,
    output wire [NUM_LANES-1:0]                           adc_valid_out
);

    // ========== DDS PHASE ACCUMULATORS ==========
    reg [31:0] phase_acc [0:NUM_LANES-1][0:NUM_CH-1];

    // Tuning word for test signals
    // Frequency = tuning_word × fs / 2^32
    // Example: 10 MHz @ 500 MSPS → tuning_word = 10M/500M × 2^32 ≈ 0x08000000
    localparam [31:0] TUNING_WORD = 32'h08000000;

    integer lane, ch;

    // Initialize phase accumulators
    initial begin
        for (lane = 0; lane < NUM_LANES; lane = lane + 1) begin
            for (ch = 0; ch < NUM_CH; ch = ch + 1) begin
                phase_acc[lane][ch] = 32'h00000000;
            end
        end
    end

    // Phase accumulation
    always @(posedge adc_clk or negedge rst_n) begin
        if (!rst_n) begin
            for (lane = 0; lane < NUM_LANES; lane = lane + 1) begin
                for (ch = 0; ch < NUM_CH; ch = ch + 1) begin
                    phase_acc[lane][ch] <= 32'h00000000;
                end
            end
        end else begin
            for (lane = 0; lane < NUM_LANES; lane = lane + 1) begin
                for (ch = 0; ch < NUM_CH; ch = ch + 1) begin
                    phase_acc[lane][ch] <= phase_acc[lane][ch] + TUNING_WORD;
                end
            end
        end
    end

    // ========== SINE/COSINE LUT FUNCTIONS ==========
    function signed [IQ_WIDTH-1:0] sine_lut;
        input [7:0] idx;
        reg [4:0] phase_idx;
        reg sign;
        begin
            sign = idx[7];  // Sign bit
            phase_idx = idx[6:2];  // 5-bit index (0-31)
            
            case (phase_idx)
                5'h00: sine_lut = 16'h0000;
                5'h01: sine_lut = 16'h0c8c;
                5'h02: sine_lut = 16'h1918;
                5'h03: sine_lut = 16'h2528;
                5'h04: sine_lut = 16'h30fb;
                5'h05: sine_lut = 16'h3c56;
                5'h06: sine_lut = 16'h471c;
                5'h07: sine_lut = 16'h5133;
                5'h08: sine_lut = 16'h5a82;
                5'h09: sine_lut = 16'h62f2;
                5'h0a: sine_lut = 16'h6a6d;
                5'h0b: sine_lut = 16'h70e2;
                5'h0c: sine_lut = 16'h7641;
                5'h0d: sine_lut = 16'h7a7d;
                5'h0e: sine_lut = 16'h7d8a;
                5'h0f: sine_lut = 16'h7f61;
                5'h10: sine_lut = 16'h7fff;
                5'h11: sine_lut = 16'h7f61;
                5'h12: sine_lut = 16'h7d8a;
                5'h13: sine_lut = 16'h7a7d;
                5'h14: sine_lut = 16'h7641;
                5'h15: sine_lut = 16'h70e2;
                5'h16: sine_lut = 16'h6a6d;
                5'h17: sine_lut = 16'h62f2;
                5'h18: sine_lut = 16'h5a82;
                5'h19: sine_lut = 16'h5133;
                5'h1a: sine_lut = 16'h471c;
                5'h1b: sine_lut = 16'h3c56;
                5'h1c: sine_lut = 16'h30fb;
                5'h1d: sine_lut = 16'h2528;
                5'h1e: sine_lut = 16'h1918;
                5'h1f: sine_lut = 16'h0c8c;
                default: sine_lut = 16'h0000;
            endcase
            
            // Apply sign
            if (sign)
                sine_lut = -sine_lut;
        end
    endfunction

    function signed [IQ_WIDTH-1:0] cosine_lut;
        input [7:0] idx;
        begin
            cosine_lut = sine_lut(idx + 8'h40);  // cos(x) = sin(x + π/2)
        end
    endfunction

    // ========== GENERATE IQ SAMPLES ==========
    genvar lane_idx, ch_idx;
    generate
        for (lane_idx = 0; lane_idx < NUM_LANES; lane_idx = lane_idx + 1) begin : gen_lane_adc
            for (ch_idx = 0; ch_idx < NUM_CH; ch_idx = ch_idx + 1) begin : gen_ch_adc
                wire [31:0] phase;
                wire [7:0] phase_idx;
                wire signed [IQ_WIDTH-1:0] i_sample;
                wire signed [IQ_WIDTH-1:0] q_sample;

                // Get phase from accumulator
                assign phase = phase_acc[lane_idx][ch_idx];
                assign phase_idx = phase[31:24];  // Top 8 bits as LUT index

                // Generate I and Q samples
                assign i_sample = sine_lut(phase_idx);
                assign q_sample = cosine_lut(phase_idx);

                // Pack into output bus
                // Format per lane: {ch23_Q, ch23_I, ..., ch0_Q, ch0_I}
                localparam BASE_BIT = (lane_idx*NUM_CH*2*IQ_WIDTH) + (ch_idx*2*IQ_WIDTH);
                
                assign adc_data_out[BASE_BIT + IQ_WIDTH-1 : BASE_BIT] = i_sample;
                assign adc_data_out[BASE_BIT + (2*IQ_WIDTH)-1 : BASE_BIT + IQ_WIDTH] = q_sample;
            end
        end
    endgenerate

    // Always valid in simulation/test mode
    assign adc_valid_out = {NUM_LANES{1'b1}};

endmodule
