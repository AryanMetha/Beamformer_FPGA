`include "beamformer_defines.vh"

module beamformer_core #(
    parameter NUM_LANES         = `NUM_LANES,
    parameter NUM_CH_PER_LANE   = `NUM_CH_PER_LANE,
    parameter IQ_WIDTH          = `IQ_DATA_WIDTH,
    parameter COEFF_WIDTH       = `COEFF_WIDTH,
    parameter ACC_WIDTH         = `ACC_WIDTH
) (
    input  wire                         core_clk,
    input  wire                         core_rst_n,

    // ADC inputs - flattened packed array
    input  wire [(NUM_LANES*NUM_CH_PER_LANE*2*IQ_WIDTH)-1:0] adc_data_i,
    input  wire [NUM_LANES-1:0]         adc_valid_i,

    // Coefficient memory interface
    input  wire [(NUM_CH_PER_LANE*2*COEFF_WIDTH)-1:0] coeff_data_i,  // From double buffer
    input  wire                         coeff_valid_i,

    // Beamformed output
    output reg signed [ACC_WIDTH-1:0]   beam_real_o,
    output reg signed [ACC_WIDTH-1:0]   beam_imag_o,
    output reg                          beam_valid_o,
    input  wire                         beam_ready_i,

    // Status & debug
    output wire [31:0]                  status_reg,
    output wire [31:0]                  debug_lane_valid
);

    // ========== LANE OUTPUTS ==========
    wire signed [ACC_WIDTH-1:0] lane_sum_real [0:NUM_LANES-1];
    wire signed [ACC_WIDTH-1:0] lane_sum_imag [0:NUM_LANES-1];
    wire [NUM_LANES-1:0] lane_valid;

    // ========== UNPACK COEFFICIENTS ==========
    // Duplicate coefficient data for all lanes (same coefficients for all lanes)
    wire [(NUM_CH_PER_LANE*COEFF_WIDTH)-1:0] coeff_i_packed;
    wire [(NUM_CH_PER_LANE*COEFF_WIDTH)-1:0] coeff_q_packed;
    
    genvar coeff_idx;
    generate
        for (coeff_idx = 0; coeff_idx < NUM_CH_PER_LANE; coeff_idx = coeff_idx + 1) begin : gen_coeff_unpack
            // Unpack {Q, I} pairs from coefficient data
            assign coeff_i_packed[(coeff_idx+1)*COEFF_WIDTH-1 : coeff_idx*COEFF_WIDTH] = 
                   coeff_data_i[(coeff_idx*2*COEFF_WIDTH) + COEFF_WIDTH-1 : coeff_idx*2*COEFF_WIDTH];
            assign coeff_q_packed[(coeff_idx+1)*COEFF_WIDTH-1 : coeff_idx*COEFF_WIDTH] = 
                   coeff_data_i[(coeff_idx*2*COEFF_WIDTH) + (2*COEFF_WIDTH)-1 : (coeff_idx*2*COEFF_WIDTH) + COEFF_WIDTH];
        end
    endgenerate
    
    // ========== LANE PROCESSOR ARRAY ==========
    genvar lane_idx;
    generate
        for (lane_idx = 0; lane_idx < NUM_LANES; lane_idx = lane_idx + 1) begin : gen_lanes
            // Extract ADC data for this lane
            wire [(NUM_CH_PER_LANE*2*IQ_WIDTH)-1:0] lane_adc_data;
            assign lane_adc_data = adc_data_i[(lane_idx+1)*NUM_CH_PER_LANE*2*IQ_WIDTH-1 : 
                                              lane_idx*NUM_CH_PER_LANE*2*IQ_WIDTH];
            
            lane_processor #(
                .NUM_CH(NUM_CH_PER_LANE),
                .IQ_WIDTH(IQ_WIDTH),
                .COEFF_WIDTH(COEFF_WIDTH),
                .MULT_WIDTH(`MULT_OUT_WIDTH),
                .ACC_WIDTH(ACC_WIDTH)
            ) u_lane_proc (
                .clk(core_clk),
                .rst_n(core_rst_n),
                .adc_data_packed(lane_adc_data),
                .adc_valid(adc_valid_i[lane_idx]),
                .coeff_i_packed(coeff_i_packed),
                .coeff_q_packed(coeff_q_packed),
                .lane_sum_real(lane_sum_real[lane_idx]),
                .lane_sum_imag(lane_sum_imag[lane_idx]),
                .lane_valid(lane_valid[lane_idx])
            );
        end
    endgenerate

    // ========== INTER-LANE ACCUMULATOR ==========
    // Sum outputs from all lanes
    reg signed [ACC_WIDTH-1:0] total_sum_real;
    reg signed [ACC_WIDTH-1:0] total_sum_imag;
    reg total_sum_valid;
    
    integer lane_sum_idx;
    always @(*) begin
        total_sum_real = {ACC_WIDTH{1'b0}};
        total_sum_imag = {ACC_WIDTH{1'b0}};
        
        for (lane_sum_idx = 0; lane_sum_idx < NUM_LANES; lane_sum_idx = lane_sum_idx + 1) begin
            total_sum_real = total_sum_real + lane_sum_real[lane_sum_idx];
            total_sum_imag = total_sum_imag + lane_sum_imag[lane_sum_idx];
        end
    end
    
    always @(*) begin
        total_sum_valid = &lane_valid;  // All lanes must be valid
    end

    // ========== OUTPUT PIPELINE WITH BACKPRESSURE ==========
    reg output_stall;
    
    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            beam_real_o  <= {ACC_WIDTH{1'b0}};
            beam_imag_o  <= {ACC_WIDTH{1'b0}};
            beam_valid_o <= 1'b0;
            output_stall <= 1'b0;
        end else begin
            if (!output_stall || beam_ready_i) begin
                // Update output when not stalled or consumer is ready
                beam_real_o  <= total_sum_real;
                beam_imag_o  <= total_sum_imag;
                beam_valid_o <= total_sum_valid;
                output_stall <= total_sum_valid && !beam_ready_i;
            end
        end
    end
    
    // ========== STATUS REGISTERS ==========
    reg [15:0] valid_count;
    reg overflow_flag;
    
    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            valid_count <= 16'd0;
            overflow_flag <= 1'b0;
        end else begin
            if (beam_valid_o && beam_ready_i)
                valid_count <= valid_count + 1;
            
            // Check for overflow (simplified - check MSBs)
            if (total_sum_real[ACC_WIDTH-1] != total_sum_real[ACC_WIDTH-2])
                overflow_flag <= 1'b1;
            if (total_sum_imag[ACC_WIDTH-1] != total_sum_imag[ACC_WIDTH-2])
                overflow_flag <= 1'b1;
        end
    end
    
    assign status_reg = {15'h0000, overflow_flag, valid_count};
    assign debug_lane_valid = {{(32-NUM_LANES){1'b0}}, lane_valid};

endmodule
