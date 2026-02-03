`include "beamformer_defines.vh"

module beamformer_top #(
    parameter NUM_LANES         = `NUM_LANES,
    parameter NUM_CH            = `NUM_CH_PER_LANE,
    parameter IQ_WIDTH          = `IQ_DATA_WIDTH,
    parameter COEFF_WIDTH       = `COEFF_WIDTH,
    parameter ACC_WIDTH         = `ACC_WIDTH
) (
    // ========== Clock & Reset ==========
    input  wire                         core_clk,
    input  wire                         axi_clk,
    input  wire                         async_rst_n,

    // ========== ADC Inputs (flattened) ==========
    // Total width: NUM_LANES × NUM_CH × 2 × IQ_WIDTH
    input  wire [(NUM_LANES*NUM_CH*2*IQ_WIDTH)-1:0] adc_data_i,
    input  wire [NUM_LANES-1:0]         adc_valid_i,

    // ========== Beamformed Output ==========
    output wire signed [ACC_WIDTH-1:0]  beam_real_o,
    output wire signed [ACC_WIDTH-1:0]  beam_imag_o,
    output wire                         beam_valid_o,
    input  wire                         beam_ready_i,

    // ========== AXI-Lite Coefficient Write Interface ==========
    input  wire [`COEFF_ADDR_WIDTH-1:0] axi_addr,
    input  wire [(2*COEFF_WIDTH)-1:0]   axi_data_w,
    input  wire                         axi_we,
    output wire                         axi_ack,

    // ========== Status/Debug ==========
    output wire [31:0]                  status_core,
    output wire [31:0]                  status_coeff,
    output wire [31:0]                  debug_lane_valid
);

    // ========== RESET SYNCHRONIZERS ==========
    wire core_rst_n = async_rst_n;  // Bypass broken synchronizer
    wire axi_rst_n = async_rst_n;

    // ========== COEFFICIENT MEMORY SIGNALS ==========
    // FIXED: Now receives all 24 channels in parallel (768 bits)
    wire [(NUM_CH*2*COEFF_WIDTH)-1:0] core_coeff_data;
    wire [31:0] core_sample_count;
    
    // Generate sample valid pulse from any lane valid
    wire core_sample_valid;
    assign core_sample_valid = |adc_valid_i;
    
    // REMOVED: coeff_addr_counter - no longer needed!
    // Coefficient buffer now outputs all channels in parallel

    // ========== COEFFICIENT DOUBLE BUFFER ==========
    coeff_double_buffer #(
        .NUM_CH(NUM_CH),
        .COEFF_WIDTH(COEFF_WIDTH),
        .ADDR_WIDTH(`COEFF_ADDR_WIDTH),
        .UPDATE_PERIOD(`WEIGHT_UPDATE_PERIOD)
    ) u_coeff_buffer (
        .core_clk(core_clk),
        .core_rst_n(core_rst_n),
        .core_sample_valid(core_sample_valid),
        // REMOVED: .core_coeff_addr(coeff_addr_counter),  // No longer needed!
        .core_coeff_data(core_coeff_data),  // Now outputs all 24 channels (768 bits)
        .core_sample_count(core_sample_count),
        
        .axi_clk(axi_clk),
        .axi_rst_n(axi_rst_n),
        .axi_addr(axi_addr),
        .axi_data_w(axi_data_w),
        .axi_we(axi_we),
        .axi_ack(axi_ack),
        
        .status_reg(status_coeff)
    );

    // ========== BEAMFORMER CORE ==========
    beamformer_core #(
        .NUM_LANES(NUM_LANES),
        .NUM_CH_PER_LANE(NUM_CH),
        .IQ_WIDTH(IQ_WIDTH),
        .COEFF_WIDTH(COEFF_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) u_core (
        .core_clk(core_clk),
        .core_rst_n(core_rst_n),
        
        .adc_data_i(adc_data_i),
        .adc_valid_i(adc_valid_i),
        
        .coeff_data_i(core_coeff_data),  // Now properly receives all 768 bits
        .coeff_valid_i(1'b1),  // Coefficients always valid in this design
        
        .beam_real_o(beam_real_o),
        .beam_imag_o(beam_imag_o),
        .beam_valid_o(beam_valid_o),
        .beam_ready_i(beam_ready_i),
        
        .status_reg(status_core),
        .debug_lane_valid(debug_lane_valid)
    );

endmodule