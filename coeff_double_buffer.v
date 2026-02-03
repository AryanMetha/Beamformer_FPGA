`include "beamformer_defines.vh"

module coeff_double_buffer #(
    parameter NUM_CH           = `NUM_CH_PER_LANE,
    parameter COEFF_WIDTH      = `COEFF_WIDTH,
    parameter ADDR_WIDTH       = `COEFF_ADDR_WIDTH,
    parameter UPDATE_PERIOD    = `WEIGHT_UPDATE_PERIOD
) (
    // ========== Core Domain (250/300 MHz) ==========
    input  wire                         core_clk,
    input  wire                         core_rst_n,
    input  wire                         core_sample_valid,  // Pulse when new sample arrives

    // FIXED: Read interface now outputs ALL channels in parallel
    output wire [(NUM_CH*2*COEFF_WIDTH)-1:0] core_coeff_data,  // All 24 channels: {ch23_Q, ch23_I, ..., ch0_Q, ch0_I}

    // Sample counter output
    output wire [31:0]                  core_sample_count,

    // ========== AXI Domain (100 MHz, slower) ==========
    input  wire                         axi_clk,
    input  wire                         axi_rst_n,

    // Write interface from CPU
    input  wire [ADDR_WIDTH-1:0]        axi_addr,
    input  wire [(2*COEFF_WIDTH)-1:0]   axi_data_w,     // {coeff_Q, coeff_I}
    input  wire                         axi_we,
    output wire                         axi_ack,

    // Status register
    output wire [31:0]                  status_reg
);

    // ========== Core Domain: Sample Counter & Bank Selection ==========
    reg [31:0] core_sample_cnt;
    reg core_active_bank;  // 0 = Bank A active, 1 = Bank B active

    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            core_sample_cnt <= 32'd0;
            core_active_bank <= 1'b0;
        end else if (core_sample_valid) begin
            if (core_sample_cnt >= UPDATE_PERIOD - 1) begin
                core_sample_cnt <= 32'd0;
                core_active_bank <= ~core_active_bank;  // Toggle bank
            end else begin
                core_sample_cnt <= core_sample_cnt + 1;
            end
        end
    end

    // ========== Dual-Port Coefficient Memory Banks ==========
    reg [(2*COEFF_WIDTH)-1:0] bank_a [0:(2**ADDR_WIDTH)-1];
    reg [(2*COEFF_WIDTH)-1:0] bank_b [0:(2**ADDR_WIDTH)-1];

    // Initialize to unity coefficients (1.0 + j0.0)
    integer init_i;
    initial begin
        for (init_i = 0; init_i < (2**ADDR_WIDTH); init_i = init_i + 1) begin
            bank_a[init_i] = {16'h0000, `Q15_ONE};  // {Q=0, I=1.0}
            bank_b[init_i] = {16'h0000, `Q15_ONE};
        end
    end

    // ========== Core Read Path - FIXED: Output all channels in parallel ==========
    // Read ALL channels from active bank simultaneously
    genvar rd_ch;
    generate
        for (rd_ch = 0; rd_ch < NUM_CH; rd_ch = rd_ch + 1) begin : gen_coeff_read
            wire [(2*COEFF_WIDTH)-1:0] channel_coeff;
            
            // Select from active bank
            assign channel_coeff = (core_active_bank == 1'b0) ? 
                                   bank_a[rd_ch] : 
                                   bank_b[rd_ch];
            
            // Pack into output bus: {ch23, ch22, ..., ch1, ch0}
            assign core_coeff_data[(rd_ch*2*COEFF_WIDTH) + (2*COEFF_WIDTH)-1 : rd_ch*2*COEFF_WIDTH] = channel_coeff;
        end
    endgenerate
    
    assign core_sample_count = core_sample_cnt;

    // ========== AXI Write Path (Clock Domain Crossing) ==========
    // Synchronize core_active_bank to AXI domain (2-FF synchronizer)
    reg [1:0] core_active_bank_sync;
    reg axi_inactive_bank;

    always @(posedge axi_clk or negedge axi_rst_n) begin
        if (!axi_rst_n) begin
            core_active_bank_sync <= 2'b00;
            axi_inactive_bank <= 1'b1;  // Default: write to Bank B
        end else begin
            core_active_bank_sync <= {core_active_bank_sync[0], core_active_bank};
            axi_inactive_bank <= ~core_active_bank_sync[1];  // Inactive = NOT(active)
        end
    end

    // Write counter
    reg [31:0] axi_wr_count;

    always @(posedge axi_clk or negedge axi_rst_n) begin
        if (!axi_rst_n) begin
            axi_wr_count <= 32'd0;
        end else if (axi_we && axi_ack) begin
            axi_wr_count <= axi_wr_count + 1;
        end
    end

    // Write to inactive bank - FIXED: Added proper reset
    always @(posedge axi_clk or negedge axi_rst_n) begin
        if (!axi_rst_n) begin
            // Reset - no action needed, memories initialized in initial block
        end else if (axi_we && axi_ack) begin
            if (axi_inactive_bank == 1'b0)
                bank_a[axi_addr] <= axi_data_w;
            else
                bank_b[axi_addr] <= axi_data_w;
        end
    end

    // Always acknowledge writes (simple version)
    assign axi_ack = axi_we;

    // ========== Status Register ==========
    // [31:24] = active_bank
    // [23:16] = unused
    // [15:0]  = sample_count[15:0]
    assign status_reg = {7'h00, core_active_bank, 8'h00, core_sample_cnt[15:0]};

endmodule