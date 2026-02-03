`timescale 1ns/1ps
`include "beamformer_defines.vh"

module accumulator_tree #(
    parameter NUM_CH    = `NUM_CH_PER_LANE,
    parameter IN_WIDTH  = `MULT_OUT_WIDTH,  // 32-bit from multiplier
    parameter OUT_WIDTH = `ACC_WIDTH        // 48-bit accumulator output
) (
    input  wire clk,
    input  wire rst_n,
    
    // Packed input: NUM_CH channels × 2 × IN_WIDTH bits
    // Format: {ch23_imag, ch23_real, ..., ch0_imag, ch0_real}
    input  wire [(NUM_CH*2*IN_WIDTH)-1:0] data_in_packed,
    input  wire inputs_valid,
    
    // Summed output
    output reg signed [OUT_WIDTH-1:0] sum_real,
    output reg signed [OUT_WIDTH-1:0] sum_imag,
    output reg sum_valid
);

    // ========== UNPACK INPUT CHANNELS ==========
    wire signed [IN_WIDTH-1:0] real_in [0:NUM_CH-1];
    wire signed [IN_WIDTH-1:0] imag_in [0:NUM_CH-1];
    
    genvar unpack_idx;
    generate
        for (unpack_idx = 0; unpack_idx < NUM_CH; unpack_idx = unpack_idx + 1) begin : gen_unpack
            assign real_in[unpack_idx] = data_in_packed[(unpack_idx*2*IN_WIDTH) + IN_WIDTH-1 : (unpack_idx*2*IN_WIDTH)];
            assign imag_in[unpack_idx] = data_in_packed[(unpack_idx*2*IN_WIDTH) + (2*IN_WIDTH)-1 : (unpack_idx*2*IN_WIDTH) + IN_WIDTH];
        end
    endgenerate

    // ========== PIPELINE STAGE REGISTERS ==========
    // Sign-extend from IN_WIDTH (32) to OUT_WIDTH (48) during accumulation
    // Level 1: 24 → 12
    reg signed [OUT_WIDTH-1:0] level1_real [0:11];
    reg signed [OUT_WIDTH-1:0] level1_imag [0:11];
    reg level1_valid;
    
    // Level 2: 12 → 6
    reg signed [OUT_WIDTH-1:0] level2_real [0:5];
    reg signed [OUT_WIDTH-1:0] level2_imag [0:5];
    reg level2_valid;
    
    // Level 3: 6 → 3
    reg signed [OUT_WIDTH-1:0] level3_real [0:2];
    reg signed [OUT_WIDTH-1:0] level3_imag [0:2];
    reg level3_valid;
    
    // Level 4: 3 → 2
    reg signed [OUT_WIDTH-1:0] level4_real [0:1];
    reg signed [OUT_WIDTH-1:0] level4_imag [0:1];
    reg level4_valid;
    
    // Level 5: 2 → 1 (intermediate)
    reg signed [OUT_WIDTH-1:0] level5_real;
    reg signed [OUT_WIDTH-1:0] level5_imag;
    reg level5_valid;
    
    integer i;

    // ========== STAGE 1: 24 → 12 ==========
    // FIXED: Added "or negedge rst_n" to sensitivity list
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level1_valid <= 1'b0;
            for (i = 0; i < 12; i = i + 1) begin
                level1_real[i] <= {OUT_WIDTH{1'b0}};
                level1_imag[i] <= {OUT_WIDTH{1'b0}};
            end
        end else begin
            level1_valid <= inputs_valid;
            
            // Pairwise addition with sign extension
            level1_real[0]  <= $signed(real_in[0])  + $signed(real_in[1]);
            level1_imag[0]  <= $signed(imag_in[0])  + $signed(imag_in[1]);
            level1_real[1]  <= $signed(real_in[2])  + $signed(real_in[3]);
            level1_imag[1]  <= $signed(imag_in[2])  + $signed(imag_in[3]);
            level1_real[2]  <= $signed(real_in[4])  + $signed(real_in[5]);
            level1_imag[2]  <= $signed(imag_in[4])  + $signed(imag_in[5]);
            level1_real[3]  <= $signed(real_in[6])  + $signed(real_in[7]);
            level1_imag[3]  <= $signed(imag_in[6])  + $signed(imag_in[7]);
            level1_real[4]  <= $signed(real_in[8])  + $signed(real_in[9]);
            level1_imag[4]  <= $signed(imag_in[8])  + $signed(imag_in[9]);
            level1_real[5]  <= $signed(real_in[10]) + $signed(real_in[11]);
            level1_imag[5]  <= $signed(imag_in[10]) + $signed(imag_in[11]);
            level1_real[6]  <= $signed(real_in[12]) + $signed(real_in[13]);
            level1_imag[6]  <= $signed(imag_in[12]) + $signed(imag_in[13]);
            level1_real[7]  <= $signed(real_in[14]) + $signed(real_in[15]);
            level1_imag[7]  <= $signed(imag_in[14]) + $signed(imag_in[15]);
            level1_real[8]  <= $signed(real_in[16]) + $signed(real_in[17]);
            level1_imag[8]  <= $signed(imag_in[16]) + $signed(imag_in[17]);
            level1_real[9]  <= $signed(real_in[18]) + $signed(real_in[19]);
            level1_imag[9]  <= $signed(imag_in[18]) + $signed(imag_in[19]);
            level1_real[10] <= $signed(real_in[20]) + $signed(real_in[21]);
            level1_imag[10] <= $signed(imag_in[20]) + $signed(imag_in[21]);
            level1_real[11] <= $signed(real_in[22]) + $signed(real_in[23]);
            level1_imag[11] <= $signed(imag_in[22]) + $signed(imag_in[23]);
        end
    end

    // ========== STAGE 2: 12 → 6 ==========
    // FIXED: Added "or negedge rst_n" to sensitivity list
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level2_valid <= 1'b0;
            for (i = 0; i < 6; i = i + 1) begin
                level2_real[i] <= {OUT_WIDTH{1'b0}};
                level2_imag[i] <= {OUT_WIDTH{1'b0}};
            end
        end else begin
            level2_valid <= level1_valid;
            
            level2_real[0] <= level1_real[0]  + level1_real[1];
            level2_imag[0] <= level1_imag[0]  + level1_imag[1];
            level2_real[1] <= level1_real[2]  + level1_real[3];
            level2_imag[1] <= level1_imag[2]  + level1_imag[3];
            level2_real[2] <= level1_real[4]  + level1_real[5];
            level2_imag[2] <= level1_imag[4]  + level1_imag[5];
            level2_real[3] <= level1_real[6]  + level1_real[7];
            level2_imag[3] <= level1_imag[6]  + level1_imag[7];
            level2_real[4] <= level1_real[8]  + level1_real[9];
            level2_imag[4] <= level1_imag[8]  + level1_imag[9];
            level2_real[5] <= level1_real[10] + level1_real[11];
            level2_imag[5] <= level1_imag[10] + level1_imag[11];
        end
    end

    // ========== STAGE 3: 6 → 3 ==========
    // FIXED: Added "or negedge rst_n" to sensitivity list
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level3_valid <= 1'b0;
            for (i = 0; i < 3; i = i + 1) begin
                level3_real[i] <= {OUT_WIDTH{1'b0}};
                level3_imag[i] <= {OUT_WIDTH{1'b0}};
            end
        end else begin
            level3_valid <= level2_valid;
            
            level3_real[0] <= level2_real[0] + level2_real[1];
            level3_imag[0] <= level2_imag[0] + level2_imag[1];
            level3_real[1] <= level2_real[2] + level2_real[3];
            level3_imag[1] <= level2_imag[2] + level2_imag[3];
            level3_real[2] <= level2_real[4] + level2_real[5];
            level3_imag[2] <= level2_imag[4] + level2_imag[5];
        end
    end

    // ========== STAGE 4: 3 → 2 ==========
    // FIXED: Added "or negedge rst_n" to sensitivity list
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level4_valid <= 1'b0;
            level4_real[0] <= {OUT_WIDTH{1'b0}};
            level4_real[1] <= {OUT_WIDTH{1'b0}};
            level4_imag[0] <= {OUT_WIDTH{1'b0}};
            level4_imag[1] <= {OUT_WIDTH{1'b0}};
        end else begin
            level4_valid <= level3_valid;
            
            level4_real[0] <= level3_real[0] + level3_real[1];
            level4_imag[0] <= level3_imag[0] + level3_imag[1];
            level4_real[1] <= level3_real[2];  // Pass through
            level4_imag[1] <= level3_imag[2];
        end
    end

    // ========== STAGE 5: 2 → 1 ==========
    // FIXED: Added "or negedge rst_n" to sensitivity list
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level5_valid <= 1'b0;
            level5_real <= {OUT_WIDTH{1'b0}};
            level5_imag <= {OUT_WIDTH{1'b0}};
        end else begin
            level5_valid <= level4_valid;
            level5_real <= level4_real[0] + level4_real[1];
            level5_imag <= level4_imag[0] + level4_imag[1];
        end
    end

    // ========== OUTPUT REGISTER ==========
    // FIXED: Added "or negedge rst_n" to sensitivity list
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_valid <= 1'b0;
            sum_real <= {OUT_WIDTH{1'b0}};
            sum_imag <= {OUT_WIDTH{1'b0}};
        end else begin
            sum_valid <= level5_valid;
            sum_real <= level5_real;
            sum_imag <= level5_imag;
        end
    end

endmodule