`ifndef BEAMFORMER_DEFINES_VH
`define BEAMFORMER_DEFINES_VH

`timescale 1ns/1ps

// ========== CORE PARAMETERS ==========
`define NUM_LANES        2        // Number of parallel processing lanes
`define NUM_CH_PER_LANE  24       // Channels per lane (total = NUM_LANES * 24)
`define IQ_DATA_WIDTH    16       // ADC I/Q sample width (signed)
`define COEFF_WIDTH      16       // Coefficient width (Q15 fixed-point)
`define MULT_OUT_WIDTH   32       // Multiplier output width (before accumulation)
`define ACC_WIDTH        48       // Accumulator width (final sum)

`define TOTAL_CHANNELS   (`NUM_LANES * `NUM_CH_PER_LANE)  // 48 total
`define COEFF_ADDR_WIDTH 5        // log2(24) = 5 bits needed

// ========== CLOCKS (periods in ns) ==========
`define ADC_CLK_PERIOD   2.0      // 500 MHz ADC sampling clock
`define CORE_CLK_PERIOD  4.0      // 250 MHz core processing clock
`define AXI_CLK_PERIOD   10.0     // 100 MHz AXI configuration clock

// ========== PIPELINE STAGES ==========
`define MULT_PIPE_STAGES 2        // Complex multiplier pipeline depth
`define ACC_TREE_STAGES  6        // Accumulator tree depth (log2(24) rounded up)
`define TOTAL_LATENCY    (`MULT_PIPE_STAGES + `ACC_TREE_STAGES + 3)  // Total pipeline

// ========== WEIGHT UPDATES ==========
`define WEIGHT_UPDATE_PERIOD 1024 // Update coefficients every N samples
`define BANK_SWITCH_DELAY    4    // Cycles to wait after bank switch

// ========== FIXED-POINT SCALING ==========
// Q15 format: 1 sign bit + 15 fractional bits
// Range: -1.0 to +0.999969 (max = 0x7FFF)
`define Q15_ONE          16'h7FFF // 1.0 in Q15 format (0.999969)
`define Q15_HALF         16'h4000 // 0.5 in Q15 format
`define Q15_ZERO         16'h0000 // 0.0

// ========== AXI MEMORY MAP ==========
`define AXI_COEFF_BASE    8'h00   // Coefficient memory start
`define AXI_STATUS_REG    8'hF0   // Status register
`define AXI_CONTROL_REG   8'hF4   // Control register

// ========== STATUS BIT DEFINITIONS ==========
`define STATUS_VALID      32'h00000001  // Output valid
`define STATUS_OVERFLOW   32'h00000002  // Accumulator overflow detected
`define STATUS_BANK_A     32'h00000004  // Bank A active (0=A, 1=B)

`endif // BEAMFORMER_DEFINES_VH
