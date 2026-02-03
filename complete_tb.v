`timescale 1ns/1ps
`include "beamformer_defines.vh"

////////////////////////////////////////////////////////////////////////////////
// COMPREHENSIVE BEAMFORMER TESTBENCH
// Based on project requirements:
// - Generate sine wave test vectors from specific directions
// - Calculate expected output (golden reference)
// - Verify cycle-accurate behavior
// - Test beamforming directivity
// - Measure latency and throughput
////////////////////////////////////////////////////////////////////////////////

module tb_beamformer_comprehensive;

    //==========================================================================
    // PARAMETERS
    //==========================================================================
    parameter NUM_LANES = `NUM_LANES;
    parameter NUM_CH = `NUM_CH_PER_LANE;
    parameter TOTAL_CH = NUM_LANES * NUM_CH;  // 48 channels
    parameter IQ_WIDTH = `IQ_DATA_WIDTH;
    parameter COEFF_WIDTH = `COEFF_WIDTH;
    parameter ACC_WIDTH = `ACC_WIDTH;
    
    parameter CLK_CORE_PERIOD = `CORE_CLK_PERIOD;
    parameter CLK_AXI_PERIOD = `AXI_CLK_PERIOD;
    
    // Test parameters
    parameter real ARRAY_SPACING = 0.5;  // Lambda/2 spacing
    parameter real SIGNAL_FREQ = 10.0e6; // 10 MHz signal
    parameter real SAMPLE_RATE = 500.0e6; // 500 MSPS
    parameter real PI = 3.14159265359;
    
    // Amplitude scaling for Q15 format
    parameter real Q15_SCALE = 32767.0;
    
    //==========================================================================
    // TESTBENCH SIGNALS
    //==========================================================================
    
    // Clocks and reset
    reg core_clk;
    reg axi_clk;
    reg async_rst_n;
    
    // ADC interface
    reg [(NUM_LANES*NUM_CH*2*IQ_WIDTH)-1:0] adc_data;
    reg [NUM_LANES-1:0] adc_valid;
    
    // Beamformer outputs
    wire signed [ACC_WIDTH-1:0] beam_real;
    wire signed [ACC_WIDTH-1:0] beam_imag;
    wire beam_valid;
    reg beam_ready;
    
    // AXI-Lite interface
    reg [`COEFF_ADDR_WIDTH-1:0] axi_addr;
    reg [(2*COEFF_WIDTH)-1:0] axi_data_w;
    reg axi_we;
    wire axi_ack;
    
    // Status/debug
    wire [31:0] status_core;
    wire [31:0] status_coeff;
    wire [31:0] debug_lane_valid;
    
    //==========================================================================
    // TEST CONTROL
    //==========================================================================
    reg [31:0] sample_count;
    reg [31:0] output_count;
    reg [31:0] error_count;
    reg [31:0] test_number;
    
    // Golden reference
    real expected_real_fp;
    real expected_imag_fp;
    reg signed [ACC_WIDTH-1:0] expected_real;
    reg signed [ACC_WIDTH-1:0] expected_imag;
    
    // Test configuration
    real signal_angle;        // Arrival angle in degrees
    real beam_steer_angle;    // Beamforming direction in degrees
    real signal_amplitude;
    
    // Timing measurements
    reg [31:0] first_valid_cycle;
    reg [31:0] latency_cycles;
    reg latency_measured;
    
    // File handles
    integer log_file;
    integer results_file;
    integer waveform_file;
    
    //==========================================================================
    // CLOCK GENERATION
    //==========================================================================
    
    initial begin
        core_clk = 0;
        forever #(CLK_CORE_PERIOD/2) core_clk = ~core_clk;
    end
    
    initial begin
        axi_clk = 0;
        forever #(CLK_AXI_PERIOD/2) axi_clk = ~axi_clk;
    end
    
    //==========================================================================
    // DUT INSTANTIATION
    //==========================================================================
   
    beamformer_top #(
        .NUM_LANES(NUM_LANES),
        .NUM_CH(NUM_CH),
        .IQ_WIDTH(IQ_WIDTH),
        .COEFF_WIDTH(COEFF_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) dut (
        .core_clk(core_clk),
        .axi_clk(axi_clk),
        .async_rst_n(async_rst_n),
        
        .adc_data_i(adc_data),
        .adc_valid_i(adc_valid),
        
        .beam_real_o(beam_real),
        .beam_imag_o(beam_imag),
        .beam_valid_o(beam_valid),
        .beam_ready_i(beam_ready),
        
        .axi_addr(axi_addr),
        .axi_data_w(axi_data_w),
        .axi_we(axi_we),
        .axi_ack(axi_ack),
        
        .status_core(status_core),
        .status_coeff(status_coeff),
        .debug_lane_valid(debug_lane_valid)
    );

    //==========================================================================
    // SINE WAVE GENERATOR WITH PHASE SHIFTS
    //==========================================================================
    
    real phase_time;
    real omega;  // Angular frequency
    integer ch_idx, lane_idx, flat_idx;
    real channel_phase;
    real i_sample_fp, q_sample_fp;
    reg signed [IQ_WIDTH-1:0] i_sample, q_sample;
    
    initial begin
        omega = 2.0 * PI * SIGNAL_FREQ / SAMPLE_RATE;
    end
    
    // Generate ADC data with spatial phase shifts
    always @(posedge core_clk or negedge async_rst_n) begin
        if (!async_rst_n) begin
            adc_data <= {(NUM_LANES*NUM_CH*2*IQ_WIDTH){1'b0}};
            phase_time <= 0.0;
        end else if (adc_valid != 2'b00) begin
            // Generate samples for all channels with spatial phase shifts
            for (lane_idx = 0; lane_idx < NUM_LANES; lane_idx = lane_idx + 1) begin
                for (ch_idx = 0; ch_idx < NUM_CH; ch_idx = ch_idx + 1) begin
                    flat_idx = lane_idx * NUM_CH + ch_idx;
                    
                    // Calculate phase shift based on angle of arrival
                    // phase = k * d * sin(theta) where k=2*pi/lambda, d=channel_spacing
                    // For lambda/2 spacing: phase = pi * channel_index * sin(theta_radians)
                    channel_phase = PI * flat_idx * $sin(signal_angle * PI / 180.0);
                    
                    // Generate I and Q samples (complex sine wave)
                    i_sample_fp = signal_amplitude * $cos(omega * phase_time + channel_phase);
                    q_sample_fp = signal_amplitude * $sin(omega * phase_time + channel_phase);
                    
                    // Convert to Q15 fixed-point
                    i_sample = $rtoi(i_sample_fp * Q15_SCALE);
                    q_sample = $rtoi(q_sample_fp * Q15_SCALE);
                    
                    // Pack into ADC data
                    adc_data[(lane_idx*NUM_CH*2*IQ_WIDTH) + (ch_idx*2*IQ_WIDTH) + IQ_WIDTH-1 -: IQ_WIDTH] = i_sample;
                    adc_data[(lane_idx*NUM_CH*2*IQ_WIDTH) + (ch_idx*2*IQ_WIDTH) + (2*IQ_WIDTH)-1 -: IQ_WIDTH] = q_sample;
                end
            end
            
            phase_time = phase_time + 1.0;
        end
    end
    
    //==========================================================================
    // BEAMFORMING COEFFICIENTS CALCULATION
    //==========================================================================
    
    // Calculate and write beamforming coefficients for given steering angle
    task set_steering_angle;
        input real angle_deg;
        integer coeff_ch;
        real steer_phase;
        real coeff_i_fp, coeff_q_fp;
        reg signed [COEFF_WIDTH-1:0] coeff_i, coeff_q;
        begin
            $display("[%0t] Setting steering angle to %.1f degrees", $time, angle_deg);
            
            for (coeff_ch = 0; coeff_ch < NUM_CH; coeff_ch = coeff_ch + 1) begin
                // Calculate phase shift for this channel
                // Conjugate for receive beamforming: -k*d*sin(theta)
                steer_phase = -PI * coeff_ch * $sin(angle_deg * PI / 180.0);
                
                // Complex exponential: e^(-j*phase) = cos(phase) - j*sin(phase)
                coeff_i_fp = $cos(steer_phase);
                coeff_q_fp = -$sin(steer_phase);
                
                // Convert to Q15
                coeff_i = $rtoi(coeff_i_fp * Q15_SCALE);
                coeff_q = $rtoi(coeff_q_fp * Q15_SCALE);
                
                // Write to coefficient memory
                write_coefficient(coeff_ch, coeff_i, coeff_q);
            end
            
            // Wait for bank switch
            repeat(20) @(posedge core_clk);
            
            $display("[%0t] Steering coefficients loaded", $time);
        end
    endtask
    
    //==========================================================================
    // GOLDEN REFERENCE CALCULATION
    //==========================================================================
    
    task calculate_expected_output;
        integer calc_ch;
        real calc_phase, calc_steer_phase;
        real signal_i, signal_q;
        real coeff_i_fp, coeff_q_fp;
        real mult_real, mult_imag;
        real sum_real, sum_imag;
        begin
            sum_real = 0.0;
            sum_imag = 0.0;
            
            for (calc_ch = 0; calc_ch < TOTAL_CH; calc_ch = calc_ch + 1) begin
                // Signal phase at this channel
                calc_phase = PI * calc_ch * $sin(signal_angle * PI / 180.0);
                signal_i = signal_amplitude * $cos(omega * (phase_time - 11.0) + calc_phase);  // -11 for pipeline delay
                signal_q = signal_amplitude * $sin(omega * (phase_time - 11.0) + calc_phase);
                
                // Beamforming coefficient phase
                calc_steer_phase = -PI * calc_ch * $sin(beam_steer_angle * PI / 180.0);
                coeff_i_fp = $cos(calc_steer_phase);
                coeff_q_fp = -$sin(calc_steer_phase);
                
                // Complex multiplication: (signal_i + j*signal_q) * (coeff_i + j*coeff_q)
                mult_real = signal_i * coeff_i_fp - signal_q * coeff_q_fp;
                mult_imag = signal_i * coeff_q_fp + signal_q * coeff_i_fp;
                
                sum_real = sum_real + mult_real;
                sum_imag = sum_imag + mult_imag;
            end
            
            // Convert to fixed-point (scaled by Q15_SCALE for both signal and coeff)
            expected_real_fp = sum_real * Q15_SCALE * Q15_SCALE;
            expected_imag_fp = sum_imag * Q15_SCALE * Q15_SCALE;
            
            expected_real = $rtoi(expected_real_fp);
            expected_imag = $rtoi(expected_imag_fp);
        end
    endtask
    
    //==========================================================================
    // OUTPUT VERIFICATION
    //==========================================================================
    
    real magnitude_actual, magnitude_expected;
    real magnitude_error_db;
    real phase_actual, phase_expected, phase_error_deg;
    
    always @(posedge core_clk) begin
        if (beam_valid && beam_ready) begin
            output_count = output_count + 1;
            
            // Calculate expected output
            calculate_expected_output;
            
            // Calculate magnitudes and phases
            magnitude_actual = $sqrt($itor(beam_real*beam_real + beam_imag*beam_imag));
            magnitude_expected = $sqrt(expected_real_fp*expected_real_fp + expected_imag_fp*expected_imag_fp);
            
            if (magnitude_expected > 0.0) begin
                magnitude_error_db = 20.0 * $log10(magnitude_actual / magnitude_expected);
            end else begin
                magnitude_error_db = 0.0;
            end
            
            phase_actual = $atan2($itor(beam_imag), $itor(beam_real)) * 180.0 / PI;
            phase_expected = $atan2(expected_imag_fp, expected_real_fp) * 180.0 / PI;
            phase_error_deg = phase_actual - phase_expected;
            
            // Normalize phase error to [-180, 180]
            while (phase_error_deg > 180.0) phase_error_deg = phase_error_deg - 360.0;
            while (phase_error_deg < -180.0) phase_error_deg = phase_error_deg + 360.0;
            
            // Check errors
            if (magnitude_error_db > 0.5 || magnitude_error_db < -0.5) begin
                $display("[%0t] WARNING: Magnitude error %.2f dB (sample %0d)", 
                         $time, magnitude_error_db, output_count);
                error_count = error_count + 1;
            end
            
            if (phase_error_deg > 5.0 || phase_error_deg < -5.0) begin
                $display("[%0t] WARNING: Phase error %.2f deg (sample %0d)", 
                         $time, phase_error_deg, output_count);
            end
            
            // Log every 100th sample
            if (output_count % 100 == 0) begin
                $fwrite(results_file, "%0d,%.6f,%.6f,%.3f,%.3f\n",
                        output_count, magnitude_actual, magnitude_expected, 
                        magnitude_error_db, phase_error_deg);
            end
            
            // Measure latency on first valid output
            if (!latency_measured && output_count == 1) begin
                latency_cycles = sample_count - 1;  // -1 because first sample is at cycle 0
                latency_measured = 1;
                $display("[%0t] Latency measured: %0d cycles (%.1f ns)", 
                         $time, latency_cycles, latency_cycles * CLK_CORE_PERIOD);
            end
        end
    end
    
    //==========================================================================
    // MAIN TEST SEQUENCE
    //==========================================================================
    
    initial begin
        // Open files
        log_file = $fopen("test_log.txt", "w");
        results_file = $fopen("test_results.csv", "w");
        
        $fwrite(results_file, "Sample,Magnitude_Actual,Magnitude_Expected,Error_dB,Phase_Error_deg\n");
        
        // Initialize
        async_rst_n = 0;
        adc_valid = 2'b00;
        beam_ready = 1;
        axi_addr = 0;
        axi_data_w = 0;
        axi_we = 0;
        
        sample_count = 0;
        output_count = 0;
        error_count = 0;
        test_number = 0;
        latency_measured = 0;
        
        signal_amplitude = 0.1;  // 10% of full scale
        
        print_header;
        
        // Reset
        $display("\n[%0t] ===== RESET =====", $time);
        #200;
        async_rst_n = 1;
        repeat(20) @(posedge core_clk);
        
        //----------------------------------------------------------------------
        // TEST 1: Single tone from boresight (θ=0°)
        //----------------------------------------------------------------------
        test_number = 1;
        $display("\n[%0t] ===== TEST 1: Single tone from boresight (θ=0°) =====", $time);
        $display("Expected: Coherent summation, 48× gain (33.6 dB)");
        
        signal_angle = 0.0;
        beam_steer_angle = 0.0;
        set_steering_angle(beam_steer_angle);
        
        adc_valid = 2'b11;
        sample_count = 0;
        output_count = 0;
        
        repeat(1000) begin
            @(posedge core_clk);
            sample_count = sample_count + 1;
        end
        
        $display("Test 1 complete: %0d outputs, %0d errors", output_count, error_count);
        
        //----------------------------------------------------------------------
        // TEST 2: Single tone from θ=30°, beam steered to 30°
        //----------------------------------------------------------------------
        test_number = 2;
        $display("\n[%0t] ===== TEST 2: Single tone from θ=30°, beam at 30° =====", $time);
        $display("Expected: Phase shifts correctly applied");
        
        signal_angle = 30.0;
        beam_steer_angle = 30.0;
        set_steering_angle(beam_steer_angle);
        
        sample_count = 0;
        output_count = 0;
        
        repeat(1000) begin
            @(posedge core_clk);
            sample_count = sample_count + 1;
        end
        
        $display("Test 2 complete: %0d outputs, %0d errors", output_count, error_count);
        
        //----------------------------------------------------------------------
        // TEST 3: Signal from 0°, beam steered to 30° (null)
        //----------------------------------------------------------------------
        test_number = 3;
        $display("\n[%0t] ===== TEST 3: Signal from 0°, beam at 30° (directivity) =====", $time);
        $display("Expected: Reduced output due to beam pattern");
        
        signal_angle = 0.0;
        beam_steer_angle = 30.0;
        set_steering_angle(beam_steer_angle);
        
        sample_count = 0;
        output_count = 0;
        
        repeat(1000) begin
            @(posedge core_clk);
            sample_count = sample_count + 1;
        end
        
        $display("Test 3 complete: %0d outputs, %0d errors", output_count, error_count);
        
        //----------------------------------------------------------------------
        // TEST 4: Throughput test - sustained 500 MSPS
        //----------------------------------------------------------------------
        test_number = 4;
        $display("\n[%0t] ===== TEST 4: Throughput verification =====", $time);
        $display("Target: 500 MSPS sustained");
        
        signal_angle = 0.0;
        beam_steer_angle = 0.0;
        
        sample_count = 0;
        output_count = 0;
        
        repeat(10000) begin
            @(posedge core_clk);
            sample_count = sample_count + 1;
        end
        
        $display("Throughput: %.2f MSPS (samples=%0d, outputs=%0d)", 
                 (output_count * 1000.0) / (sample_count * CLK_CORE_PERIOD),
                 sample_count, output_count);
        
        // Stop ADC
        adc_valid = 2'b00;
        repeat(50) @(posedge core_clk);
        
        // Print summary
        print_summary;
        
        $fclose(log_file);
        $fclose(results_file);
        $finish;
    end
    
    //==========================================================================
    // HELPER TASKS
    //==========================================================================
    
    task write_coefficient;
        input [`COEFF_ADDR_WIDTH-1:0] addr;
        input signed [COEFF_WIDTH-1:0] coeff_i;
        input signed [COEFF_WIDTH-1:0] coeff_q;
        begin
            @(posedge axi_clk);
            axi_addr = addr;
            axi_data_w = {coeff_q, coeff_i};
            axi_we = 1;
            
            @(posedge axi_clk);
            wait(axi_ack == 1);
            @(posedge axi_clk);
            axi_we = 0;
        end
    endtask
    
    task print_header;
        begin
            $display("\n");
            $display("╔══════════════════════════════════════════════════════════════╗");
            $display("║       BEAMFORMER COMPREHENSIVE TEST SUITE                   ║");
            $display("╚══════════════════════════════════════════════════════════════╝");
            $display("Configuration:");
            $display("  Total channels:     %0d", TOTAL_CH);
            $display("  IQ width:           %0d bits", IQ_WIDTH);
            $display("  Accumulator width:  %0d bits", ACC_WIDTH);
            $display("  Sample rate:        %.1f MSPS", SAMPLE_RATE/1e6);
            $display("  Signal frequency:   %.1f MHz", SIGNAL_FREQ/1e6);
            $display("  Array spacing:      λ/2");
            $display("");
        end
    endtask
    
    task print_summary;
        begin
            $display("\n");
            $display("╔══════════════════════════════════════════════════════════════╗");
            $display("║                    TEST SUMMARY                              ║");
            $display("╠══════════════════════════════════════════════════════════════╣");
            $display("║ Total samples:          %10d                         ║", sample_count);
            $display("║ Total outputs:          %10d                         ║", output_count);
            $display("║ Errors:                 %10d                         ║", error_count);
            $display("║ Pipeline latency:       %10d cycles                  ║", latency_cycles);
            $display("║ Latency (ns):           %10.1f                       ║", latency_cycles * CLK_CORE_PERIOD);
            $display("╚══════════════════════════════════════════════════════════════╝");
            
            if (error_count == 0) begin
                $display("\n✓ ALL TESTS PASSED\n");
            end else begin
                $display("\n✗ %0d ERRORS DETECTED\n", error_count);
            end
        end
    endtask
    
    //==========================================================================
    // WAVEFORM DUMP
    //==========================================================================
    
    initial begin
        $dumpfile("comprehensive_waves.vcd");
        $dumpvars(0, tb_beamformer_comprehensive);
    end

endmodule