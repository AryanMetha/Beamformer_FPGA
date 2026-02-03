################################################################################
# MINIMAL XDC - For Quick Synthesis/Implementation Test
# Beamformer Test Wrapper
################################################################################

################################################################################
# CLOCKS - REQUIRED
################################################################################

# Core clock: 250 MHz (4ns period)
create_clock -period 4.000 -name clk_250mhz [get_ports clk_250mhz]

# AXI clock: 100 MHz (10ns period)
create_clock -period 10.000 -name clk_100mhz [get_ports clk_100mhz]

# Clocks are asynchronous (no timing relationship)
set_clock_groups -asynchronous \
    -group [get_clocks clk_250mhz] \
    -group [get_clocks clk_100mhz]

################################################################################
# RESET - REQUIRED
################################################################################

# Reset is asynchronous - no timing check needed
set_false_path -from [get_ports rst_n]

################################################################################
# OUTPUTS - OPTIONAL (but recommended)
################################################################################

# Outputs relative to 250 MHz clock
set_output_delay -clock clk_250mhz -max 2.0 [get_ports beam_*]
set_output_delay -clock clk_250mhz -min 0.0 [get_ports beam_*]

# LEDs are slow - no timing requirements
set_false_path -to [get_ports status_leds[*]]

################################################################################
# That's it! Minimal constraints for first test
################################################################################
