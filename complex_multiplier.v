// complex_multiplier.v - Single (I + jQ) * (Cr + jCi)
// Verilog 2001 compatible

`timescale 1ns/1ps

module complex_multiplier #(
    parameter DATA_WIDTH  = 16,
    parameter COEFF_WIDTH = 16,
    parameter OUT_WIDTH   = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire signed [DATA_WIDTH-1:0]  i_in,
    input  wire signed [DATA_WIDTH-1:0]  q_in,
    input  wire signed [COEFF_WIDTH-1:0] coeff_real,
    input  wire signed [COEFF_WIDTH-1:0] coeff_imag,

    output reg  signed [OUT_WIDTH-1:0]   real_out,
    output reg  signed [OUT_WIDTH-1:0]   imag_out
);

    // (I + jQ)(Cr + jCi) = (I*Cr - Q*Ci) + j(I*Ci + Q*Cr)

    reg signed [OUT_WIDTH-1:0] prod_ir;  // I * Cr
    reg signed [OUT_WIDTH-1:0] prod_qc;  // Q * Ci
    reg signed [OUT_WIDTH-1:0] prod_ic;  // I * Ci
    reg signed [OUT_WIDTH-1:0] prod_qr;  // Q * Cr

    // Stage 1: multiplies
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_ir <= {OUT_WIDTH{1'b0}};
            prod_qc <= {OUT_WIDTH{1'b0}};
            prod_ic <= {OUT_WIDTH{1'b0}};
            prod_qr <= {OUT_WIDTH{1'b0}};
        end else begin
            prod_ir <= i_in * coeff_real;
            prod_qc <= q_in * coeff_imag;
            prod_ic <= i_in * coeff_imag;
            prod_qr <= q_in * coeff_real;
        end
    end

    // Stage 2: add/sub
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            real_out <= {OUT_WIDTH{1'b0}};
            imag_out <= {OUT_WIDTH{1'b0}};
        end else begin
            real_out <= prod_ir - prod_qc;
            imag_out <= prod_ic + prod_qr;
        end
    end

endmodule
