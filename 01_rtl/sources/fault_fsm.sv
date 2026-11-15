`timescale 1ns/1ps

module fault_fsm #(
    parameter int OC_THRESHOLD_SQ  = 26214,  // (0.9 pu)^2 * 32767 in Q1.15 units
    parameter int IQ_STALL_THRESH  = 26214,  // 0.8 pu in Q1.15
    parameter int OMEGA_MIN_THRESH = 128,    // 2 rad/s in Q10.6
    parameter int STALL_TIMEOUT    = 100_000 // cycles
)(
    input  logic               clk,
    input  logic               rst,
    input  logic signed [15:0] i_alpha,       // Q1.15
    input  logic signed [15:0] i_beta,        // Q1.15
    input  logic        [15:0] omega_meas,    // Q10.6
    input  logic signed [15:0] iq_ref,        // Q1.15
    input  logic               external_fault,
    output logic               fault_active,
    output logic               pwm_inhibit,
    output logic [2:0]         fault_code     // 0=none, 1=OC, 2=stall, 3=ext
);

    // Pipelined current-magnitude computation for 125 MHz timing closure
    logic signed [31:0] alpha_sq_r;
    logic signed [31:0] beta_sq_r;
    logic signed [31:0] i_sq_r;
    logic signed [15:0] abs_iq_ref_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            alpha_sq_r   <= 32'sd0;
            beta_sq_r    <= 32'sd0;
            i_sq_r       <= 32'sd0;
            abs_iq_ref_r <= 16'sd0;
        end else begin
            alpha_sq_r   <= $signed(i_alpha) * $signed(i_alpha);
            beta_sq_r    <= $signed(i_beta) * $signed(i_beta);
            i_sq_r       <= (alpha_sq_r + beta_sq_r) >>> 15;
            abs_iq_ref_r <= (iq_ref < 16'sd0) ? (-iq_ref) : iq_ref;
        end
    end

    logic [31:0] stall_counter;

    always_ff @(posedge clk) begin
        if (rst) begin
            fault_active  <= 1'b0;
            pwm_inhibit   <= 1'b0;
            fault_code    <= 3'd0;
            stall_counter <= 32'd0;
        end else if (!fault_active) begin
            if (external_fault) begin
                fault_active <= 1'b1;
                pwm_inhibit  <= 1'b1;
                fault_code   <= 3'd3;
            end else if (i_sq_r > OC_THRESHOLD_SQ) begin
                fault_active <= 1'b1;
                pwm_inhibit  <= 1'b1;
                fault_code   <= 3'd1;
            end else if (abs_iq_ref_r > 16'(IQ_STALL_THRESH) && omega_meas < 16'(OMEGA_MIN_THRESH)) begin
                if (stall_counter >= STALL_TIMEOUT - 1) begin
                    fault_active <= 1'b1;
                    pwm_inhibit  <= 1'b1;
                    fault_code   <= 3'd2;
                end else begin
                    stall_counter <= stall_counter + 32'd1;
                end
            end else begin
                stall_counter <= 32'd0;
            end
        end else begin
            // Latched until rst
            pwm_inhibit <= 1'b1;
        end
    end

endmodule
