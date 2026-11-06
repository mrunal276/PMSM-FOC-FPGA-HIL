`timescale 1ns/1ps

module pmsm_plant (
    input  logic        clk,
    input  logic        rst,
    input  logic        update,          // Pulse at 20 kHz
    input  logic signed [15:0] vd_q15,  // Q1.15 voltage input (1.0 = 24 V)
    input  logic signed [15:0] vq_q15,  // Q1.15 voltage input (1.0 = 24 V)
    output logic signed [15:0] id_q15,  // Q1.15 current output (1.0 = 20 A)
    output logic signed [15:0] iq_q15,  // Q1.15 current output (1.0 = 20 A)
    output logic        [15:0] omega_q6,  // Q10.6 unsigned mechanical speed (clamped >= 0)
    output logic        [15:0] theta_q14, // Q2.14 unsigned electrical angle [0, 32767]
    output logic               update_done
);

    // Fixed-point scaling constants (in Q8.24 or Q16.16)
    localparam signed [31:0] C_DT_LD    = 32'sd98690;   // dt/Ld in Q8.24
    localparam signed [31:0] C_RS_DT_LD = 32'sd148035;  // Rs*dt/Ld in Q8.24
    localparam signed [31:0] C_DT_J     = 32'sd838861;  // dt/J in Q8.24
    localparam signed [31:0] C_TL_DT_J  = 32'sd419430;  // TL*dt/J in Q8.24
    localparam signed [31:0] C_TORQUE   = 32'sd34406;   // 1.5*PP*lam in Q16.16 (0.525)
    localparam signed [31:0] C_LAM_BEMF = 32'sd34541;   // lam*PP*dt/Lq in Q8.24
    localparam signed [31:0] C_PP_DT    = 32'sd1678;    // PP*dt in Q8.24
    localparam signed [31:0] C_THETA_K  = 32'sd34178;   // theta increment factor

    // State registers in Q16.16
    logic signed [31:0] id_state_q16;
    logic signed [31:0] iq_state_q16;
    logic signed [31:0] wm_state_q16;
    logic        [15:0] th_state_q14;

    // Convert inputs to Q16.16: V = vd_q15 * 48
    logic signed [31:0] vd_q16, vq_q16;
    assign vd_q16 = $signed(vd_q15) * 32'sd48;
    assign vq_q16 = $signed(vq_q15) * 32'sd48;

    // Output assignments
    // id_q15 = id_q16 / 40
    assign id_q15     = 16'(id_state_q16 / 32'sd40);
    assign iq_q15     = 16'(iq_state_q16 / 32'sd40);
    assign omega_q6   = (wm_state_q16 < 32'sd0) ? 16'd0 : 16'(wm_state_q16 >>> 10);
    assign theta_q14  = th_state_q14 & 16'h7FFF;

    // Euler calculations
    logic signed [63:0] term_vd_64, term_rsid_64, we_dt_64, cross_d_64;
    logic signed [31:0] did_q16;

    logic signed [63:0] term_vq_64, term_rsiq_64, cross_q_64, term_bemf_64;
    logic signed [31:0] diq_q16;

    logic signed [63:0] te_64, dwm_64;
    logic signed [31:0] te_q16, dwm_q16;

    logic signed [63:0] dth_64;
    logic signed [15:0] dth_q14;

    always_comb begin
        // did calculation
        term_vd_64   = ($signed(vd_q16) * $signed(C_DT_LD)) >>> 24;
        term_rsid_64 = ($signed(id_state_q16) * $signed(C_RS_DT_LD)) >>> 24;
        we_dt_64     = ($signed(wm_state_q16) * $signed(C_PP_DT)) >>> 24;
        cross_d_64   = ($signed(we_dt_64[31:0]) * $signed(iq_state_q16)) >>> 16;
        did_q16      = 32'(term_vd_64 - term_rsid_64 + cross_d_64);

        // diq calculation
        term_vq_64   = ($signed(vq_q16) * $signed(C_DT_LD)) >>> 24;
        term_rsiq_64 = ($signed(iq_state_q16) * $signed(C_RS_DT_LD)) >>> 24;
        cross_q_64   = ($signed(we_dt_64[31:0]) * $signed(id_state_q16)) >>> 16;
        term_bemf_64 = ($signed(wm_state_q16) * $signed(C_LAM_BEMF)) >>> 24;
        diq_q16      = 32'(term_vq_64 - term_rsiq_64 - cross_q_64 - term_bemf_64);

        // torque & dwm
        te_64   = ($signed(C_TORQUE) * $signed(iq_state_q16)) >>> 16;
        te_q16  = 32'(te_64);
        dwm_64  = ($signed(te_q16) * $signed(C_DT_J)) >>> 24;
        dwm_q16 = 32'(dwm_64 - ($signed(C_TL_DT_J) >>> 8));

        // dtheta in Q2.14 counts
        dth_64  = ($signed(wm_state_q16) * $signed(C_THETA_K)) >>> 22;
        dth_q14 = 16'(dth_64);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            id_state_q16 <= 32'sd0;
            iq_state_q16 <= 32'sd0;
            wm_state_q16 <= 32'sd0;
            th_state_q14 <= 16'd0;
            update_done  <= 1'b0;
        end else if (update) begin
            id_state_q16 <= id_state_q16 + did_q16;
            iq_state_q16 <= iq_state_q16 + diq_q16;
            wm_state_q16 <= wm_state_q16 + dwm_q16;
            th_state_q14 <= (th_state_q14 + dth_q14) & 16'h7FFF;
            update_done  <= 1'b1;
        end else begin
            update_done  <= 1'b0;
        end
    end

endmodule
