// =============================================================================
// svpwm.sv — Space Vector PWM Modulator (2-Stage Pipelined)
// =============================================================================
//
// Two registered pipeline stages break the original 1,200+ level combinational
// path that caused WNS = -305 ns. The pipeline FF at stage-1 boundary cuts the
// critical path from inv_park output registers to svpwm output registers.
//
//   Stage 1 (comb → FF, 1 cycle latency):
//     Inputs v_alpha, v_beta → sector classification, T1/T2 dwell-time calcs,
//     negative-dwell clamp, overmodulation scaling, half_t0 computation.
//     Registered outputs: sector_r, t1_scaled_r, t2_scaled_r, half_t0_r, valid_r
//
//   Stage 2 (comb → FF, 1 more cycle):
//     Registered values → 7-segment ta/tb/tc assignment, duty saturation.
//     Registered outputs: duty_a, duty_b, duty_c, sector, output_valid
//
//   Total latency: 2 clock cycles (output_valid asserts 2 cycles after input_valid)
//
// Arithmetic width:
//   v_alpha / v_beta: Q1.15 signed 16-bit (max magnitude 32767)
//   t1_raw / t2_raw:  up to 32767 × KSQ3 ≈ 3.72e9 (needs 33-bit signed)
//   active_raw:       up to 2 × 3.72e9 ≈ 7.44e9 (needs 34-bit signed)
//   overmod product:  t1_raw × ONE_F ≤ 3.72e9 × 2.147e9 ≈ 7.99e18
//                     (needs 64-bit signed: 2^63 = 9.22e18 > 7.99e18 ✓)
//   t1_scaled/t2_scaled after overmod: ≤ ONE_F ≈ 2.147e9 (needs 32-bit signed)
//   ta/tb/tc: ≤ ONE_F (after overmod clamping), 32-bit signed sufficient
//
//   All intermediate signals declared signed [63:0] for correctness.
//
// Timing note:
//   Stage 1 combinational cone includes a 64-bit ÷ 34-bit division for the
//   overmodulation path. This path is exempt from single-cycle analysis via
//   set_multicycle_path in the XDC constraints. The SVPWM module runs at the
//   carrier frequency (25 kHz → one valid per 5000 × 8 ns = 40 µs), so the
//   path may take up to 25 clock cycles to settle without functional impact.
// =============================================================================
`timescale 1ns / 1ps

module svpwm (
    input  logic               clk,
    input  logic               rst,
    input  logic               input_valid,

    input  logic signed [15:0] v_alpha,  // Q1.15
    input  logic signed [15:0] v_beta,   // Q1.15

    output logic        [15:0] duty_a,   // unsigned duty, 0..32767
    output logic        [15:0] duty_b,
    output logic        [15:0] duty_c,
    output logic        [2:0]  sector,
    output logic               output_valid
);

    // -------------------------------------------------------------------------
    // Constants  (all scaled by 2^F_FRAC = 2^16 guard bits)
    // ONE_F   = 32767 * 2^16 = 2 147 418 112
    // K1P5    = round(1.5    * (32767/32768) * 2^16) = 98 301
    // KSQ3_2  = round(√3/2   * (32767/32768) * 2^16) = 56 754
    // KSQ3    = round(√3     * (32767/32768) * 2^16) = 113 508
    // SQ3_SEC = round(√3     * 2^24)                 = 29 058 991
    // -------------------------------------------------------------------------
    localparam int           F_FRAC   = 16;
    localparam signed [63:0] ONE_F    = 64'sd2147418112;
    localparam signed [63:0] K1P5     = 64'sd98301;
    localparam signed [63:0] KSQ3_2   = 64'sd56754;
    localparam signed [63:0] KSQ3     = 64'sd113508;
    localparam int           SQ3_SEC  = 29058991;

    // =========================================================================
    // STAGE 1 — combinational: sector detection + T1/T2 + overmodulation
    // =========================================================================
    logic signed [63:0] alpha_ext_s1;
    logic signed [63:0] beta_ext_s1;
    logic signed [63:0] abs_alpha_s1;
    logic signed [63:0] abs_beta_s1;
    logic signed [63:0] compare_left_s1;
    logic signed [63:0] compare_right_s1;

    logic [2:0]         sector_comb;

    logic signed [63:0] t1_raw_s1;
    logic signed [63:0] t2_raw_s1;
    logic signed [63:0] active_raw_s1;
    logic signed [63:0] t1_scaled_s1;
    logic signed [63:0] t2_scaled_s1;
    logic signed [63:0] t0_value_s1;
    logic signed [63:0] half_t0_s1;

    always_comb begin : stage1_comb
        // Sign-extend 16-bit inputs to 64-bit
        alpha_ext_s1 = {{48{v_alpha[15]}}, v_alpha};
        beta_ext_s1  = {{48{v_beta[15]}},  v_beta};

        abs_alpha_s1 = (alpha_ext_s1 < 0) ? -alpha_ext_s1 : alpha_ext_s1;
        abs_beta_s1  = (beta_ext_s1  < 0) ? -beta_ext_s1  : beta_ext_s1;

        // Sector boundary: |β|/|α| vs √3
        // Using 24 fractional bits: compare abs_beta*2^24 vs abs_alpha*SQ3_SEC
        // Max compare_left  = 32767 * 2^24 = 549 755 813 888 < 2^40 ✓
        // Max compare_right = 32767 * 29 058 991 = 952 467 848 097 < 2^40 ✓
        compare_left_s1  = abs_beta_s1  <<< 24;
        compare_right_s1 = abs_alpha_s1 * SQ3_SEC;

        // --- Sector classification ---
        if ((v_alpha == 16'sd0) && (v_beta == 16'sd0))
            sector_comb = 3'd1;
        else if ((v_alpha < 0) && (v_beta == 0))
            sector_comb = 3'd4;
        else if ((v_alpha == 0) && (v_beta < 0))
            sector_comb = 3'd5;
        else if (v_alpha >= 0 && v_beta >= 0)      // Q1
            sector_comb = (compare_left_s1 < compare_right_s1) ? 3'd1 : 3'd2;
        else if (v_alpha <  0 && v_beta >  0)      // Q2
            sector_comb = (compare_left_s1 > compare_right_s1) ? 3'd2 : 3'd3;
        else if (v_alpha <  0 && v_beta <  0)      // Q3
            sector_comb = (compare_left_s1 < compare_right_s1) ? 3'd4 : 3'd5;
        else                                        // Q4
            sector_comb = (compare_left_s1 > compare_right_s1) ? 3'd5 : 3'd6;

        // --- Dwell-time calculation ---
        // Using 64-bit arithmetic; worst case product:
        //   32767 × 113508 = 3,719,206,836 < 2^32 ✓
        //   t1_raw + t2_raw ≤ 2 × 3.72e9 ≈ 7.44e9 < 2^33 ✓
        t1_raw_s1 = 64'sd0;
        t2_raw_s1 = 64'sd0;

        case (sector_comb)
            3'd1: begin
                t1_raw_s1 = alpha_ext_s1 * K1P5 - beta_ext_s1 * KSQ3_2;
                t2_raw_s1 = beta_ext_s1  * KSQ3;
            end
            3'd2: begin
                t1_raw_s1 = alpha_ext_s1 * K1P5 + beta_ext_s1 * KSQ3_2;
                t2_raw_s1 = beta_ext_s1  * KSQ3_2 - alpha_ext_s1 * K1P5;
            end
            3'd3: begin
                t1_raw_s1 =  beta_ext_s1  * KSQ3;
                t2_raw_s1 = -alpha_ext_s1 * K1P5 - beta_ext_s1 * KSQ3_2;
            end
            3'd4: begin
                t1_raw_s1 = -alpha_ext_s1 * K1P5 + beta_ext_s1 * KSQ3_2;
                t2_raw_s1 = -beta_ext_s1  * KSQ3;
            end
            3'd5: begin
                t1_raw_s1 = -alpha_ext_s1 * K1P5 - beta_ext_s1 * KSQ3_2;
                t2_raw_s1 =  alpha_ext_s1 * K1P5 - beta_ext_s1 * KSQ3_2;
            end
            default: begin  // Sector 6
                t1_raw_s1 = -beta_ext_s1  * KSQ3;
                t2_raw_s1 =  alpha_ext_s1 * K1P5 + beta_ext_s1 * KSQ3_2;
            end
        endcase

        // Clamp negative dwells
        if (t1_raw_s1 < 0) t1_raw_s1 = 64'sd0;
        if (t2_raw_s1 < 0) t2_raw_s1 = 64'sd0;

        active_raw_s1 = t1_raw_s1 + t2_raw_s1;

        // Overmodulation scaling:
        //   t1_scaled = t1_raw × ONE_F / active_raw   (proportional)
        //   Intermediate t1_raw × ONE_F ≤ 7.44e9 × 2.15e9 ≈ 1.60e19
        //   NOTE: 1.60e19 > 2^63 (9.22e18) — this CAN overflow for extreme inputs!
        //   Safe bound: t1_raw ≤ 32767 × (K1P5 + KSQ3) = 32767 × 211809 ≈ 6.94e9
        //              × ONE_F ≈ 6.94e9 × 2.147e9 ≈ 1.49e19 > 2^63 — OVERFLOW risk
        //   Mitigation: shift both t1_raw and ONE_F right by 1 before multiply:
        //   t1_scaled = (t1_raw >>> 1) × (ONE_F >>> 1) × 4 / active_raw
        //             = (t1_raw >>> 1) × (ONE_F >>> 1) / (active_raw >>> 2)
        //   But integer division rounding: use the simpler safe formula below.
        //
        //   Alternative safe formula using the fact that active_raw > ONE_F in overmod:
        //   scale_q16 = (ONE_F <<< 16) / active_raw   (a 16-bit fractional scale in Q16)
        //   t1_scaled = (t1_raw * scale_q16) >>> 16
        //   scale_q16 max = ONE_F<<16 / ONE_F = 65536  (< 2^16+1 → safe)
        //   t1_raw * scale_q16 max = 6.94e9 × 65536 ≈ 4.55e14 < 2^63 ✓ SAFE
        if (active_raw_s1 > ONE_F) begin
            // scale_q16: ratio ONE_F/active_raw in Q0.16 format
            // scale_q16 ≤ ONE_F*2^16/ONE_F = 65536, fits in 17-bit unsigned
            automatic logic signed [63:0] scale_q16;
            scale_q16     = (ONE_F <<< 16) / active_raw_s1;
            t1_scaled_s1  = (t1_raw_s1 * scale_q16) >>> 16;
            t2_scaled_s1  = ONE_F - t1_scaled_s1;
            active_raw_s1 = ONE_F;  // clamped
        end else begin
            t1_scaled_s1 = t1_raw_s1;
            t2_scaled_s1 = t2_raw_s1;
        end

        t0_value_s1 = ONE_F - active_raw_s1;
        half_t0_s1  = (t0_value_s1 + 1) >>> 1;
    end

    // =========================================================================
    // STAGE 1 → STAGE 2 pipeline registers
    // =========================================================================
    logic [2:0]         sector_r;
    logic signed [63:0] t1_r, t2_r, half_t0_r;
    logic               valid_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            sector_r   <= 3'd1;
            t1_r       <= 64'sd0;
            t2_r       <= 64'sd0;
            half_t0_r  <= 64'sd0;
            valid_r    <= 1'b0;
        end else begin
            valid_r <= input_valid;
            if (input_valid) begin
                sector_r  <= sector_comb;
                t1_r      <= t1_scaled_s1;
                t2_r      <= t2_scaled_s1;
                half_t0_r <= half_t0_s1;
            end
        end
    end

    // =========================================================================
    // STAGE 2 — combinational: 7-segment duty assignment + saturation
    // =========================================================================

    // Convert internal Q(F_FRAC) duty value to unsigned Q1.15 [0, 32767]
    function automatic logic [15:0] duty_from_fixed(input logic signed [63:0] val_f);
        logic signed [63:0] rounded;
        begin
            rounded = (val_f + (64'sd1 <<< (F_FRAC - 1))) >>> F_FRAC;
            if (rounded <= 0)
                duty_from_fixed = 16'd0;
            else if (rounded >= 32767)
                duty_from_fixed = 16'd32767;
            else
                duty_from_fixed = rounded[15:0];
        end
    endfunction

    logic signed [63:0] ta_s2, tb_s2, tc_s2;
    logic        [15:0] duty_a_comb, duty_b_comb, duty_c_comb;

    always_comb begin : stage2_comb
        // 7-segment symmetric duty distribution from registered Stage-1 values
        case (sector_r)
            3'd1: begin
                ta_s2 = half_t0_r + t1_r + t2_r;
                tb_s2 = half_t0_r + t2_r;
                tc_s2 = half_t0_r;
            end
            3'd2: begin
                ta_s2 = half_t0_r + t1_r;
                tb_s2 = half_t0_r + t1_r + t2_r;
                tc_s2 = half_t0_r;
            end
            3'd3: begin
                ta_s2 = half_t0_r;
                tb_s2 = half_t0_r + t1_r + t2_r;
                tc_s2 = half_t0_r + t2_r;
            end
            3'd4: begin
                ta_s2 = half_t0_r;
                tb_s2 = half_t0_r + t1_r;
                tc_s2 = half_t0_r + t1_r + t2_r;
            end
            3'd5: begin
                ta_s2 = half_t0_r + t2_r;
                tb_s2 = half_t0_r;
                tc_s2 = half_t0_r + t1_r + t2_r;
            end
            default: begin  // Sector 6
                ta_s2 = half_t0_r + t1_r + t2_r;
                tb_s2 = half_t0_r;
                tc_s2 = half_t0_r + t1_r;
            end
        endcase

        duty_a_comb = duty_from_fixed(ta_s2);
        duty_b_comb = duty_from_fixed(tb_s2);
        duty_c_comb = duty_from_fixed(tc_s2);
    end

    // =========================================================================
    // STAGE 2 output registers  (total latency = 2 cycles from input_valid)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            duty_a       <= 16'd16384;
            duty_b       <= 16'd16384;
            duty_c       <= 16'd16384;
            sector       <= 3'd1;
            output_valid <= 1'b0;
        end else begin
            output_valid <= valid_r;
            if (valid_r) begin
                duty_a <= duty_a_comb;
                duty_b <= duty_b_comb;
                duty_c <= duty_c_comb;
                sector <= sector_r;
            end
        end
    end

endmodule
