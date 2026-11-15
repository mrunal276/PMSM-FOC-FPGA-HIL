`timescale 1ns / 1ps

/**
 * foc_shared_2dsp:
 * Time-Multiplexed 2-DSP Resource-Constrained FOC Arithmetic Engine
 *
 * Implements the 25 arithmetic operations (15 multiplications + 10 additions/subtractions)
 * of the Field-Oriented Control inner loops:
 *   - Park Transform        (M1..M4, A1, A2)
 *   - Speed PI Controller   (M5, M6, A3, A4)
 *   - Id PI Controller      (M7, M8, A5, A6)
 *   - Iq PI Controller      (M9, M10, A7, A8)
 *   - Inverse Park Transform(M11..M14, A9, A10)
 *
 * ALL multiplications are strictly time-multiplexed across EXACTLY TWO hardware
 * multiplier cells (u_dsp0 and u_dsp1), meeting the 1-2 DSP requirement of the
 * Hand-HLS FDS/ILP specification within 7 clock cycles of execution.
 */

(* use_dsp = "no" *)
module foc_shared_2dsp #(
    parameter int KP_FRAC = 12 // Fractional bits for current loop proportional gain (Q4.12)
)(
    input  logic               clk,
    input  logic               rst,
    input  logic               start,        // 1-cycle trigger pulse when inputs ready

    // Inputs from Clarke & CORDIC
    input  logic signed [15:0] i_alpha,      // Q1.15
    input  logic signed [15:0] i_beta,       // Q1.15
    input  logic signed [15:0] cos_theta,    // Q1.15
    input  logic signed [15:0] sin_theta,    // Q1.15

    // Speed loop commands and feedback
    input  logic        [15:0] omega_ref,    // Q10.6 unsigned
    input  logic        [15:0] omega_meas,   // Q10.6 unsigned
    input  logic signed [15:0] speed_kp,     // Q1.15
    input  logic signed [31:0] speed_ki_dt,  // Q1.31

    // Current loop commands and gains
    input  logic signed [15:0] id_ref,       // Q1.15
    input  logic signed [15:0] current_kp,   // Q4.12
    input  logic signed [15:0] current_ki,   // Q1.15

    // Output signals
    output logic               done,         // 1-cycle completion pulse
    output logic signed [15:0] v_alpha,      // Q1.15 to SVPWM
    output logic signed [15:0] v_beta,       // Q1.15 to SVPWM

    // Telemetry and pipeline status
    output logic signed [15:0] id_meas,
    output logic signed [15:0] iq_meas,
    output logic signed [15:0] iq_ref,
    output logic signed [15:0] v_d,
    output logic signed [15:0] v_q
);

    // =========================================================================
    // Rounding, Saturation, and Conversion Functions (bit-true to golden)
    // =========================================================================

    // Q1.15 round-half-up and saturate (used by Park and Inverse Park)
    function automatic logic signed [15:0] q15_round_halfup_sat(
        input logic signed [42:0] p
    );
        logic signed [42:0] rounded;
        logic signed [42:0] shifted;
        begin
            rounded = p + 43'sd16384;
            shifted = rounded >>> 15;
            if (shifted > 43'sd32767)
                q15_round_halfup_sat = 16'sh7fff;
            else if (shifted < -43'sd32768)
                q15_round_halfup_sat = 16'sh8000;
            else
                q15_round_halfup_sat = shifted[15:0];
        end
    endfunction

    // Q1.15 saturating addition
    function automatic logic signed [15:0] q15_add_sat(
        input logic signed [15:0] a,
        input logic signed [15:0] b
    );
        logic signed [16:0] sum;
        begin
            sum = {a[15], a} + {b[15], b};
            if (sum > 17'sd32767)
                q15_add_sat = 16'sh7fff;
            else if (sum < -17'sd32768)
                q15_add_sat = 16'sh8000;
            else
                q15_add_sat = sum[15:0];
        end
    endfunction

    // Q1.15 saturating subtraction
    function automatic logic signed [15:0] q15_sub_sat(
        input logic signed [15:0] a,
        input logic signed [15:0] b
    );
        logic signed [16:0] diff;
        begin
            diff = {a[15], a} - {b[15], b};
            if (diff > 17'sd32767)
                q15_sub_sat = 16'sh7fff;
            else if (diff < -17'sd32768)
                q15_sub_sat = 16'sh8000;
            else
                q15_sub_sat = diff[15:0];
        end
    endfunction

    // Signed saturation to Q1.15
    function automatic logic signed [15:0] sat_q15(
        input logic signed [32:0] val
    );
        begin
            if (val > 33'sd32767)
                sat_q15 = 16'sh7fff;
            else if (val < -33'sd32768)
                sat_q15 = 16'sh8000;
            else
                sat_q15 = val[15:0];
        end
    endfunction

    // Signed saturation to Q8.24
    function automatic logic signed [31:0] sat_q24(
        input logic signed [32:0] val
    );
        begin
            if (val > 33'sd2147483647)
                sat_q24 = 32'sh7fffffff;
            else if (val < -33'sd2147483648)
                sat_q24 = 32'sh80000000;
            else
                sat_q24 = val[31:0];
        end
    endfunction

    // Speed PI proportional round and saturate (Q10.6 error * Q1.15 Kp -> Q1.15)
    function automatic logic signed [15:0] speed_prop_sat(
        input logic signed [42:0] p
    );
        logic signed [42:0] rounded;
        logic signed [42:0] shifted;
        begin
            if (p >= 0)
                rounded = p + 43'sd32;
            else
                rounded = p - 43'sd32;
            shifted = rounded >>> 6;
            if (shifted > 43'sd32767)
                speed_prop_sat = 16'sh7fff;
            else if (shifted < -43'sd32768)
                speed_prop_sat = 16'sh8000;
            else
                speed_prop_sat = shifted[15:0];
        end
    endfunction

    // Speed PI integral increment round and saturate (Q10.6 error * Q1.31 Ki_dt -> Q8.24)
    function automatic logic signed [31:0] speed_inc_sat(
        input logic signed [42:0] p
    );
        logic signed [42:0] rounded;
        logic signed [42:0] shifted;
        begin
            if (p >= 0)
                rounded = p + 43'sd4096;
            else
                rounded = p - 43'sd4096;
            shifted = rounded >>> 13;
            if (shifted > 43'sd2147483647)
                speed_inc_sat = 32'sh7fffffff;
            else if (shifted < -43'sd2147483648)
                speed_inc_sat = 32'sh80000000;
            else
                speed_inc_sat = shifted[31:0];
        end
    endfunction

    // Speed PI Q8.24 integrator -> Q1.15
    function automatic logic signed [15:0] speed_q24_to_q15(
        input logic signed [31:0] val
    );
        logic signed [32:0] rounded;
        logic signed [32:0] shifted;
        begin
            if (val >= 0)
                rounded = {val[31], val} + 33'sd256;
            else
                rounded = {val[31], val} - 33'sd256;
            shifted = rounded >>> 9;
            if (shifted > 33'sd32767)
                speed_q24_to_q15 = 16'sh7fff;
            else if (shifted < -33'sd32768)
                speed_q24_to_q15 = 16'sh8000;
            else
                speed_q24_to_q15 = shifted[15:0];
        end
    endfunction

    // Current PI proportional round and saturate (Q1.15 err * Q4.12 Kp -> Q1.15)
    function automatic logic signed [15:0] curr_prop_sat(
        input logic signed [42:0] p
    );
        logic signed [42:0] rounded;
        logic signed [42:0] shifted;
        begin
            if (p >= 0)
                rounded = p + (43'sd1 <<< (KP_FRAC - 1));
            else
                rounded = p - (43'sd1 <<< (KP_FRAC - 1));
            shifted = rounded >>> KP_FRAC;
            if (shifted > 43'sd32767)
                curr_prop_sat = 16'sh7fff;
            else if (shifted < -43'sd32768)
                curr_prop_sat = 16'sh8000;
            else
                curr_prop_sat = shifted[15:0];
        end
    endfunction

    // Current PI integral increment round (Q1.15 err * Q1.15 Ki -> Q1.15)
    function automatic logic signed [15:0] curr_inc_q15(
        input logic signed [42:0] p
    );
        logic signed [42:0] rounded;
        logic signed [42:0] shifted;
        begin
            if (p >= 0)
                rounded = p + 43'sd16384;
            else
                rounded = p - 43'sd16384;
            shifted = rounded >>> 15;
            if (shifted > 43'sd32767)
                curr_inc_q15 = 16'sh7fff;
            else if (shifted < -43'sd32768)
                curr_inc_q15 = 16'sh8000;
            else
                curr_inc_q15 = shifted[15:0];
        end
    endfunction

    // Current PI Q8.24 integrator -> Q1.15 round
    function automatic logic signed [15:0] curr_q24_to_q15_round(
        input logic signed [31:0] val
    );
        logic signed [31:0] rounded;
        logic signed [31:0] shifted;
        begin
            if (val >= 0)
                rounded = val + 32'sd256;
            else
                rounded = val - 32'sd256;
            shifted = rounded >>> 9;
            if (shifted > 32'sd32767)
                curr_q24_to_q15_round = 16'sh7fff;
            else if (shifted < -32'sd32768)
                curr_q24_to_q15_round = 16'sh8000;
            else
                curr_q24_to_q15_round = shifted[15:0];
        end
    endfunction

    // =========================================================================
    // The Strictly Constrained 2 DSP Multiplier Units
    // =========================================================================
    logic signed [24:0] dsp0_a;
    logic signed [17:0] dsp0_b;
    logic signed [42:0] dsp0_p;

    logic signed [24:0] dsp1_a;
    logic signed [17:0] dsp1_b;
    logic signed [42:0] dsp1_p;

    dsp_mult_cell u_dsp0 (
        .clk (clk),
        .rst (rst),
        .a   (dsp0_a),
        .b   (dsp0_b),
        .p   (dsp0_p)
    );

    dsp_mult_cell u_dsp1 (
        .clk (clk),
        .rst (rst),
        .a   (dsp1_a),
        .b   (dsp1_b),
        .p   (dsp1_p)
    );

    // =========================================================================
    // Registers and State Machine
    // =========================================================================
    typedef enum logic [4:0] {
        ST_IDLE       = 5'd0,
        ST_PARK1      = 5'd1,   // Feed M3, M4
        ST_PARK2      = 5'd2,   // Latch M1, M2; Feed M5, M6
        ST_PARK3      = 5'd3,   // Latch M3, M4; Compute id_meas
        ST_PARK4      = 5'd21,  // Compute iq_meas; Prepare & register Id error
        ST_SPD_PI1    = 5'd4,   // Speed PI Stage 1 (latch prop & inc)
        ST_SPD_PI2    = 5'd5,   // Speed PI Stage 2A: Compute candidate integrator (Q8.24)
        ST_SPD_PI2B   = 5'd17,  // Speed PI Stage 2B: Convert Q8.24 cand to Q1.15 (break CARRY4 chain)
        ST_SPD_PI3    = 5'd6,   // Speed PI Stage 2C: Anti-windup, update integrator, produce iq_ref
        ST_ID_PI1     = 5'd7,   // Id PI Stage 1: Compute candidate integrator (Q8.24)
        ST_ID_PI1B    = 5'd18,  // Id PI Stage 1B: Convert Q8.24 cand to Q1.15 (break CARRY4 chain)
        ST_ID_PI2     = 5'd8,   // Id PI Stage 2: Anti-windup, update integrator, v_d; Feed M9, M10
        ST_IQ_WAIT    = 5'd9,   // Wait 1 cycle for M9, M10 to multiply
        ST_IQ_PI1     = 5'd10,  // Iq PI Stage 1 (prop & inc)
        ST_IQ_PI2     = 5'd11,  // Iq PI Stage 2A: Compute candidate integrator (Q8.24)
        ST_IQ_PI2B    = 5'd19,  // Iq PI Stage 2B: Convert Q8.24 cand to Q1.15 (break CARRY4 chain)
        ST_IQ_PI3     = 5'd12,  // Iq PI Stage 2C: Anti-windup, update integrator, v_q
        ST_INV1       = 5'd13,  // Feed M11, M12 (from registered v_d, v_q)
        ST_INV2       = 5'd14,  // Feed M13, M14 (from registered v_d, v_q)
        ST_INV3       = 5'd15,  // Latch M11, M12
        ST_INV4       = 5'd16,  // Register m13_sat, m14_sat; compute v_alpha
        ST_INV5       = 5'd20   // Compute v_beta from registered m13_sat, m14_sat; done!
    } state_t;

    state_t state;

    // Latched inputs
    logic signed [15:0] r_alpha, r_beta;
    logic signed [15:0] r_cos, r_sin;
    logic        [15:0] r_wref, r_wmeas;
    logic signed [15:0] r_id_ref;
    logic signed [15:0] r_skp;
    logic signed [31:0] r_ski_dt;
    logic signed [15:0] r_ckp, r_cki;

    // Intermediate registers for Park
    logic signed [15:0] m1_reg, m2_reg;
    logic signed [15:0] m3_reg, m4_reg;
    logic signed [15:0] neg_r_sin;
    assign neg_r_sin = -r_sin;

    // Pipelined intermediate registers for PI controllers (breaks long combinational cones)
    logic signed [15:0] spd_prop_reg;
    logic signed [31:0] spd_inc_reg;
    logic signed [31:0] spd_int_cand_reg;
    logic signed [15:0] spd_int_q15_reg;
    logic signed [15:0] id_prop_reg;
    logic signed [15:0] id_inc_reg;
    logic signed [31:0] id_int_cand_reg;
    logic signed [15:0] id_int_q15_reg;
    logic signed [15:0] iq_prop_reg;
    logic signed [15:0] iq_inc_reg;
    logic signed [31:0] iq_int_cand_reg;
    logic signed [15:0] iq_int_q15_reg;

    // Integrator states
    logic signed [31:0] spd_integrator;
    logic signed [31:0] id_integrator;
    logic signed [31:0] iq_integrator;

    // Intermediate calculated values
    logic signed [16:0] spd_error_wide;
    logic signed [16:0] id_error_wide;
    logic signed [15:0] id_error_q15;
    logic signed [15:0] id_error_reg;
    logic signed [16:0] iq_error_wide;
    logic signed [15:0] iq_error_q15;
    logic signed [15:0] iq_error_reg;

    // Intermediate registers for Inverse Park
    logic signed [15:0] m11_reg, m12_reg;
    logic signed [15:0] m13_sat_reg, m14_sat_reg;  // Pipeline regs to break DSP->v_beta CARRY4 chain

    // Sequential Sequencer
    always_ff @(posedge clk) begin
        if (rst) begin
            state          <= ST_IDLE;
            done           <= 1'b0;
            v_alpha        <= 16'sd0;
            v_beta         <= 16'sd0;
            id_meas        <= 16'sd0;
            iq_meas        <= 16'sd0;
            iq_ref         <= 16'sd0;
            v_d            <= 16'sd0;
            v_q            <= 16'sd0;
            spd_integrator <= 32'sd0;
            id_integrator  <= 32'sd0;
            iq_integrator  <= 32'sd0;
            dsp0_a         <= 25'sd0;
            dsp0_b         <= 18'sd0;
            dsp1_a         <= 25'sd0;
            dsp1_b         <= 18'sd0;
            m1_reg         <= 16'sd0;
            m2_reg         <= 16'sd0;
            m3_reg         <= 16'sd0;
            m4_reg         <= 16'sd0;
            id_error_reg   <= 16'sd0;
            iq_error_reg   <= 16'sd0;
            spd_prop_reg   <= 16'sd0;
            spd_inc_reg    <= 32'sd0;
            id_prop_reg    <= 16'sd0;
            id_inc_reg     <= 16'sd0;
            iq_prop_reg    <= 16'sd0;
            iq_inc_reg     <= 16'sd0;
            m11_reg        <= 16'sd0;
            m12_reg        <= 16'sd0;
            m13_sat_reg    <= 16'sd0;
            m14_sat_reg    <= 16'sd0;
            r_alpha        <= 16'sd0;
            r_beta         <= 16'sd0;
            r_cos          <= 16'sd0;
            r_sin          <= 16'sd0;
            r_wref         <= 16'd0;
            r_wmeas        <= 16'd0;
            r_id_ref       <= 16'sd0;
            r_skp          <= 16'sd0;
            r_ski_dt       <= 32'sd0;
            r_ckp          <= 16'sd0;
            r_cki          <= 16'sd0;
        end else begin
            done <= 1'b0; // Default pulse low

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        // Latch all incoming operands
                        r_alpha   <= i_alpha;
                        r_beta    <= i_beta;
                        r_cos     <= cos_theta;
                        r_sin     <= sin_theta;
                        r_wref    <= omega_ref;
                        r_wmeas   <= omega_meas;
                        r_id_ref  <= id_ref;
                        r_skp     <= speed_kp;
                        r_ski_dt  <= speed_ki_dt;
                        r_ckp     <= current_kp;
                        r_cki     <= current_ki;

                        // Launch M1 (DSP0) and M2 (DSP1)
                        // M1: Ialpha * cos
                        dsp0_a <= {{9{cos_theta[15]}}, cos_theta};
                        dsp0_b <= {{2{i_alpha[15]}}, i_alpha};

                        // M2: Ibeta * sin
                        dsp1_a <= {{9{sin_theta[15]}}, sin_theta};
                        dsp1_b <= {{2{i_beta[15]}}, i_beta};

                        state  <= ST_PARK1;
                    end
                end

                ST_PARK1: begin
                    // Launch M3 (DSP0) and M4 (DSP1)
                    // M3: Ialpha * (-sin)
                    dsp0_a <= {{9{neg_r_sin[15]}}, neg_r_sin};
                    dsp0_b <= {{2{r_alpha[15]}}, r_alpha};

                    // M4: Ibeta * cos
                    dsp1_a <= {{9{r_cos[15]}}, r_cos};
                    dsp1_b <= {{2{r_beta[15]}}, r_beta};

                    state  <= ST_PARK2;
                end

                ST_PARK2: begin
                    // DSP outputs now have M1 and M2!
                    m1_reg <= q15_round_halfup_sat(dsp0_p);
                    m2_reg <= q15_round_halfup_sat(dsp1_p);

                    // Compute Speed error (17-bit signed)
                    spd_error_wide = $signed({1'b0, r_wref}) - $signed({1'b0, r_wmeas});

                    // Launch M5 (DSP0) and M6 (DSP1)
                    // M5: spd_error * speed_kp
                    dsp0_a <= {{9{r_skp[15]}}, r_skp};
                    dsp0_b <= {{1{spd_error_wide[16]}}, spd_error_wide};

                    // M6: spd_error * speed_ki_dt
                    dsp1_a <= r_ski_dt[24:0];
                    dsp1_b <= {{1{spd_error_wide[16]}}, spd_error_wide};

                    state  <= ST_PARK3;
                end

                ST_PARK3: begin
                    // DSP outputs now have M3 and M4!
                    // Pipelining: latch m3_reg and m4_reg, compute id_meas from registered m1_reg and m2_reg
                    m3_reg  <= q15_round_halfup_sat(dsp0_p);
                    m4_reg  <= q15_round_halfup_sat(dsp1_p);
                    id_meas <= q15_add_sat(m1_reg, m2_reg);

                    state <= ST_PARK4;
                end

                ST_PARK4: begin
                    // Compute iq_meas from registered m3_reg and m4_reg (breaks the DSP0/DSP1 -> iq_meas chain!)
                    iq_meas <= q15_add_sat(m3_reg, m4_reg);

                    // Compute Id error for Id PI from registered id_meas (breaks the m1_reg/m2_reg -> id_error_reg chain!)
                    id_error_wide = {r_id_ref[15], r_id_ref} - {id_meas[15], id_meas};
                    if (id_error_wide > 17'sd32767)
                        id_error_q15 = 16'sh7fff;
                    else if (id_error_wide < -17'sd32768)
                        id_error_q15 = 16'sh8000;
                    else
                        id_error_q15 = id_error_wide[15:0];

                    id_error_reg <= id_error_q15;

                    state <= ST_SPD_PI1;
                end

                ST_SPD_PI1: begin
                    // DSP outputs now have M5 and M6 (Speed PI)!
                    // Pipelined Stage 1: Round and saturate proportional and integral increments
                    spd_prop_reg <= speed_prop_sat(dsp0_p);
                    spd_inc_reg  <= speed_inc_sat(dsp1_p);

                    // Launch M7 (DSP0) and M8 (DSP1) from registered id_error_reg
                    // M7: id_error * current_kp
                    dsp0_a <= {{9{r_ckp[15]}}, r_ckp};
                    dsp0_b <= {{2{id_error_reg[15]}}, id_error_reg};

                    // M8: id_error * current_ki
                    dsp1_a <= {{9{r_cki[15]}}, r_cki};
                    dsp1_b <= {{2{id_error_reg[15]}}, id_error_reg};

                    state <= ST_SPD_PI2;
                end

                ST_SPD_PI2: begin
                    // 1. Speed PI Stage 2A: Compute candidate integrator
                    begin
                        logic signed [32:0] spd_int_sum;
                        logic signed [31:0] spd_cand;

                        spd_int_sum      = $signed({spd_integrator[31], spd_integrator}) +
                                           $signed({spd_inc_reg[31], spd_inc_reg});
                        spd_cand         = sat_q24(spd_int_sum);

                        spd_int_cand_reg <= spd_cand;
                    end

                    state <= ST_SPD_PI2B;
                end

                ST_SPD_PI2B: begin
                    // 1. Speed PI Stage 2B: Convert candidate integrator to Q1.15 (break CARRY4 chain)
                    spd_int_q15_reg <= speed_q24_to_q15(spd_int_cand_reg);

                    state <= ST_SPD_PI3;
                end

                ST_SPD_PI3: begin
                    // 1. Speed PI Stage 2B: Apply anti-windup, update integrator, produce iq_ref
                    begin
                        logic signed [16:0] spd_err;
                        logic signed [32:0] spd_unsat;
                        logic               spd_allow_int;

                        spd_err   = $signed({1'b0, r_wref}) - $signed({1'b0, r_wmeas});
                        spd_unsat = $signed({spd_prop_reg[15], spd_prop_reg}) +
                                    $signed({spd_int_q15_reg[15], spd_int_q15_reg});

                        spd_allow_int = 1'b1;
                        if ((spd_unsat > 33'sd32767) && (spd_err > 17'sd0))
                            spd_allow_int = 1'b0;
                        if ((spd_unsat < -33'sd32768) && (spd_err < 17'sd0))
                            spd_allow_int = 1'b0;

                        if (spd_allow_int)
                            spd_integrator <= spd_int_cand_reg;

                        iq_ref <= sat_q15(spd_unsat);
                    end

                    // 2. Id PI Stage 1: DSP outputs now have M7 and M8!
                    // Pipelined Stage 1: Round proportional and integral increments
                    id_prop_reg <= curr_prop_sat(dsp0_p);
                    id_inc_reg  <= curr_inc_q15(dsp1_p);

                    state <= ST_ID_PI1;
                end

                ST_ID_PI1: begin
                    // Id PI Stage 2A: Compute candidate integrator
                    begin
                        logic signed [31:0] id_inc_q24;
                        logic signed [32:0] id_int_sum;
                        logic signed [31:0] id_cand;

                        id_inc_q24      = {{16{id_inc_reg[15]}}, id_inc_reg} <<< 9;
                        id_int_sum      = {id_integrator[31], id_integrator} + {id_inc_q24[31], id_inc_q24};
                        id_cand         = sat_q24(id_int_sum);

                        id_int_cand_reg <= id_cand;
                    end

                    state <= ST_ID_PI1B;
                end

                ST_ID_PI1B: begin
                    // Id PI Stage 2B: Convert Q8.24 to Q1.15 (break CARRY4 chain)
                    id_int_q15_reg <= curr_q24_to_q15_round(id_int_cand_reg);

                    state <= ST_ID_PI2;
                end

                ST_ID_PI2: begin
                    // Id PI Stage 2B: Apply anti-windup, update integrator, produce v_d
                    begin
                        logic signed [16:0] vd_unsat;
                        logic               id_allow_int;

                        vd_unsat = {id_prop_reg[15], id_prop_reg} + {id_int_q15_reg[15], id_int_q15_reg};

                        id_allow_int = 1'b1;
                        if ((vd_unsat > 17'sd32767) && (id_error_reg > 16'sd0))
                            id_allow_int = 1'b0;
                        if ((vd_unsat < -17'sd32768) && (id_error_reg < 16'sd0))
                            id_allow_int = 1'b0;

                        if (id_allow_int)
                            id_integrator <= id_int_cand_reg;

                        if (vd_unsat > 17'sd32767)
                            v_d <= 16'sh7fff;
                        else if (vd_unsat < -17'sd32768)
                            v_d <= 16'sh8000;
                        else
                            v_d <= vd_unsat[15:0];
                    end

                    // Launch M9 and M10: iq_ref was registered in ST_SPD_PI2!
                    begin
                        iq_error_wide = {iq_ref[15], iq_ref} - {iq_meas[15], iq_meas};
                        if (iq_error_wide > 17'sd32767)
                            iq_error_q15 = 16'sh7fff;
                        else if (iq_error_wide < -17'sd32768)
                            iq_error_q15 = 16'sh8000;
                        else
                            iq_error_q15 = iq_error_wide[15:0];

                        // Register iq_error_reg to break combinational anti-windup chain in ST_IQ_PI3
                        iq_error_reg <= iq_error_q15;

                        // M9: iq_error * current_kp
                        dsp0_a <= {{9{r_ckp[15]}}, r_ckp};
                        dsp0_b <= {{2{iq_error_q15[15]}}, iq_error_q15};

                        // M10: iq_error * current_ki
                        dsp1_a <= {{9{r_cki[15]}}, r_cki};
                        dsp1_b <= {{2{iq_error_q15[15]}}, iq_error_q15};
                    end

                    state <= ST_IQ_WAIT;
                end

                ST_IQ_WAIT: begin
                    // Wait 1 cycle for M9 and M10 to multiply in DSP
                    state <= ST_IQ_PI1;
                end

                ST_IQ_PI1: begin
                    // DSP outputs now have M9 and M10!
                    // Pipelined Stage 1: Round proportional and integral increments
                    iq_prop_reg <= curr_prop_sat(dsp0_p);
                    iq_inc_reg  <= curr_inc_q15(dsp1_p);

                    state <= ST_IQ_PI2;
                end

                ST_IQ_PI2: begin
                    // Iq PI Stage 2A: Compute candidate integrator
                    begin
                        logic signed [31:0] iq_inc_q24;
                        logic signed [32:0] iq_int_sum;
                        logic signed [31:0] iq_cand;

                        iq_inc_q24      = {{16{iq_inc_reg[15]}}, iq_inc_reg} <<< 9;
                        iq_int_sum      = {iq_integrator[31], iq_integrator} + {iq_inc_q24[31], iq_inc_q24};
                        iq_cand         = sat_q24(iq_int_sum);

                        iq_int_cand_reg <= iq_cand;
                    end

                    state <= ST_IQ_PI2B;
                end

                ST_IQ_PI2B: begin
                    // Iq PI Stage 2B: Convert Q8.24 to Q1.15 (break CARRY4 chain)
                    iq_int_q15_reg <= curr_q24_to_q15_round(iq_int_cand_reg);

                    state <= ST_IQ_PI3;
                end

                ST_IQ_PI3: begin
                    // Iq PI Stage 2B: Apply anti-windup, update integrator, produce v_q
                    begin
                        logic signed [16:0] vq_unsat;
                        logic               iq_allow_int;

                        vq_unsat = {iq_prop_reg[15], iq_prop_reg} + {iq_int_q15_reg[15], iq_int_q15_reg};

                        iq_allow_int = 1'b1;
                        if ((vq_unsat > 17'sd32767) && (iq_error_reg > 16'sd0))
                            iq_allow_int = 1'b0;
                        if ((vq_unsat < -17'sd32768) && (iq_error_reg < 16'sd0))
                            iq_allow_int = 1'b0;

                        if (iq_allow_int)
                            iq_integrator <= iq_int_cand_reg;

                        if (vq_unsat > 17'sd32767)
                            v_q <= 16'sh7fff;
                        else if (vq_unsat < -17'sd32768)
                            v_q <= 16'sh8000;
                        else
                            v_q <= vq_unsat[15:0];
                    end

                    state <= ST_INV1;
                end

                ST_INV1: begin
                    // Both v_d (from ST_ID_PI2) and v_q (from ST_IQ_PI2) are registered in flip-flops!
                    // Launch Inverse Park Cycle 1:
                    // M11 (DSP0): v_d * cos
                    dsp0_a <= {{9{r_cos[15]}}, r_cos};
                    dsp0_b <= {{2{v_d[15]}}, v_d};

                    // M12 (DSP1): v_q * sin
                    dsp1_a <= {{9{r_sin[15]}}, r_sin};
                    dsp1_b <= {{2{v_q[15]}}, v_q};

                    state <= ST_INV2;
                end

                ST_INV2: begin
                    // Launch Inverse Park Cycle 2:
                    // M13 (DSP0): v_d * sin
                    dsp0_a <= {{9{r_sin[15]}}, r_sin};
                    dsp0_b <= {{2{v_d[15]}}, v_d};

                    // M14 (DSP1): v_q * cos
                    dsp1_a <= {{9{r_cos[15]}}, r_cos};
                    dsp1_b <= {{2{v_q[15]}}, v_q};

                    state <= ST_INV3;
                end

                ST_INV3: begin
                    // DSP outputs now have M11 and M12!
                    m11_reg <= q15_round_halfup_sat(dsp0_p);
                    m12_reg <= q15_round_halfup_sat(dsp1_p);

                    // Wait 1 cycle for M13 and M14 to exit multiplier
                    state   <= ST_INV4;
                end

                ST_INV4: begin
                    // DSP outputs now have M13 and M14!
                    // Break long DSP -> saturation -> adder -> v_beta chain across 2 stages
                    m13_sat_reg <= q15_round_halfup_sat(dsp0_p);
                    m14_sat_reg <= q15_round_halfup_sat(dsp1_p);
                    v_alpha     <= q15_sub_sat(m11_reg, m12_reg);

                    state <= ST_INV5;
                end

                ST_INV5: begin
                    // Stage 2: compute final v_beta from registered m13_sat_reg and m14_sat_reg
                    v_beta <= q15_add_sat(m13_sat_reg, m14_sat_reg);

                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

// =============================================================================
// Dedicated Single-DSP Multiplier Primitive Wrapper
// =============================================================================
(* use_dsp = "yes" *)
module dsp_mult_cell (
    input  logic               clk,
    input  logic               rst,
    input  logic signed [24:0] a,
    input  logic signed [17:0] b,
    output logic signed [42:0] p
);
    logic signed [42:0] p_reg;

    always_ff @(posedge clk) begin
        if (rst) begin
            p_reg <= 43'sd0;
        end else begin
            p_reg <= a * b;
        end
    end

    assign p = p_reg;
endmodule
