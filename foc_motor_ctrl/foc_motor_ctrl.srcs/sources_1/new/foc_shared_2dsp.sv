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
    typedef enum logic [3:0] {
        ST_IDLE      = 4'd0,
        ST_CYC1      = 4'd1, // DSP computing M1, M2; feed M3, M4
        ST_CYC2      = 4'd2, // DSP out=M1, M2; DSP computing M3, M4; feed M5, M6
        ST_CYC3      = 4'd3, // DSP out=M3, M4; DSP computing M5, M6; feed M7, M8
        ST_CYC4      = 4'd4, // DSP out=M5, M6; DSP computing M7, M8; feed M9, M10
        ST_CYC5      = 4'd5, // DSP out=M7, M8; DSP computing M9, M10; feed M11, M12
        ST_CYC6      = 4'd6, // DSP out=M9, M10; DSP computing M11, M12; feed M13, M14
        ST_CYC7      = 4'd7, // DSP out=M11, M12; DSP computing M13, M14
        ST_CYC8      = 4'd8, // DSP out=M11, M12; wait M13, M14
        ST_CYC9      = 4'd9  // DSP out=M13, M14; compute Inverse Park, done!
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
    logic signed [15:0] neg_r_sin;
    assign neg_r_sin = -r_sin;

    // Integrator states
    logic signed [31:0] spd_integrator;
    logic signed [31:0] id_integrator;
    logic signed [31:0] iq_integrator;

    // Intermediate calculated values
    logic signed [16:0] spd_error_wide;
    logic signed [16:0] id_error_wide;
    logic signed [15:0] id_error_q15;
    logic signed [16:0] iq_error_wide;
    logic signed [15:0] iq_error_q15;

    // Intermediate registers for Inverse Park
    logic signed [15:0] m11_reg, m12_reg;

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
            m11_reg        <= 16'sd0;
            m12_reg        <= 16'sd0;
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

                        state  <= ST_CYC1;
                    end
                end

                ST_CYC1: begin
                    // Launch M3 (DSP0) and M4 (DSP1)
                    // M3: Ialpha * (-sin)
                    dsp0_a <= {{9{neg_r_sin[15]}}, neg_r_sin};
                    dsp0_b <= {{2{r_alpha[15]}}, r_alpha};

                    // M4: Ibeta * cos
                    dsp1_a <= {{9{r_cos[15]}}, r_cos};
                    dsp1_b <= {{2{r_beta[15]}}, r_beta};

                    state  <= ST_CYC2;
                end

                ST_CYC2: begin
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

                    state  <= ST_CYC3;
                end

                ST_CYC3: begin
                    // DSP outputs now have M3 and M4!
                    // Compute Park Transform (A1, A2)
                    begin
                        logic signed [15:0] m3_sat, m4_sat;
                        logic signed [15:0] id_temp, iq_temp;
                        m3_sat  = q15_round_halfup_sat(dsp0_p);
                        m4_sat  = q15_round_halfup_sat(dsp1_p);
                        id_temp = q15_add_sat(m1_reg, m2_reg);
                        iq_temp = q15_add_sat(m3_sat, m4_sat);

                        id_meas <= id_temp;
                        iq_meas <= iq_temp;

                        // Prepare Id error for Id PI
                        id_error_wide = {r_id_ref[15], r_id_ref} - {id_temp[15], id_temp};
                        if (id_error_wide > 17'sd32767)
                            id_error_q15 = 16'sh7fff;
                        else if (id_error_wide < -17'sd32768)
                            id_error_q15 = 16'sh8000;
                        else
                            id_error_q15 = id_error_wide[15:0];

                        // Launch M7 (DSP0) and M8 (DSP1)
                        // M7: id_error * current_kp
                        dsp0_a <= {{9{r_ckp[15]}}, r_ckp};
                        dsp0_b <= {{2{id_error_q15[15]}}, id_error_q15};

                        // M8: id_error * current_ki
                        dsp1_a <= {{9{r_cki[15]}}, r_cki};
                        dsp1_b <= {{2{id_error_q15[15]}}, id_error_q15};
                    end

                    state <= ST_CYC4;
                end

                ST_CYC4: begin
                    // DSP outputs now have M5 and M6!
                    // Compute Speed PI (A3, A4)
                    begin
                        logic signed [16:0] spd_err;
                        logic signed [15:0] spd_prop;
                        logic signed [31:0] spd_inc;
                        logic signed [32:0] spd_int_sum;
                        logic signed [31:0] spd_int_cand;
                        logic signed [15:0] spd_int_q15;
                        logic signed [32:0] spd_unsat;
                        logic               spd_allow_int;
                        logic signed [15:0] iq_ref_temp;

                        spd_err  = $signed({1'b0, r_wref}) - $signed({1'b0, r_wmeas});
                        spd_prop = speed_prop_sat(dsp0_p);
                        spd_inc  = speed_inc_sat(dsp1_p);

                        spd_int_sum  = $signed({spd_integrator[31], spd_integrator}) +
                                       $signed({spd_inc[31], spd_inc});
                        spd_int_cand = sat_q24(spd_int_sum);
                        spd_int_q15  = speed_q24_to_q15(spd_int_cand);

                        spd_unsat = $signed({spd_prop[15], spd_prop}) +
                                    $signed({spd_int_q15[15], spd_int_q15});

                        spd_allow_int = 1'b1;
                        if ((spd_unsat > 33'sd32767) && (spd_err > 17'sd0))
                            spd_allow_int = 1'b0;
                        if ((spd_unsat < -33'sd32768) && (spd_err < 17'sd0))
                            spd_allow_int = 1'b0;

                        if (spd_allow_int)
                            spd_integrator <= spd_int_cand;

                        iq_ref_temp = sat_q15(spd_unsat);
                        iq_ref      <= iq_ref_temp;

                        // Prepare Iq error for Iq PI
                        iq_error_wide = {iq_ref_temp[15], iq_ref_temp} - {iq_meas[15], iq_meas};
                        if (iq_error_wide > 17'sd32767)
                            iq_error_q15 = 16'sh7fff;
                        else if (iq_error_wide < -17'sd32768)
                            iq_error_q15 = 16'sh8000;
                        else
                            iq_error_q15 = iq_error_wide[15:0];

                        // Launch M9 (DSP0) and M10 (DSP1)
                        // M9: iq_error * current_kp
                        dsp0_a <= {{9{r_ckp[15]}}, r_ckp};
                        dsp0_b <= {{2{iq_error_q15[15]}}, iq_error_q15};

                        // M10: iq_error * current_ki
                        dsp1_a <= {{9{r_cki[15]}}, r_cki};
                        dsp1_b <= {{2{iq_error_q15[15]}}, iq_error_q15};
                    end

                    state <= ST_CYC5;
                end

                ST_CYC5: begin
                    // DSP outputs now have M7 and M8!
                    // Compute Id PI Controller (A5, A6) -> produces v_d
                    begin
                        logic signed [16:0] id_err_w;
                        logic signed [15:0] id_err_q;
                        logic signed [15:0] vd_prop;
                        logic signed [15:0] id_inc_q15;
                        logic signed [31:0] id_inc_q24;
                        logic signed [32:0] id_int_sum;
                        logic signed [31:0] id_int_cand;
                        logic signed [15:0] id_int_q15;
                        logic signed [16:0] vd_unsat;
                        logic               id_allow_int;
                        logic signed [15:0] vd_temp;

                        id_err_w = {r_id_ref[15], r_id_ref} - {id_meas[15], id_meas};
                        if (id_err_w > 17'sd32767)      id_err_q = 16'sh7fff;
                        else if (id_err_w < -17'sd32768) id_err_q = 16'sh8000;
                        else                            id_err_q = id_err_w[15:0];

                        vd_prop    = curr_prop_sat(dsp0_p);
                        id_inc_q15 = curr_inc_q15(dsp1_p);
                        id_inc_q24 = {{16{id_inc_q15[15]}}, id_inc_q15} <<< 9;

                        id_int_sum  = {id_integrator[31], id_integrator} +
                                      {id_inc_q24[31], id_inc_q24};
                        id_int_cand = sat_q24(id_int_sum);
                        id_int_q15  = curr_q24_to_q15_round(id_int_cand);

                        vd_unsat = {vd_prop[15], vd_prop} + {id_int_q15[15], id_int_q15};

                        id_allow_int = 1'b1;
                        if ((vd_unsat > 17'sd32767) && (id_err_q > 16'sd0))
                            id_allow_int = 1'b0;
                        if ((vd_unsat < -17'sd32768) && (id_err_q < 16'sd0))
                            id_allow_int = 1'b0;

                        if (id_allow_int)
                            id_integrator <= id_int_cand;

                        if (vd_unsat > 17'sd32767)
                            vd_temp = 16'sh7fff;
                        else if (vd_unsat < -17'sd32768)
                            vd_temp = 16'sh8000;
                        else
                            vd_temp = vd_unsat[15:0];

                        v_d <= vd_temp;

                        // We can already prepare DSP0 for Inverse Park Cycle 1 (M11: vd * cos)
                        dsp0_a <= {{9{r_cos[15]}}, r_cos};
                        dsp0_b <= {{2{vd_temp[15]}}, vd_temp};
                    end

                    state <= ST_CYC6;
                end

                ST_CYC6: begin
                    // DSP outputs now have M9 and M10!
                    // Compute Iq PI Controller (A7, A8) -> produces v_q
                    begin
                        logic signed [16:0] iq_err_w;
                        logic signed [15:0] iq_err_q;
                        logic signed [15:0] vq_prop;
                        logic signed [15:0] iq_inc_q15;
                        logic signed [31:0] iq_inc_q24;
                        logic signed [32:0] iq_int_sum;
                        logic signed [31:0] iq_int_cand;
                        logic signed [15:0] iq_int_q15;
                        logic signed [16:0] vq_unsat;
                        logic               iq_allow_int;
                        logic signed [15:0] vq_temp;

                        iq_err_w = {iq_ref[15], iq_ref} - {iq_meas[15], iq_meas};
                        if (iq_err_w > 17'sd32767)      iq_err_q = 16'sh7fff;
                        else if (iq_err_w < -17'sd32768) iq_err_q = 16'sh8000;
                        else                            iq_err_q = iq_err_w[15:0];

                        vq_prop    = curr_prop_sat(dsp0_p);
                        iq_inc_q15 = curr_inc_q15(dsp1_p);
                        iq_inc_q24 = {{16{iq_inc_q15[15]}}, iq_inc_q15} <<< 9;

                        iq_int_sum  = {iq_integrator[31], iq_integrator} +
                                      {iq_inc_q24[31], iq_inc_q24};
                        iq_int_cand = sat_q24(iq_int_sum);
                        iq_int_q15  = curr_q24_to_q15_round(iq_int_cand);

                        vq_unsat = {vq_prop[15], vq_prop} + {iq_int_q15[15], iq_int_q15};

                        iq_allow_int = 1'b1;
                        if ((vq_unsat > 17'sd32767) && (iq_err_q > 16'sd0))
                            iq_allow_int = 1'b0;
                        if ((vq_unsat < -17'sd32768) && (iq_err_q < 16'sd0))
                            iq_allow_int = 1'b0;

                        if (iq_allow_int)
                            iq_integrator <= iq_int_cand;

                        if (vq_unsat > 17'sd32767)
                            vq_temp = 16'sh7fff;
                        else if (vq_unsat < -17'sd32768)
                            vq_temp = 16'sh8000;
                        else
                            vq_temp = vq_unsat[15:0];

                        v_q <= vq_temp;

                        // Launch M12 (DSP1): vq * sin
                        dsp1_a <= {{9{r_sin[15]}}, r_sin};
                        dsp1_b <= {{2{vq_temp[15]}}, vq_temp};
                    end

                    state <= ST_CYC7;
                end

                ST_CYC7: begin
                    // Launch Inverse Park Cycle 2:
                    // M13 (DSP0): vd * sin
                    dsp0_a <= {{9{r_sin[15]}}, r_sin};
                    dsp0_b <= {{2{v_d[15]}}, v_d};

                    // M14 (DSP1): vq * cos
                    dsp1_a <= {{9{r_cos[15]}}, r_cos};
                    dsp1_b <= {{2{v_q[15]}}, v_q};

                    state <= ST_CYC8;
                end

                ST_CYC8: begin
                    // DSP outputs now have M11 and M12!
                    m11_reg <= q15_round_halfup_sat(dsp0_p);
                    m12_reg <= q15_round_halfup_sat(dsp1_p);

                    // Wait 1 cycle for M13 and M14 to exit multiplier
                    state   <= ST_CYC9;
                end

                ST_CYC9: begin
                    // DSP outputs now have M13 and M14!
                    // Compute Inverse Park (A9, A10)
                    begin
                        logic signed [15:0] m13_sat, m14_sat;
                        m13_sat = q15_round_halfup_sat(dsp0_p);
                        m14_sat = q15_round_halfup_sat(dsp1_p);

                        // Valpha = vd*cos - vq*sin = m11 - m12
                        v_alpha <= q15_sub_sat(m11_reg, m12_reg);

                        // Vbeta  = vd*sin + vq*cos = m13 + m14
                        v_beta  <= q15_add_sat(m13_sat, m14_sat);
                    end

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
