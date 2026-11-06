module current_pi #(
    parameter int KP_FRAC = 15 // Set KP_FRAC=12 for Q4.12 gains (e.g. Kp=3.0), or 15 for Q1.15
)(
    input  logic               clk,
    input  logic               rst,
    input  logic               input_valid,

    // Q1.15 reference and measured current
    input  logic signed [15:0] ref_in,
    input  logic signed [15:0] meas_in,

    // Gain format: signed [15:0] with KP_FRAC fractional bits
    input  logic signed [15:0] kp,
    input  logic signed [15:0] ki,

    // Q1.15 output
    output logic signed [15:0] output_data,
    output logic               output_valid
);

    localparam logic signed [15:0] Q15_MAX = 16'sh7fff;
    localparam logic signed [15:0] Q15_MIN = 16'sh8000;

    localparam logic signed [31:0] Q24_MAX = 32'sh7fffffff;
    localparam logic signed [31:0] Q24_MIN = 32'sh80000000;


    // ============================================================
    // Q1.15 multiply
    //
    // Golden-model behavior:
    //
    //   product = a*b
    //   if product >= 0: product += 2^14
    //   else:            product -= 2^14
    //   result = product >>> 15
    //
    // This is sign-symmetric round-to-nearest.
    // ============================================================

    function automatic signed [15:0] q15_mul (
        input logic signed [15:0] a,
        input logic signed [15:0] b
    );

        logic signed [31:0] product;
        logic signed [31:0] rounded;
        logic signed [31:0] shifted;

        begin
            product = a * b;

            if (product >= 0)
                rounded = product + 32'sd16384;
            else
                rounded = product - 32'sd16384;

            shifted = rounded >>> 15;

            if (shifted > 32'sd32767)
                q15_mul = Q15_MAX;
            else if (shifted < -32'sd32768)
                q15_mul = Q15_MIN;
            else
                q15_mul = shifted[15:0];
        end

    endfunction


    // ============================================================
    // Parameterized proportional multiply: (kp * error) -> Q1.15
    // Supports arbitrary KP_FRAC fractional bits (e.g. 12 or 15)
    // ============================================================
    function automatic signed [15:0] kp_mul_sat (
        input logic signed [15:0] gain,
        input logic signed [15:0] err
    );
        logic signed [31:0] product;
        logic signed [31:0] rounded;
        logic signed [31:0] shifted;
        begin
            product = gain * err;

            if (product >= 0)
                rounded = product + (32'sd1 <<< (KP_FRAC - 1));
            else
                rounded = product - (32'sd1 <<< (KP_FRAC - 1));

            shifted = rounded >>> KP_FRAC;

            if (shifted > 32'sd32767)
                kp_mul_sat = Q15_MAX;
            else if (shifted < -32'sd32768)
                kp_mul_sat = Q15_MIN;
            else
                kp_mul_sat = shifted[15:0];
        end
    endfunction


    // ============================================================
    // Q1.15 saturating conversion of a signed wider value.
    // ============================================================

    function automatic signed [15:0] sat_q15 (
        input logic signed [32:0] value
    );

        begin
            if (value > 33'sd32767)
                sat_q15 = Q15_MAX;
            else if (value < -33'sd32768)
                sat_q15 = Q15_MIN;
            else
                sat_q15 = value[15:0];
        end

    endfunction


    // ============================================================
    // Saturating Q1.15 addition
    // ============================================================

    function automatic signed [15:0] q15_add_sat (
        input logic signed [15:0] a,
        input logic signed [15:0] b
    );

        logic signed [16:0] sum;

        begin
            sum = {a[15], a} + {b[15], b};

            if (sum > 17'sd32767)
                q15_add_sat = Q15_MAX;
            else if (sum < -17'sd32768)
                q15_add_sat = Q15_MIN;
            else
                q15_add_sat = sum[15:0];
        end

    endfunction


    // ============================================================
    // Convert Q8.24 to Q1.15 with sign-symmetric round-to-nearest,
    // followed by Q1.15 saturation.
    //
    // Q8.24 has 24 fractional bits.
    // Q1.15 has 15 fractional bits.
    // Difference is 9 bits.
    //
    // Rounding offset = 2^8 = 256.
    // ============================================================

    function automatic signed [15:0] q24_to_q15_round (
        input logic signed [31:0] value_q24
    );

        logic signed [31:0] rounded;
        logic signed [31:0] shifted;

        begin
            if (value_q24 >= 0)
                rounded = value_q24 + 32'sd256;
            else
                rounded = value_q24 - 32'sd256;

            shifted = rounded >>> 9;

            if (shifted > 32'sd32767)
                q24_to_q15_round = Q15_MAX;
            else if (shifted < -32'sd32768)
                q24_to_q15_round = Q15_MIN;
            else
                q24_to_q15_round = shifted[15:0];
        end

    endfunction


    // ============================================================
    // Q8.24 saturating integrator.
    // ============================================================

    function automatic signed [31:0] sat_q24 (
        input logic signed [32:0] value
    );

        begin
            if (value > 33'sd2147483647)
                sat_q24 = Q24_MAX;
            else if (value < -33'sd2147483648)
                sat_q24 = Q24_MIN;
            else
                sat_q24 = value[31:0];
        end

    endfunction


    // ============================================================
    // Internal state and combinational datapath
    // ============================================================

    logic signed [31:0] integrator;
    logic signed [15:0] error_q15;
    logic signed [15:0] proportional_q15;
    logic signed [15:0] integral_increment_q15;
    logic signed [15:0] integrator_q15;

    logic signed [31:0] integral_increment_q24;
    logic signed [31:0] integrator_candidate;

    logic signed [32:0] integrator_sum_wide;

    logic signed [16:0] unsaturated_wide;
    logic signed [15:0] output_q15;

    logic allow_integrator;

    logic signed [16:0] error_wide;


    always_comb begin

        // --------------------------------------------------------
        // Error = reference - feedback
        // Then saturate to Q1.15.
        // --------------------------------------------------------

        error_wide =
            {ref_in[15], ref_in} -
            {meas_in[15], meas_in};

        if (error_wide > 17'sd32767)
            error_q15 = Q15_MAX;
        else if (error_wide < -17'sd32768)
            error_q15 = Q15_MIN;
        else
            error_q15 = error_wide[15:0];


        // --------------------------------------------------------
        // Proportional term
        // Scaled with KP_FRAC
        // --------------------------------------------------------

        proportional_q15 =
            kp_mul_sat(kp, error_q15);


        // --------------------------------------------------------
        // Integral increment
        // Q1.15 * Q1.15 -> Q1.15
        // then Q1.15 -> Q8.24 by << 9.
        //
        // Current-loop Ki is per-update.
        // No dt multiplication.
        // --------------------------------------------------------

        integral_increment_q15 =
            q15_mul(ki, error_q15);

        integral_increment_q24 =
            {{16{integral_increment_q15[15]}},
             integral_increment_q15} <<< 9;


        // --------------------------------------------------------
        // Candidate integrator
        // --------------------------------------------------------

        integrator_sum_wide =
            {integrator[31], integrator} +
            {integral_increment_q24[31],
             integral_increment_q24};

        integrator_candidate =
            sat_q24(integrator_sum_wide);


        // --------------------------------------------------------
        // Convert candidate integrator to Q1.15.
        // --------------------------------------------------------

        integrator_q15 =
            q24_to_q15_round(integrator_candidate);


        // --------------------------------------------------------
        // Raw / unsaturated output.
        //
        // IMPORTANT:
        // This is deliberately wider than Q1.15 so saturation
        // detection can see the overflow.
        // --------------------------------------------------------

        unsaturated_wide =
            {proportional_q15[15], proportional_q15} +
            {integrator_q15[15], integrator_q15};


        // --------------------------------------------------------
        // Output saturation
        // --------------------------------------------------------

        if (unsaturated_wide > 17'sd32767)
            output_q15 = Q15_MAX;
        else if (unsaturated_wide < -17'sd32768)
            output_q15 = Q15_MIN;
        else
            output_q15 = unsaturated_wide[15:0];


        // --------------------------------------------------------
        // Conditional-integration anti-windup.
        //
        // Golden model:
        //
        // saturated_high && error > 0  -> don't integrate
        // saturated_low  && error < 0  -> don't integrate
        // otherwise                      integrate
        // --------------------------------------------------------

        allow_integrator = 1'b1;

        if ((unsaturated_wide > 17'sd32767) &&
            (error_q15 > 16'sd0))
            allow_integrator = 1'b0;

        if ((unsaturated_wide < -17'sd32768) &&
            (error_q15 < 16'sd0))
            allow_integrator = 1'b0;

    end


    // ============================================================
    // Sequential state update
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin
            integrator   <= 32'sd0;
            output_data  <= 16'sd0;
            output_valid <= 1'b0;
        end

        else begin

            output_valid <= input_valid;

            if (input_valid) begin

                output_data <= output_q15;

                if (allow_integrator)
                    integrator <= integrator_candidate;

            end

        end

    end

endmodule
