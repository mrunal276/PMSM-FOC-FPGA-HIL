module speed_pi (
    input  logic         clk,
    input  logic         rst,
    input  logic         input_valid,

    // Speed signals: unsigned Q10.6
    // 1.0 rad/s = 64 counts
    input  logic [15:0]  omega_ref,
    input  logic [15:0]  omega_meas,

    // Kp: signed Q1.15
    input  logic signed [15:0] kp,

    // Ki*dt: signed Q1.31
    // For Ki=0.5 and dt=50us:
    // round(0.5*50e-6*2^31) = 53687
    input  logic signed [31:0] ki_dt_q31,

    // iq_ref: signed Q1.15
    output logic signed [15:0] iq_ref,
    output logic               output_valid
);

    localparam logic signed [15:0] Q15_MAX = 16'sh7fff;
    localparam logic signed [15:0] Q15_MIN = 16'sh8000;

    // Q8.24 integrator state
    logic signed [31:0] integrator;

    // ----------------------------------------------------------------
    // Saturate signed value to Q1.15
    // ----------------------------------------------------------------
    function automatic logic signed [15:0] sat_q15(
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

    // ----------------------------------------------------------------
    // Q10.6 speed error -> proportional Q1.15
    //
    // error = error_int / 2^6
    // kp    = kp_int    / 2^15
    //
    // product to Q1.15 therefore requires >> 6.
    //
    // Sign-aware rounding matches the fixed-point PI convention.
    // ----------------------------------------------------------------
    function automatic logic signed [15:0] proportional_q15(
        input logic signed [16:0] error,
        input logic signed [15:0] gain
    );
        logic signed [32:0] product;
        logic signed [32:0] rounded;
        logic signed [32:0] shifted;

        begin
            product = error * gain;

            if (product >= 0)
                rounded = product + 33'sd32;
            else
                rounded = product - 33'sd32;

            shifted = rounded >>> 6;

            proportional_q15 = sat_q15(shifted);
        end
    endfunction

    // ----------------------------------------------------------------
    // Ki*error*dt:
    //
    // error    = Q10.6
    // ki_dt    = Q1.31
    // result   = Q8.24
    //
    // Required shift:
    //   31 + 6 - 24 = 13
    // ----------------------------------------------------------------
    function automatic logic signed [31:0] integrator_increment(
        input logic signed [16:0] error,
        input logic signed [31:0] ki_dt
    );
        logic signed [48:0] product;
        logic signed [48:0] rounded;
        logic signed [48:0] shifted;

        begin
            product = error * ki_dt;

            if (product >= 0)
                rounded = product + 49'sd4096;
            else
                rounded = product - 49'sd4096;

            shifted = rounded >>> 13;

            // Q8.24 increment is saturated to signed 32-bit range.
            if (shifted > 49'sd2147483647)
                integrator_increment = 32'sh7fffffff;
            else if (shifted < -49'sd2147483648)
                integrator_increment = 32'sh80000000;
            else
                integrator_increment = shifted[31:0];
        end
    endfunction

    // ----------------------------------------------------------------
    // Q8.24 -> Q1.15
    // ----------------------------------------------------------------
    function automatic logic signed [15:0] q24_to_q15(
        input logic signed [31:0] value
    );
        logic signed [32:0] rounded;
        logic signed [32:0] shifted;

        begin
            if (value >= 0)
                rounded = {value[31], value} + 33'sd256;
            else
                rounded = {value[31], value} - 33'sd256;

            shifted = rounded >>> 9;

            q24_to_q15 = sat_q15(shifted);
        end
    endfunction

    // ----------------------------------------------------------------
    // Main PI update
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin : pi_update
        logic signed [16:0] error_wide;

        logic signed [15:0] error_q15_unused;
        logic signed [15:0] proportional;
        logic signed [31:0] increment;
        logic signed [32:0] integrator_sum;
        logic signed [31:0] integrator_candidate;
        logic signed [15:0] integrator_q15;

        logic signed [32:0] unsaturated_output;

        logic allow_integrator;

        if (rst) begin
            integrator  <= 32'sd0;
            iq_ref      <= 16'sd0;
            output_valid <= 1'b0;
        end
        else begin
            output_valid <= 1'b0;

            if (input_valid) begin

                // ----------------------------------------------------
                // Signed speed error.
                // Both inputs are unsigned Q10.6.
                // ----------------------------------------------------
                error_wide =
                    $signed({1'b0, omega_ref}) -
                    $signed({1'b0, omega_meas});

                // ----------------------------------------------------
                // Proportional path
                // ----------------------------------------------------
                proportional =
                    proportional_q15(error_wide, kp);

                // ----------------------------------------------------
                // Integral candidate
                // ----------------------------------------------------
                increment =
                    integrator_increment(error_wide, ki_dt_q31);

                integrator_sum =
                    $signed({integrator[31], integrator}) +
                    $signed({increment[31], increment});

                // Full signed Q8.24 saturation.
                if (integrator_sum > 33'sd2147483647)
                    integrator_candidate = 32'sh7fffffff;
                else if (integrator_sum < -33'sd2147483648)
                    integrator_candidate = 32'sh80000000;
                else
                    integrator_candidate = integrator_sum[31:0];

                integrator_q15 =
                    q24_to_q15(integrator_candidate);

                // ----------------------------------------------------
                // P + I
                // ----------------------------------------------------
                unsaturated_output =
                    $signed({proportional[15], proportional}) +
                    $signed({integrator_q15[15], integrator_q15});

                // ----------------------------------------------------
                // Conditional anti-windup
                // ----------------------------------------------------
                allow_integrator = 1'b1;

                if ((unsaturated_output > 33'sd32767) &&
                    (error_wide > 17'sd0))
                    allow_integrator = 1'b0;

                if ((unsaturated_output < -33'sd32768) &&
                    (error_wide < 17'sd0))
                    allow_integrator = 1'b0;

                if (allow_integrator)
                    integrator <= integrator_candidate;

                // ----------------------------------------------------
                // Saturated output
                // ----------------------------------------------------
                iq_ref <= sat_q15(unsaturated_output);

                output_valid <= 1'b1;
            end
        end
    end

endmodule