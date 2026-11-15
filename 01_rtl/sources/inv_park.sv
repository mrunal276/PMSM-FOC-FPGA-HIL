module inv_park (
    input  logic               clk,
    input  logic               rst,
    input  logic               input_valid,

    input  logic signed [15:0] v_d,
    input  logic signed [15:0] v_q,

    input  logic signed [15:0] cos_theta,
    input  logic signed [15:0] sin_theta,

    output logic               output_valid,
    output logic signed [15:0] v_alpha,
    output logic signed [15:0] v_beta
);

    localparam int Q15_MAX = 32767;
    localparam int Q15_MIN = -32768;

    // ------------------------------------------------------------
    // Q1.15 multiply with round-to-nearest and saturation.
    //
    // product : Q2.30
    // +16384  : rounding offset = 2^14
    // >>>15   : back to Q1.15
    // ------------------------------------------------------------
    function automatic signed [15:0] q15_mul_sat (
        input logic signed [15:0] a,
        input logic signed [15:0] b
    );
        logic signed [31:0] product;
        logic signed [31:0] rounded;
        logic signed [31:0] shifted;

        begin
            product = a * b;
            rounded = product + 32'sd16384;
            shifted = rounded >>> 15;

            if (shifted > Q15_MAX)
                q15_mul_sat = 16'sh7fff;
            else if (shifted < Q15_MIN)
                q15_mul_sat = 16'sh8000;
            else
                q15_mul_sat = shifted[15:0];
        end
    endfunction

    // ------------------------------------------------------------
    // Q1.15 saturating addition.
    // ------------------------------------------------------------
    function automatic signed [15:0] q15_add_sat (
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

    // ------------------------------------------------------------
    // Q1.15 saturating subtraction.
    //
    // V_alpha = Vd*cos - Vq*sin
    // ------------------------------------------------------------
    function automatic signed [15:0] q15_sub_sat (
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

    logic signed [15:0] vd_cos;
    logic signed [15:0] vq_sin;
    logic signed [15:0] vd_sin;
    logic signed [15:0] vq_cos;

    logic signed [15:0] valpha_next;
    logic signed [15:0] vbeta_next;

    always_comb begin

        vd_cos = q15_mul_sat(v_d, cos_theta);
        vq_sin = q15_mul_sat(v_q, sin_theta);

        vd_sin = q15_mul_sat(v_d, sin_theta);
        vq_cos = q15_mul_sat(v_q, cos_theta);

        valpha_next = q15_sub_sat(vd_cos, vq_sin);
        vbeta_next  = q15_add_sat(vd_sin, vq_cos);

    end

    // ------------------------------------------------------------
    // One-cycle registered output.
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            output_valid <= 1'b0;
            v_alpha      <= 16'sd0;
            v_beta       <= 16'sd0;
        end
        else begin
            output_valid <= input_valid;

            if (input_valid) begin
                v_alpha <= valpha_next;
                v_beta  <= vbeta_next;
            end
        end
    end

endmodule