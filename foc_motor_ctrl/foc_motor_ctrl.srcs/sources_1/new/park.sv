module park (
    input  logic               clk,
    input  logic               rst,
    input  logic               input_valid,

    input  logic signed [15:0] i_alpha,
    input  logic signed [15:0] i_beta,

    input  logic signed [15:0] cos_theta,
    input  logic signed [15:0] sin_theta,

    output logic               output_valid,
    output logic signed [15:0] i_d,
    output logic signed [15:0] i_q
);

    localparam int Q15_MAX = 32767;
    localparam int Q15_MIN = -32768;

    /*
     * Q1.15 multiplication:
     *
     * Q1.15 × Q1.15 = Q2.30
     * + 2^14 for round-to-nearest
     * >>> 15 -> Q1.15
     * saturate to Q1.15 range
     */
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

    /*
     * Saturating Q1.15 addition.
     */
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

    logic signed [15:0] id_next;
    logic signed [15:0] iq_next;

    always_comb begin
        /*
         * Id = Ialpha*cos(theta) + Ibeta*sin(theta)
         */
        id_next = q15_add_sat(
            q15_mul_sat(i_alpha, cos_theta),
            q15_mul_sat(i_beta,  sin_theta)
        );

        /*
         * Iq = -Ialpha*sin(theta) + Ibeta*cos(theta)
         */
        iq_next = q15_add_sat(
            q15_mul_sat(i_alpha, -sin_theta),
            q15_mul_sat(i_beta,  cos_theta)
        );
    end

    /*
     * One-cycle registered interface.
     */
    always_ff @(posedge clk) begin
        if (rst) begin
            output_valid <= 1'b0;
            i_d          <= 16'sd0;
            i_q          <= 16'sd0;
        end
        else begin
            output_valid <= input_valid;

            if (input_valid) begin
                i_d <= id_next;
                i_q <= iq_next;
            end
        end
    end

endmodule