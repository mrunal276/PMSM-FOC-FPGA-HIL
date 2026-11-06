module clarke (
    input  logic               clk,
    input  logic               rst,
    input  logic               input_valid,

    input  logic signed [15:0] ia_in,
    input  logic signed [15:0] ib_in,
    input  logic signed [15:0] ic_in,

    output logic               output_valid,
    output logic signed [15:0] i_alpha,
    output logic signed [15:0] i_beta
);

    // Q1.15 constant:
    // 1/sqrt(3) = 0.577350269...
    // round(0.577350269 * 32768) = 18919
    localparam logic signed [15:0] INV_SQRT3_Q15 = 16'sd18919;

    // ------------------------------------------------------------------------
    // Q1.15 multiplication
    //
    // Q1.15 × Q1.15 = Q2.30
    // Round-to-nearest by adding 2^14.
    // Shift right by 15 to return to Q1.15.
    // Saturate to signed Q1.15 range.
    // ------------------------------------------------------------------------
    function automatic signed [15:0] q15_mul (
        input logic signed [15:0] a,
        input logic signed [15:0] b
    );
        logic signed [31:0] product;
        logic signed [31:0] rounded;
        logic signed [31:0] shifted;

        begin
            product = a * b;

            // Round-to-nearest.
            rounded = product + 32'sd16384;

            // Q2.30 -> Q1.15
            shifted = rounded >>> 15;

            // Saturation.
            if (shifted > 32'sd32767)
                q15_mul = 16'sh7fff;
            else if (shifted < -32'sd32768)
                q15_mul = 16'sh8000;
            else
                q15_mul = shifted[15:0];
        end
    endfunction

    // ------------------------------------------------------------------------
    // Saturating Q1.15 addition
    // ------------------------------------------------------------------------
    function automatic signed [15:0] q15_add (
        input logic signed [15:0] a,
        input logic signed [15:0] b
    );
        logic signed [16:0] sum;

        begin
            sum = {a[15], a} + {b[15], b};

            if (sum > 17'sd32767)
                q15_add = 16'sh7fff;
            else if (sum < -17'sd32768)
                q15_add = 16'sh8000;
            else
                q15_add = sum[15:0];
        end
    endfunction

    // Intermediate terms for:
    //
    // Ibeta = Ia/sqrt(3)
    //       + Ib/sqrt(3)
    //       + Ib/sqrt(3)
    //
    // This intentionally mirrors the frozen Python golden implementation.
    logic signed [15:0] term_ia;
    logic signed [15:0] term_ib1;
    logic signed [15:0] term_ib2;

    logic signed [15:0] beta_sum_1;
    logic signed [15:0] beta_sum_2;

    always_comb begin

        // Ia / sqrt(3)
        term_ia = q15_mul(
            ia_in,
            INV_SQRT3_Q15
        );

        // Ib / sqrt(3)
        term_ib1 = q15_mul(
            ib_in,
            INV_SQRT3_Q15
        );

        // Second copy of Ib / sqrt(3)
        term_ib2 = q15_mul(
            ib_in,
            INV_SQRT3_Q15
        );

        // Saturating additions, exactly following golden model.
        beta_sum_1 = q15_add(
            term_ia,
            term_ib1
        );

        beta_sum_2 = q15_add(
            beta_sum_1,
            term_ib2
        );
    end

    // ------------------------------------------------------------------------
    // Registered Clarke output
    //
    // One-cycle interface latency:
    // input_valid at cycle N
    // output_valid at cycle N+1
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            output_valid <= 1'b0;
            i_alpha      <= 16'sd0;
            i_beta       <= 16'sd0;
        end
        else begin
            output_valid <= input_valid;

            if (input_valid) begin
                // Ialpha = Ia
                //
                // Ia is already Q1.15, so this is a saturated pass-through.
                if (ia_in > 16'sh7fff)
                    i_alpha <= 16'sh7fff;
                else if (ia_in < 16'sh8000)
                    i_alpha <= 16'sh8000;
                else
                    i_alpha <= ia_in;

                i_beta <= beta_sum_2;
            end
        end
    end

    // ic_in is intentionally unused.
    // Clarke uses the balanced-system relation:
    //
    //     Ia + Ib + Ic = 0
    //
    // and therefore the frozen golden implementation computes beta from
    // Ia and Ib only.

endmodule