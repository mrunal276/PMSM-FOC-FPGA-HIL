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
    // Q1.15 rounding and saturation from Q2.30 product
    //
    // Q1.15 × Q1.15 = Q2.30
    // Round-to-nearest by adding 2^14.
    // Shift right by 15 to return to Q1.15.
    // Saturate to signed Q1.15 range.
    // ------------------------------------------------------------------------
    function automatic signed [15:0] q15_round_sat (
        input logic signed [31:0] product
    );
        logic signed [31:0] rounded;
        logic signed [31:0] shifted;

        begin
            // Round-to-nearest.
            rounded = product + 32'sd16384;

            // Q2.30 -> Q1.15
            shifted = rounded >>> 15;

            // Saturation.
            if (shifted > 32'sd32767)
                q15_round_sat = 16'sh7fff;
            else if (shifted < -32'sd32768)
                q15_round_sat = 16'sh8000;
            else
                q15_round_sat = shifted[15:0];
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

    // ------------------------------------------------------------------------
    // Pipelined Clarke Architecture (4 Stages for 125 MHz Timing Closure)
    //
    // Stage 1: DSP Multiplications (Ia * 1/sqrt3, Ib * 1/sqrt3) into DSP P-registers
    // Stage 2: Q1.15 rounding and saturation into slice registers
    // Stage 3: First saturating addition: sum1 = (ia/sqrt3 + ib/sqrt3)
    // Stage 4: Second saturating addition: beta = sum1 + ib/sqrt3; registered outputs
    //
    // Total latency: 4 clock cycles.
    // In foc_pipeline, CLARKE_DELAY is matched (4 + 14 = 18 cycles to CORDIC).
    // This intentionally mirrors the frozen Python golden implementation bit-truely.
    // ------------------------------------------------------------------------
    logic               stage1_valid;
    logic signed [15:0] stage1_ia;
    (* use_dsp = "yes" *) logic signed [31:0] stage1_prod_ia;
    (* use_dsp = "yes" *) logic signed [31:0] stage1_prod_ib;

    always_ff @(posedge clk) begin
        if (rst) begin
            stage1_valid   <= 1'b0;
            stage1_ia      <= 16'sd0;
            stage1_prod_ia <= 32'sd0;
            stage1_prod_ib <= 32'sd0;
        end else begin
            stage1_valid   <= input_valid;
            if (input_valid) begin
                stage1_ia      <= ia_in;
                stage1_prod_ia <= ia_in * INV_SQRT3_Q15;
                stage1_prod_ib <= ib_in * INV_SQRT3_Q15;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Stage 2: Q1.15 Rounding and Saturation
    // ------------------------------------------------------------------------
    logic               stage2_valid;
    logic signed [15:0] stage2_ia;
    logic signed [15:0] stage2_term_ia;
    logic signed [15:0] stage2_term_ib;

    always_ff @(posedge clk) begin
        if (rst) begin
            stage2_valid   <= 1'b0;
            stage2_ia      <= 16'sd0;
            stage2_term_ia <= 16'sd0;
            stage2_term_ib <= 16'sd0;
        end else begin
            stage2_valid   <= stage1_valid;
            if (stage1_valid) begin
                stage2_ia      <= stage1_ia;
                stage2_term_ia <= q15_round_sat(stage1_prod_ia);
                stage2_term_ib <= q15_round_sat(stage1_prod_ib);
            end
        end
    end

    // ------------------------------------------------------------------------
    // Stage 3: First Saturating Addition (sum1 = term_ia + term_ib)
    // ------------------------------------------------------------------------
    logic               stage3_valid;
    logic signed [15:0] stage3_ia;
    logic signed [15:0] stage3_sum1;
    logic signed [15:0] stage3_term_ib;

    always_ff @(posedge clk) begin
        if (rst) begin
            stage3_valid   <= 1'b0;
            stage3_ia      <= 16'sd0;
            stage3_sum1    <= 16'sd0;
            stage3_term_ib <= 16'sd0;
        end else begin
            stage3_valid   <= stage2_valid;
            if (stage2_valid) begin
                stage3_ia      <= stage2_ia;
                stage3_sum1    <= q15_add(stage2_term_ia, stage2_term_ib);
                stage3_term_ib <= stage2_term_ib;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Stage 4: Second Saturating Addition (beta = sum1 + term_ib) & Outputs
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            output_valid <= 1'b0;
            i_alpha      <= 16'sd0;
            i_beta       <= 16'sd0;
        end else begin
            output_valid <= stage3_valid;
            if (stage3_valid) begin
                if (stage3_ia > 16'sh7fff)
                    i_alpha <= 16'sh7fff;
                else if (stage3_ia < -16'sh8000)
                    i_alpha <= 16'sh8000;
                else
                    i_alpha <= stage3_ia;

                // beta = sum1 + ib/sqrt3
                i_beta <= q15_add(stage3_sum1, stage3_term_ib);
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
