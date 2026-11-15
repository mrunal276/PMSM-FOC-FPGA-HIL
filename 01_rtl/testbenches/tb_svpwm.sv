
`timescale 1ns/1ps

module tb_svpwm;

    logic clk;
    logic rst;
    logic input_valid;

    logic signed [15:0] v_alpha;
    logic signed [15:0] v_beta;

    logic [15:0] duty_a;
    logic [15:0] duty_b;
    logic [15:0] duty_c;
    logic [2:0] sector;
    logic output_valid;

    integer vectors_tested;
    integer failures;
    integer max_error;

    localparam integer DUTY_TOLERANCE = 1;

    svpwm dut (
        .clk(clk),
        .rst(rst),
        .input_valid(input_valid),
        .v_alpha(v_alpha),
        .v_beta(v_beta),
        .duty_a(duty_a),
        .duty_b(duty_b),
        .duty_c(duty_c),
        .sector(sector),
        .output_valid(output_valid)
    );

    // 100 MHz clock
    initial clk = 1'b0;
    always #5 clk = ~clk;

    task automatic apply_golden_vector(
        input integer alpha,
        input integer beta,
        input integer expected_sector,
        input integer expected_a,
        input integer expected_b,
        input integer expected_c
    );
        integer error_a;
        integer error_b;
        integer error_c;
        integer local_max_error;

        begin
            @(negedge clk);

            v_alpha = alpha;
            v_beta = beta;
            input_valid = 1'b1;

            // SVPWM now has 2-cycle latency: wait 2 rising edges before sampling
            @(posedge clk);
            @(posedge clk);
            #1;

            vectors_tested = vectors_tested + 1;

            if (output_valid !== 1'b1) begin
                $display(
                    "FAIL: output_valid missing for vector %0d",
                    vectors_tested
                );
                failures = failures + 1;
            end
            else begin
                error_a = $signed({1'b0, duty_a}) - expected_a;
                error_b = $signed({1'b0, duty_b}) - expected_b;
                error_c = $signed({1'b0, duty_c}) - expected_c;

                if (error_a < 0) error_a = -error_a;
                if (error_b < 0) error_b = -error_b;
                if (error_c < 0) error_c = -error_c;

                local_max_error = error_a;

                if (error_b > local_max_error)
                    local_max_error = error_b;

                if (error_c > local_max_error)
                    local_max_error = error_c;

                if (local_max_error > max_error)
                    max_error = local_max_error;

                $display(
                    "Vector %0d: alpha=%0d beta=%0d | sector=%0d (expected %0d) | duties=%0d,%0d,%0d | golden=%0d,%0d,%0d | max_error=%0d",
                    vectors_tested,
                    alpha,
                    beta,
                    sector,
                    expected_sector,
                    duty_a,
                    duty_b,
                    duty_c,
                    expected_a,
                    expected_b,
                    expected_c,
                    local_max_error
                );

                if (sector !== expected_sector) begin
                    $display("  FAIL: sector mismatch");
                    failures = failures + 1;
                end

                if (error_a > DUTY_TOLERANCE ||
                    error_b > DUTY_TOLERANCE ||
                    error_c > DUTY_TOLERANCE) begin
                    $display("  FAIL: duty error exceeds tolerance");
                    failures = failures + 1;
                end

                if (duty_a > 16'd32767 ||
                    duty_b > 16'd32767 ||
                    duty_c > 16'd32767) begin
                    $display("  FAIL: duty outside golden-model range");
                    failures = failures + 1;
                end
            end

            @(negedge clk);
            input_valid = 1'b0;
        end
    endtask

    initial begin
        rst = 1'b1;
        input_valid = 1'b0;
        v_alpha = 16'sd0;
        v_beta = 16'sd0;

        vectors_tested = 0;
        failures = 0;
        max_error = 0;

        repeat (3) @(negedge clk);
        rst = 1'b0;

        // Expected duties are from frozen svpwm.py:
        // svpwm_fixed(alpha, beta)

        // Zero vector
        apply_golden_vector(
            0, 0, 1,
            16384, 16384, 16384
        );

        // Sector 1
        apply_golden_vector(
            28378, 16384, 1,
            32767, 16383, 0
        );

        // Sector 2
        apply_golden_vector(
            0, 32767, 2,
            16383, 32767, 0
        );

        // Sector 3
        apply_golden_vector(
            -28378, 16384, 3,
            0, 32767, 16384
        );

        // Sector 4
        apply_golden_vector(
            -28378, -16384, 4,
            0, 16384, 32767
        );

        // Sector 5
        apply_golden_vector(
            0, -32767, 5,
            16384, 0, 32767
        );

        // Sector 6
        apply_golden_vector(
            28378, -16384, 6,
            32767, 0, 16383
        );

        // Overmodulation
        apply_golden_vector(
            32767, 32767, 1,
            32767, 23987, 0
        );

        // Negative extreme
        apply_golden_vector(
            -32768, -32768, 4,
            0, 8780, 32767
        );

        $display("");
        $display("==========================================");
        $display("SVPWM GOLDEN COMPARISON COMPLETE");
        $display("==========================================");
        $display("Vectors tested = %0d", vectors_tested);
        $display("Failures       = %0d", failures);
        $display("Maximum duty error = %0d LSB", max_error);
        $display("Allowed duty error = %0d LSB", DUTY_TOLERANCE);
        $display("==========================================");

        if (failures == 0) begin
            $display("SVPWM GOLDEN COMPARISON: PASS");
        end
        else begin
            $display("SVPWM GOLDEN COMPARISON: FAIL");
        end

        $display("==========================================");
        $finish;
    end

endmodule
