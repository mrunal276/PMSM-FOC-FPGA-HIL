
`timescale 1ns / 1ps

module tb_svpwm_random;

    localparam integer DUTY_TOLERANCE = 1;
    localparam integer EXPECTED_VECTORS = 2175;
    localparam integer NUM_SECTORS = 6;

    logic clk = 1'b0;
    logic rst = 1'b1;
    logic input_valid = 1'b0;

    logic signed [15:0] v_alpha = '0;
    logic signed [15:0] v_beta  = '0;

    logic [15:0] duty_a, duty_b, duty_c;
    logic [2:0] sector;
    logic output_valid;

    integer fd;
    integer scan_result;
    integer alpha_i, beta_i;
    integer expected_sector;
    integer expected_a, expected_b, expected_c;

    integer vector_count = 0;
    integer failed_vectors = 0;
    integer failures = 0;
    integer sector_mismatches = 0;
    integer duty_failures = 0;
    integer range_violations = 0;
    integer valid_failures = 0;
    integer malformed_rows = 0;

    integer sector_counts [1:6];
    integer sector_seen [1:6];

    integer max_error = 0;
    integer error_a, error_b, error_c;
    integer vector_error;
    integer failed_this_vector;

    integer max_error_vector = 0;
    integer max_error_alpha = 0;
    integer max_error_beta = 0;
    integer max_error_sector = 0;
    integer max_error_a = 0;
    integer max_error_b = 0;
    integer max_error_c = 0;
    integer max_expected_a = 0;
    integer max_expected_b = 0;
    integer max_expected_c = 0;

    integer i;

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

    always #5 clk = ~clk;

    initial begin
        for (i = 1; i <= NUM_SECTORS; i = i + 1) begin
            sector_counts[i] = 0;
            sector_seen[i] = 0;
        end

        $display("");
        $display("==========================================================");
        $display("       SVPWM EXPANDED GOLDEN VERIFICATION");
        $display("==========================================================");
        $display("Reference : golden/svpwm_testvectors.csv");
        $display("Tolerance : %0d LSB per duty output", DUTY_TOLERANCE);
        $display("Expected vectors: %0d", EXPECTED_VECTORS);
        $display("");

        repeat (4) @(negedge clk);
        rst = 1'b0;

        fd = $fopen(
            "C:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/golden/svpwm_testvectors.csv",
            "r"
        );

        if (fd == 0) begin
            $display("FATAL: Could not open SVPWM golden CSV.");
            $fatal(1);
        end

        $display("Representative comparisons:");
        $display("Vector | Alpha  | Beta   | Exp.Sec | DUT.Sec | Expected A,B,C | Actual A,B,C | MaxErr");
        $display("-------+--------+--------+---------+---------+----------------+--------------+-------");

        scan_result = $fscanf(
            fd, "%d,%d,%d,%d,%d,%d\n",
            alpha_i, beta_i,
            expected_sector, expected_a, expected_b, expected_c
        );

        while (scan_result == 6) begin
            @(negedge clk);
            input_valid = 1'b1;
            v_alpha = alpha_i;
            v_beta = beta_i;

            // DUT samples the request on the intervening rising edge.
            // SVPWM pipeline is 2 stages → outputs appear 2 cycles after input_valid.
            // Wait one extra negedge so we sample after the second FF stage commits.
            @(negedge clk);
            input_valid = 1'b0;
            @(negedge clk);  // extra wait for 2nd pipeline stage
            #1;

            vector_count = vector_count + 1;
            failed_this_vector = 0;

            error_a = (duty_a >= expected_a) ?
                      (duty_a - expected_a) :
                      (expected_a - duty_a);

            error_b = (duty_b >= expected_b) ?
                      (duty_b - expected_b) :
                      (expected_b - duty_b);

            error_c = (duty_c >= expected_c) ?
                      (duty_c - expected_c) :
                      (expected_c - duty_c);

            vector_error = error_a;
            if (error_b > vector_error) vector_error = error_b;
            if (error_c > vector_error) vector_error = error_c;

            if (expected_sector >= 1 &&
                expected_sector <= NUM_SECTORS) begin
                sector_counts[expected_sector] =
                    sector_counts[expected_sector] + 1;
            end

            // Verify registered valid response.
            if (output_valid !== 1'b1) begin
                valid_failures = valid_failures + 1;
                failures = failures + 1;
                failed_this_vector = 1;

                $display(
                    "FAIL vector %0d: output_valid was not asserted",
                    vector_count
                );
            end

            // Sector must match the golden reference exactly.
            if (sector !== expected_sector[2:0]) begin
                sector_mismatches = sector_mismatches + 1;
                failures = failures + 1;
                failed_this_vector = 1;

                $display(
                    "FAIL sector: vector=%0d alpha=%0d beta=%0d expected=%0d actual=%0d",
                    vector_count, alpha_i, beta_i,
                    expected_sector, sector
                );
            end

            // Each phase duty must be within the original tolerance.
            if ((error_a > DUTY_TOLERANCE) ||
                (error_b > DUTY_TOLERANCE) ||
                (error_c > DUTY_TOLERANCE)) begin

                duty_failures = duty_failures + 1;
                failures = failures + 1;
                failed_this_vector = 1;

                $display(
                    "FAIL duty: vector=%0d errors A/B/C=%0d/%0d/%0d",
                    vector_count, error_a, error_b, error_c
                );
                $display(
                    "  Input alpha/beta=%0d/%0d | Expected=%0d,%0d,%0d | Actual=%0d,%0d,%0d",
                    alpha_i, beta_i,
                    expected_a, expected_b, expected_c,
                    duty_a, duty_b, duty_c
                );
            end

            // Q1.15 duty must be within 0..32767.
            if ((duty_a > 16'd32767) ||
                (duty_b > 16'd32767) ||
                (duty_c > 16'd32767)) begin

                range_violations = range_violations + 1;
                failures = failures + 1;
                failed_this_vector = 1;

                $display(
                    "FAIL range: vector=%0d duties=%0d,%0d,%0d",
                    vector_count, duty_a, duty_b, duty_c
                );
            end

            if (failed_this_vector != 0)
                failed_vectors = failed_vectors + 1;

            // Save the maximum-error example for the final report.
            if (vector_error > max_error) begin
                max_error = vector_error;
                max_error_vector = vector_count;
                max_error_alpha = alpha_i;
                max_error_beta = beta_i;
                max_error_sector = expected_sector;

                max_error_a = duty_a;
                max_error_b = duty_b;
                max_error_c = duty_c;

                max_expected_a = expected_a;
                max_expected_b = expected_b;
                max_expected_c = expected_c;
            end

            // Print the first 12 vectors and one example per sector.
            if ((vector_count <= 12) ||
                ((expected_sector >= 1) &&
                 (expected_sector <= NUM_SECTORS) &&
                 (sector_seen[expected_sector] == 0))) begin

                $display(
                    "%0d | %0d | %0d | %0d | %0d | %0d,%0d,%0d | %0d,%0d,%0d | %0d",
                    vector_count,
                    alpha_i, beta_i,
                    expected_sector, sector,
                    expected_a, expected_b, expected_c,
                    duty_a, duty_b, duty_c,
                    vector_error
                );

                if (expected_sector >= 1 &&
                    expected_sector <= NUM_SECTORS)
                    sector_seen[expected_sector] = 1;
            end

            scan_result = $fscanf(
                fd, "%d,%d,%d,%d,%d,%d\n",
                alpha_i, beta_i,
                expected_sector, expected_a, expected_b, expected_c
            );
        end

        // A clean EOF returns -1. Any other result indicates a malformed row.
        if (scan_result != -1) begin
            malformed_rows = malformed_rows + 1;
            failures = failures + 1;
            $display(
                "FAIL: CSV parsing stopped unexpectedly; fscanf result=%0d",
                scan_result
            );
        end

        $fclose(fd);

        if (vector_count != EXPECTED_VECTORS) begin
            failures = failures + 1;
            $display(
                "FAIL: expected %0d vectors, read %0d",
                EXPECTED_VECTORS, vector_count
            );
        end

        $display("");
        $display("==========================================================");
        $display("                  VERIFICATION RESULTS");
        $display("==========================================================");
        $display("Vectors processed       : %0d", vector_count);
        $display("Expected vectors        : %0d", EXPECTED_VECTORS);
        $display("Unique failed vectors   : %0d", failed_vectors);
        $display("Failed checks           : %0d", failures);
        $display("");
        $display("Sector mismatches       : %0d", sector_mismatches);
        $display("Duty tolerance failures : %0d", duty_failures);
        $display("Duty range violations   : %0d", range_violations);
        $display("output_valid failures   : %0d", valid_failures);
        $display("Malformed CSV rows      : %0d", malformed_rows);
        $display("");
        $display("Sector coverage (golden reference):");
        for (i = 1; i <= NUM_SECTORS; i = i + 1)
            $display("  Sector %0d: %0d vectors", i, sector_counts[i]);

        $display("");
        $display("Maximum duty error      : %0d LSB", max_error);
        $display("Allowed duty error      : %0d LSB", DUTY_TOLERANCE);
        $display("Max-error vector        : %0d", max_error_vector);
        $display("Max-error input         : alpha=%0d beta=%0d",
                 max_error_alpha, max_error_beta);
        $display("Expected sector         : %0d", max_error_sector);
        $display("Expected duties A/B/C   : %0d, %0d, %0d",
                 max_expected_a, max_expected_b, max_expected_c);
        $display("Actual duties A/B/C     : %0d, %0d, %0d",
                 max_error_a, max_error_b, max_error_c);

        $display("");
        $display("==========================================================");

        if ((failures == 0) &&
            (vector_count == EXPECTED_VECTORS) &&
            (max_error <= DUTY_TOLERANCE)) begin
            $display("FINAL RESULT: PASS");
            $display("All vectors and configured checks passed.");
        end
        else begin
            $display("FINAL RESULT: FAIL");
            $display("Review the failure messages above.");
        end

        $display("==========================================================");
        $finish;
    end

endmodule
