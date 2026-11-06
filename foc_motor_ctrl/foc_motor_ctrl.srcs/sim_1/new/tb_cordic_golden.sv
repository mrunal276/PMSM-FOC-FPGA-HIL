`timescale 1ns/1ps

module tb_cordic_golden;

    // ============================================================
    // DUT interface
    // ============================================================

    logic clk;
    logic rst;

    logic        theta_valid;
    logic [15:0] theta_in;

    logic              result_valid;
    logic signed [15:0] sin_out;
    logic signed [15:0] cos_out;

    cordic dut (
        .clk          (clk),
        .rst          (rst),
        .theta_valid  (theta_valid),
        .theta_in     (theta_in),
        .result_valid (result_valid),
        .sin_out      (sin_out),
        .cos_out      (cos_out)
    );

    // ============================================================
    // Clock: 100 MHz
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // ============================================================
    // CSV file
    // ============================================================

    localparam string CSV_FILE =
        "C:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/golden/cordic_testvectors.csv";

    integer fd;
    integer scan_status;
    string line;

    // CSV fields
    integer theta_deg;
    integer theta_hex;
    integer cos_hex;
    integer sin_hex;

    // Expected signed Q1.15 values
    integer expected_cos;
    integer expected_sin;

    // Errors
    integer error_cos;
    integer error_sin;

    integer max_error_cos;
    integer max_error_sin;

    integer vector_count;
    integer fail_count;

    integer timeout_count;

    // ============================================================
    // Apply one vector
    // ============================================================

    task automatic apply_vector(
        input integer theta_value,
        input integer expected_cos_value,
        input integer expected_sin_value
    );

        logic [15:0] theta_q214;

        begin

            theta_q214 = theta_value[15:0];

            // ----------------------------------------------------
            // Apply input
            // ----------------------------------------------------

            @(posedge clk);

            theta_in    <= theta_q214;
            theta_valid <= 1'b1;

            @(posedge clk);

            theta_valid <= 1'b0;
            theta_in    <= 16'h0000;

            // ----------------------------------------------------
            // Wait for result
            // ----------------------------------------------------

            timeout_count = 0;

            while (result_valid != 1'b1) begin

                @(posedge clk);

                timeout_count = timeout_count + 1;

                if (timeout_count > 100) begin

                    $display(
                        "ERROR: Timeout, theta_q214=%h",
                        theta_q214
                    );

                    fail_count = fail_count + 1;

                    return;
                end

            end

            // ----------------------------------------------------
            // Calculate absolute errors
            // ----------------------------------------------------

            error_cos =
                $signed(cos_out) - expected_cos_value;

            error_sin =
                $signed(sin_out) - expected_sin_value;

            if (error_cos < 0)
                error_cos = -error_cos;

            if (error_sin < 0)
                error_sin = -error_sin;

            // ----------------------------------------------------
            // Track maximum error
            // ----------------------------------------------------

            if (error_cos > max_error_cos)
                max_error_cos = error_cos;

            if (error_sin > max_error_sin)
                max_error_sin = error_sin;

            vector_count = vector_count + 1;

            // ----------------------------------------------------
            // Failure
            // ----------------------------------------------------

            if ((error_cos > 1) || (error_sin > 1)) begin

                fail_count = fail_count + 1;

                $display(
                    "FAIL: deg=%0d theta=%h cos=%0d exp=%0d err=%0d sin=%0d exp=%0d err=%0d",
                    theta_deg,
                    theta_q214,
                    $signed(cos_out),
                    expected_cos_value,
                    error_cos,
                    $signed(sin_out),
                    expected_sin_value,
                    error_sin
                );

            end
            else begin

                // Print only first 5 vectors and major boundaries.
                if ((vector_count <= 5) ||
                    (theta_deg == 90) ||
                    (theta_deg == 180) ||
                    (theta_deg == 270) ||
                    (theta_deg == 359)) begin

                    $display(
                        "PASS: deg=%0d theta=%h cos=%0d exp=%0d err=%0d sin=%0d exp=%0d err=%0d",
                        theta_deg,
                        theta_q214,
                        $signed(cos_out),
                        expected_cos_value,
                        error_cos,
                        $signed(sin_out),
                        expected_sin_value,
                        error_sin
                    );

                end

            end

        end

    endtask

    // ============================================================
    // Main test
    // ============================================================

    initial begin

        // --------------------------------------------------------
        // Initialization
        // --------------------------------------------------------

        theta_valid = 1'b0;
        theta_in    = 16'h0000;

        max_error_cos = 0;
        max_error_sin = 0;

        vector_count = 0;
        fail_count   = 0;

        // --------------------------------------------------------
        // Reset
        // --------------------------------------------------------

        rst = 1'b1;

        repeat (5)
            @(posedge clk);

        rst = 1'b0;

        // --------------------------------------------------------
        // Open CSV
        // --------------------------------------------------------

        fd = $fopen(CSV_FILE, "r");

        if (fd == 0) begin

            $display("");
            $display("==========================================");
            $display("ERROR: Could not open CORDIC CSV file");
            $display("==========================================");
            $display("%s", CSV_FILE);
            $display("==========================================");

            $finish;

        end

        $display("");
        $display("==========================================");
        $display("CORDIC GOLDEN VECTOR TEST");
        $display("==========================================");
        $display("CSV: %s", CSV_FILE);
        $display("");

        // --------------------------------------------------------
        // Skip CSV header
        // --------------------------------------------------------

        $fgets(line, fd);

        // --------------------------------------------------------
        // Read CSV
        // --------------------------------------------------------

        while (!$feof(fd)) begin

            line = "";

            $fgets(line, fd);

            if (line != "") begin

                scan_status = $sscanf(
                    line,
                    "%d,%h,%h,%h",
                    theta_deg,
                    theta_hex,
                    cos_hex,
                    sin_hex
                );

                if (scan_status == 4) begin

                    // ------------------------------------------------
                    // Convert Q1.15 hex to signed integer
                    // ------------------------------------------------

                    if (cos_hex >= 16'h8000)
                        expected_cos = cos_hex - 65536;
                    else
                        expected_cos = cos_hex;

                    if (sin_hex >= 16'h8000)
                        expected_sin = sin_hex - 65536;
                    else
                        expected_sin = sin_hex;

                    // ------------------------------------------------
                    // Apply vector
                    // ------------------------------------------------

                    apply_vector(
                        theta_hex,
                        expected_cos,
                        expected_sin
                    );

                end

            end

        end

        // --------------------------------------------------------
        // Close CSV
        // --------------------------------------------------------

        $fclose(fd);

        // --------------------------------------------------------
        // Final report
        // --------------------------------------------------------

        $display("");
        $display("==========================================");
        $display("CORDIC GOLDEN VECTOR TEST COMPLETE");
        $display("==========================================");

        $display(
            "Vectors tested       = %0d",
            vector_count
        );

        $display(
            "Maximum cosine error = %0d LSB",
            max_error_cos
        );

        $display(
            "Maximum sine error   = %0d LSB",
            max_error_sin
        );

        $display(
            "Failed vectors       = %0d",
            fail_count
        );

        $display("");

        if ((vector_count == 360) &&
            (max_error_cos <= 1) &&
            (max_error_sin <= 1) &&
            (fail_count == 0)) begin

            $display("CORDIC GOLDEN VECTOR TEST: PASS");

        end
        else begin

            $display("CORDIC GOLDEN VECTOR TEST: FAIL");

        end

        $display("==========================================");

        #100;

        $finish;

    end

endmodule