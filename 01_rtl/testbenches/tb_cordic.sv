`timescale 1ns/1ps

module tb_cordic;

    // ============================================================
    // Clock / reset
    // ============================================================

    logic clk;
    logic rst;

    // ============================================================
    // DUT interface
    // ============================================================

    logic        theta_valid;
    logic [15:0] theta_in;

    logic        result_valid;
    logic signed [15:0] sin_out;
    logic signed [15:0] cos_out;

    // ============================================================
    // DUT
    // ============================================================

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
    // 100 MHz clock
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // ============================================================
    // Test
    // ============================================================

    integer i;

    integer error_sin;
    integer error_cos;

    integer max_error_sin;
    integer max_error_cos;

    integer signed sin_expected;
    integer signed cos_expected;

    // ------------------------------------------------------------
    // Basic vector set.
    //
    // These are the important angle boundaries plus common
    // quadrants. Full 360-vector CSV comparison will follow once
    // the basic RTL timing/interface is confirmed.
    // ------------------------------------------------------------

    task automatic apply_vector(
        input [15:0] theta,
        input integer expected_sin,
        input integer expected_cos
    );

        begin

            @(posedge clk);

            theta_in   <= theta;
            theta_valid <= 1'b1;

            @(posedge clk);

            theta_valid <= 1'b0;
            theta_in    <= 16'h0000;

            // Wait for the pipelined result.
            wait (result_valid == 1'b1);

            error_sin = $signed(sin_out) - expected_sin;
            error_cos = $signed(cos_out) - expected_cos;

            if (error_sin < 0)
                error_sin = -error_sin;

            if (error_cos < 0)
                error_cos = -error_cos;

            if (error_sin > max_error_sin)
                max_error_sin = error_sin;

            if (error_cos > max_error_cos)
                max_error_cos = error_cos;

            $display(
                "theta=%h  sin=%0d expected=%0d  cos=%0d expected=%0d  err_sin=%0d err_cos=%0d",
                theta,
                $signed(sin_out),
                expected_sin,
                $signed(cos_out),
                expected_cos,
                error_sin,
                error_cos
            );

            if (error_sin > 1 || error_cos > 1) begin
                $display("ERROR: CORDIC result exceeds 1 LSB");
            end

        end

    endtask

    // ============================================================
    // Main stimulus
    // ============================================================

    initial begin

        theta_valid = 1'b0;
        theta_in    = 16'h0000;

        max_error_sin = 0;
        max_error_cos = 0;

        rst = 1'b1;

        repeat (5)
            @(posedge clk);

        rst = 1'b0;

        // --------------------------------------------------------
        // Exact Q1.15 reference values
        // --------------------------------------------------------

        // 0 degrees
        apply_vector(
            16'h0000,
            0,
            32767
        );

        // 90 degrees
        apply_vector(
            16'h2000,
            32767,
            0
        );

        // 180 degrees
        apply_vector(
            16'h4000,
            0,
            -32768
        );

        // 270 degrees
        apply_vector(
            16'h6000,
            -32768,
            0
        );

        // 360 degrees = 0 degrees
        apply_vector(
            16'h8000,
            0,
            32767
        );

        // --------------------------------------------------------
        // Report
        // --------------------------------------------------------

        $display("");
        $display("==========================================");
        $display("CORDIC BASIC TEST COMPLETE");
        $display("Maximum sine error   = %0d LSB", max_error_sin);
        $display("Maximum cosine error = %0d LSB", max_error_cos);
        $display("==========================================");

        if (max_error_sin <= 1 && max_error_cos <= 1) begin
            $display("CORDIC BASIC TEST: PASS");
        end
        else begin
            $display("CORDIC BASIC TEST: FAIL");
        end

        #100;

        $finish;

    end

endmodule