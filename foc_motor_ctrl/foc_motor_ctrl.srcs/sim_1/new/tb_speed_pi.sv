module tb_speed_pi;

    logic clk;
    logic rst;
    logic input_valid;

    logic [15:0] omega_ref;
    logic [15:0] omega_meas;

    logic signed [15:0] kp;
    logic signed [31:0] ki_dt_q31;

    logic signed [15:0] iq_ref;
    logic output_valid;

    integer vectors_tested;
    integer passed_vectors;
    integer failed_vectors;

    // ------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------
    speed_pi dut (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (input_valid),
        .omega_ref    (omega_ref),
        .omega_meas   (omega_meas),
        .kp            (kp),
        .ki_dt_q31    (ki_dt_q31),
        .iq_ref       (iq_ref),
        .output_valid (output_valid)
    );

    // ------------------------------------------------------------
    // Clock: 100 MHz
    // ------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // ------------------------------------------------------------
    // Frozen gains
    //
    // Kp = 0.04
    // round(0.04 * 32768) = 1311
    //
    // Ki = 0.5
    // dt = 50 us
    // Ki*dt = 25e-6
    // round(25e-6 * 2^31) = 53687
    // ------------------------------------------------------------
    localparam logic signed [15:0] KP_004 =
        16'sd1311;

    localparam logic signed [31:0] KI_DT =
        32'sd53687;

    // ------------------------------------------------------------
    // Reset
    // ------------------------------------------------------------
    task automatic do_reset;
        begin
            rst = 1'b1;
            input_valid = 1'b0;
            omega_ref = 16'd0;
            omega_meas = 16'd0;
            kp = KP_004;
            ki_dt_q31 = KI_DT;

            repeat (3) @(posedge clk);

            rst = 1'b0;

            @(posedge clk);
        end
    endtask

    // ------------------------------------------------------------
    // Apply one vector
    // ------------------------------------------------------------
    task automatic run_vector(
        input [15:0] ref_value,
        input [15:0] meas_value,
        input signed [15:0] kp_value,
        input signed [31:0] ki_dt_value,
        input signed [15:0] expected
    );

        integer error_value;

        begin
            @(negedge clk);

            omega_ref  = ref_value;
            omega_meas = meas_value;
            kp         = kp_value;
            ki_dt_q31  = ki_dt_value;
            input_valid = 1'b1;

            @(negedge clk);
            input_valid = 1'b0;

            @(posedge clk);

            if (!output_valid) begin
                $display("ERROR: output_valid not asserted");
                failed_vectors = failed_vectors + 1;
            end
            else begin
                error_value = $signed(iq_ref) - expected;

                $display(
                    "ref=%0d meas=%0d kp=%0d ki_dt=%0d | iq_ref=%0d exp=%0d err=%0d",
                    $unsigned(ref_value),
                    $unsigned(meas_value),
                    $signed(kp_value),
                    $signed(ki_dt_value),
                    $signed(iq_ref),
                    expected,
                    error_value
                );

                vectors_tested = vectors_tested + 1;

                if (error_value == 0)
                    passed_vectors = passed_vectors + 1;
                else
                    failed_vectors = failed_vectors + 1;
            end
        end
    endtask

    // ------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------
    initial begin

        vectors_tested = 0;
        passed_vectors = 0;
        failed_vectors = 0;

        rst = 1'b0;
        input_valid = 1'b0;
        omega_ref = 16'd0;
        omega_meas = 16'd0;
        kp = KP_004;
        ki_dt_q31 = KI_DT;

        // --------------------------------------------------------
        // 1. Zero error
        // --------------------------------------------------------
        do_reset();

        run_vector(
            16'd0,
            16'd0,
            KP_004,
            KI_DT,
            16'sd0
        );

        // --------------------------------------------------------
        // 2. Positive speed error
        //
        // error = 64 counts = 1 rad/s
        //
        // P = 1311
        // I increment = 419 Q8.24
        // I contribution = 1 Q1.15
        // output = 1312
        // --------------------------------------------------------
        do_reset();

        run_vector(
            16'd64,
            16'd0,
            KP_004,
            KI_DT,
            16'sd1312
        );

        // Second update:
        // integrator = 838 Q8.24
        // I contribution = 2
        // output = 1313
        run_vector(
            16'd64,
            16'd0,
            KP_004,
            KI_DT,
            16'sd1313
        );

        // --------------------------------------------------------
        // 3. Negative speed error
        // --------------------------------------------------------
        do_reset();

        run_vector(
            16'd0,
            16'd64,
            KP_004,
            KI_DT,
            -16'sd1314
        );

        run_vector(
            16'd0,
            16'd64,
            KP_004,
            KI_DT,
            -16'sd1315
        );

        // --------------------------------------------------------
        // 4. Larger positive error
        //
        // error = 640 counts = 10 rad/s
        // --------------------------------------------------------
        do_reset();

        run_vector(
            16'd640,
            16'd0,
            KP_004,
            KI_DT,
            16'sd13118
        );

        run_vector(
            16'd640,
            16'd0,
            KP_004,
            KI_DT,
            16'sd13126
        );

        run_vector(
            16'd640,
            16'd0,
            KP_004,
            KI_DT,
            16'sd13135
        );

        // --------------------------------------------------------
        // 5. Positive output saturation / anti-windup
        //
        // Kp = ~1.0
        // error = 3200 counts
        // P alone exceeds Q1.15.
        // Integrator must not wind up.
        // --------------------------------------------------------
        do_reset();

        run_vector(
            16'd3200,
            16'd0,
            16'sd32767,
            KI_DT,
            16'sd32767
        );

        run_vector(
            16'd3200,
            16'd0,
            16'sd32767,
            KI_DT,
            16'sd32767
        );

        run_vector(
            16'd3200,
            16'd0,
            16'sd32767,
            KI_DT,
            16'sd32767
        );

        // --------------------------------------------------------
        // 6. Negative saturation / anti-windup
        // --------------------------------------------------------
        do_reset();

        run_vector(
            16'd0,
            16'd3200,
            16'sd32767,
            KI_DT,
            -16'sd32768
        );

        run_vector(
            16'd0,
            16'd3200,
            16'sd32767,
            KI_DT,
            -16'sd32768
        );

        run_vector(
            16'd0,
            16'd3200,
            16'sd32767,
            KI_DT,
            -16'sd32768
        );

        // --------------------------------------------------------
        // 7. Reset clears integrator
        // --------------------------------------------------------
        do_reset();

        run_vector(
            16'd640,
            16'd0,
            KP_004,
            KI_DT,
            16'sd13118
        );

        do_reset();

        run_vector(
            16'd640,
            16'd0,
            KP_004,
            KI_DT,
            16'sd13118
        );

        // --------------------------------------------------------
        // Final report
        // --------------------------------------------------------
        $display("");
        $display("==========================================");
        $display("SPEED PI BASIC TEST COMPLETE");
        $display("==========================================");
        $display("Vectors tested       = %0d", vectors_tested);
        $display("Passed vectors       = %0d", passed_vectors);
        $display("Failed vectors       = %0d", failed_vectors);
        $display("==========================================");

        if (failed_vectors == 0)
            $display("SPEED PI BASIC TEST: PASS");
        else
            $display("SPEED PI BASIC TEST: FAIL");

        $display("==========================================");

        $finish;
    end

endmodule