`timescale 1ns/1ps

module tb_current_pi;

    logic clk;
    logic rst;
    logic input_valid;

    logic signed [15:0] ref_in;
    logic signed [15:0] meas_in;
    logic signed [15:0] kp;
    logic signed [15:0] ki;

    logic signed [15:0] output_data;
    logic               output_valid;

    integer pass_count;
    integer fail_count;

    current_pi dut (
        .clk         (clk),
        .rst         (rst),
        .input_valid (input_valid),
        .ref_in      (ref_in),
        .meas_in     (meas_in),
        .kp          (kp),
        .ki          (ki),
        .output_data (output_data),
        .output_valid(output_valid)
    );

    always #5 clk = ~clk;


    // ============================================================
    // Reset helper
    // ============================================================

    task automatic do_reset;
        begin
            @(negedge clk);
            rst = 1'b1;
            input_valid = 1'b0;

            @(negedge clk);
            rst = 1'b0;
        end
    endtask


    // ============================================================
    // One PI update
    // ============================================================

    task automatic run_vector (
        input integer ref_val,
        input integer meas_val,
        input integer kp_val,
        input integer ki_val,
        input integer expected_output
    );

        integer actual_output;
        integer error;

        begin
            @(negedge clk);

            ref_in      = ref_val;
            meas_in     = meas_val;
            kp          = kp_val;
            ki          = ki_val;
            input_valid = 1'b1;

            @(negedge clk);
            input_valid = 1'b0;

            @(posedge clk);
            #1;

            actual_output = $signed(output_data);

            error = actual_output - expected_output;

            if (error < 0)
                error = -error;

            if (actual_output == expected_output) begin
                pass_count = pass_count + 1;

                $display(
                    "ref=%0d meas=%0d kp=%0d ki=%0d | output=%0d exp=%0d err=%0d",
                    ref_val,
                    meas_val,
                    kp_val,
                    ki_val,
                    actual_output,
                    expected_output,
                    error
                );
            end
            else begin
                fail_count = fail_count + 1;

                $display(
                    "FAIL: ref=%0d meas=%0d kp=%0d ki=%0d | output=%0d exp=%0d err=%0d",
                    ref_val,
                    meas_val,
                    kp_val,
                    ki_val,
                    actual_output,
                    expected_output,
                    error
                );
            end
        end
    endtask


    initial begin

        clk = 1'b0;
        rst = 1'b1;
        input_valid = 1'b0;

        ref_in  = 16'sd0;
        meas_in = 16'sd0;
        kp      = 16'sd0;
        ki      = 16'sd0;

        pass_count = 0;
        fail_count = 0;

        // Initial reset
        repeat (2)
            @(posedge clk);

        rst = 1'b0;


        // ========================================================
        // TEST 1: Zero error
        //
        // e = 0
        // P = 0
        // I = 0
        // output = 0
        // ========================================================

        run_vector(
            0,
            0,
            16384,
            819,
            0
        );


        // ========================================================
        // TEST 2: Positive PI accumulation
        //
        // Kp = 0.5  -> 16384
        // Ki = 0.025 -> 819
        // e  = 0.5 -> 16384
        //
        // First:
        // P = 8192
        // I = 410
        // output = 8602
        //
        // Second:
        // I = 820
        // output = 9012
        // ========================================================

        do_reset;

        run_vector(
            16384,
            0,
            16384,
            819,
            8602
        );

        run_vector(
            16384,
            0,
            16384,
            819,
            9012
        );


        // ========================================================
        // TEST 3: Negative PI accumulation
        //
        // Sign-aware Q15 rounding is important here.
        //
        // First output = -8604
        // Second output = -9014
        // ========================================================

        do_reset;

        run_vector(
            -16384,
            0,
            16384,
            819,
            -8604
        );

        run_vector(
            -16384,
            0,
            16384,
            819,
            -9014
        );


        // ========================================================
        // TEST 4: Error saturation
        //
        // ref - meas = 32767 - (-32768)
        //             = 65535
        //
        // Golden model saturates error to 32767.
        //
        // Kp=0.5:
        // P = 16384
        // I = 819
        // output = 17203
        // ========================================================

        do_reset;

        run_vector(
            32767,
            -32768,
            16384,
            819,
            17203
        );


        // ========================================================
        // TEST 5: Positive output saturation + positive error
        //
        // This must trigger anti-windup.
        //
        // The output saturates at +32767.
        // The positive integrator increment must NOT be committed.
        //
        // Repeating the same input must therefore continue to
        // produce +32767 rather than accumulating the integrator.
        // ========================================================

        do_reset;

        run_vector(
            32767,
            0,
            32767,
            819,
            32767
        );

        run_vector(
            32767,
            0,
            32767,
            819,
            32767
        );

        run_vector(
            32767,
            0,
            32767,
            819,
            32767
        );


        // ========================================================
        // TEST 6: Positive saturation + negative error
        //
        // First force positive saturation.
        // Then apply negative error.
        //
        // The integrator must be allowed to unwind.
        // ========================================================

        do_reset;

        run_vector(
            32767,
            0,
            32767,
            819,
            32767
        );

        run_vector(
            0,
            32767,
            32767,
            819,
            -32768
        );


        // ========================================================
        // TEST 7: Negative output saturation + negative error
        //
        // Negative saturation must also prevent further negative
        // integrator accumulation.
        // ========================================================

        do_reset;

        run_vector(
            -32768,
            0,
            32767,
            819,
            -32768
        );

        run_vector(
            -32768,
            0,
            32767,
            819,
            -32768
        );

        run_vector(
            -32768,
            0,
            32767,
            819,
            -32768
        );


        // ========================================================
        // TEST 8: Negative saturation + positive error
        //
        // Force negative saturation, then apply positive error.
        // Integrator must be allowed to unwind.
        // ========================================================

        do_reset;

        run_vector(
            -32768,
            0,
            32767,
            819,
            -32768
        );

        run_vector(
            0,
            -32768,
            32767,
            819,
            32767
        );


        // ========================================================
        // TEST 9: Reset clears accumulated state
        // ========================================================

        do_reset;

        run_vector(
            16384,
            0,
            16384,
            819,
            8602
        );

        do_reset;

        run_vector(
            16384,
            0,
            16384,
            819,
            8602
        );


        // ========================================================
        // SUMMARY
        // ========================================================

        $display("");
        $display("==========================================");
        $display("CURRENT PI BASIC TEST COMPLETE");
        $display("==========================================");
        $display("Vectors tested       = %0d", pass_count + fail_count);
        $display("Passed vectors       = %0d", pass_count);
        $display("Failed vectors       = %0d", fail_count);
        $display("==========================================");

        if (fail_count == 0) begin
            $display("CURRENT PI BASIC TEST: PASS");
        end
        else begin
            $display("CURRENT PI BASIC TEST: FAIL");
        end

        $display("==========================================");

        #20;
        $finish;

    end

endmodule