`timescale 1ns/1ps

module tb_clarke;

    logic clk;
    logic rst;
    logic input_valid;

    logic signed [15:0] ia_in;
    logic signed [15:0] ib_in;
    logic signed [15:0] ic_in;

    logic output_valid;
    logic signed [15:0] i_alpha;
    logic signed [15:0] i_beta;

    clarke dut (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (input_valid),
        .ia_in        (ia_in),
        .ib_in        (ib_in),
        .ic_in        (ic_in),
        .output_valid (output_valid),
        .i_alpha      (i_alpha),
        .i_beta       (i_beta)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    integer error_alpha;
    integer error_beta;

    integer max_error_alpha;
    integer max_error_beta;

    integer test_count;
    integer fail_count;

    task automatic apply_vector(
        input logic signed [15:0] ia,
        input logic signed [15:0] ib,
        input logic signed [15:0] ic,
        input integer expected_alpha,
        input integer expected_beta
    );
        begin
            @(posedge clk);

            ia_in       <= ia;
            ib_in       <= ib;
            ic_in       <= ic;
            input_valid <= 1'b1;

            @(posedge clk);

            input_valid <= 1'b0;

            wait (output_valid == 1'b1);

            error_alpha = $signed(i_alpha) - expected_alpha;
            error_beta  = $signed(i_beta)  - expected_beta;

            if (error_alpha < 0)
                error_alpha = -error_alpha;

            if (error_beta < 0)
                error_beta = -error_beta;

            if (error_alpha > max_error_alpha)
                max_error_alpha = error_alpha;

            if (error_beta > max_error_beta)
                max_error_beta = error_beta;

            test_count = test_count + 1;

            $display(
                "Ia=%0d Ib=%0d Ic=%0d | alpha=%0d exp=%0d err=%0d | beta=%0d exp=%0d err=%0d",
                $signed(ia),
                $signed(ib),
                $signed(ic),
                $signed(i_alpha),
                expected_alpha,
                error_alpha,
                $signed(i_beta),
                expected_beta,
                error_beta
            );

            if (error_alpha > 1 || error_beta > 1) begin
                fail_count = fail_count + 1;
                $display("ERROR: Clarke result exceeds 1 LSB");
            end
        end
    endtask

    initial begin
        rst         = 1'b1;
        input_valid = 1'b0;

        ia_in = 16'sd0;
        ib_in = 16'sd0;
        ic_in = 16'sd0;

        max_error_alpha = 0;
        max_error_beta  = 0;

        test_count = 0;
        fail_count = 0;

        repeat (5)
            @(posedge clk);

        rst = 1'b0;

        // Test 1
        // Ia = 1.0, Ib = -0.5, Ic = -0.5
        // Golden: alpha=32767, beta=0
        apply_vector(
            16'sd32767,
            -16'sd16384,
            -16'sd16384,
            32767,
            0
        );

        // Test 2
        // Zero vector
        apply_vector(
            16'sd0,
            16'sd0,
            16'sd0,
            0,
            0
        );

        // Test 3
        // Ia = 0.5, Ib = -0.25, Ic = -0.25
        apply_vector(
            16'sd16384,
            -16'sd8192,
            -16'sd8192,
            16384,
            0
        );

        // Test 4
        // Ia = 0, Ib = 0.5, Ic = -0.5
        // Exact frozen-golden result: beta = 18920
        apply_vector(
            16'sd0,
            16'sd16384,
            -16'sd16384,
            0,
            18920
        );

        // Test 5
        // Ia = -0.5, Ib = 0.25, Ic = 0.25
        // Exact frozen-golden result: beta = 1
        apply_vector(
            -16'sd16384,
            16'sd8192,
            16'sd8192,
            -16384,
            1
        );

        // Test 6
        // Ia = 0.25, Ib = 0.25, Ic = -0.5
        // Exact frozen-golden result: beta = 14190
        apply_vector(
            16'sd8192,
            16'sd8192,
            -16'sd16384,
            8192,
            14190
        );

        $display("");
        $display("==========================================");
        $display("CLARKE BASIC TEST COMPLETE");
        $display("==========================================");

        $display(
            "Vectors tested       = %0d",
            test_count
        );

        $display(
            "Maximum alpha error  = %0d LSB",
            max_error_alpha
        );

        $display(
            "Maximum beta error   = %0d LSB",
            max_error_beta
        );

        $display(
            "Failed vectors       = %0d",
            fail_count
        );

        $display("==========================================");

        if ((fail_count == 0) &&
            (max_error_alpha <= 1) &&
            (max_error_beta <= 1)) begin

            $display("CLARKE BASIC TEST: PASS");

        end
        else begin

            $display("CLARKE BASIC TEST: FAIL");

        end

        $display("==========================================");

        #100;
        $finish;
    end

endmodule