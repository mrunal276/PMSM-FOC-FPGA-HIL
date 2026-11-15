`timescale 1ns/1ps

module tb_park;

    logic clk;
    logic rst;
    logic input_valid;

    logic signed [15:0] i_alpha;
    logic signed [15:0] i_beta;
    logic signed [15:0] cos_theta;
    logic signed [15:0] sin_theta;

    logic output_valid;
    logic signed [15:0] i_d;
    logic signed [15:0] i_q;

    integer pass_count;
    integer fail_count;
    integer max_id_error;
    integer max_iq_error;

    park dut (
        .clk        (clk),
        .rst        (rst),
        .input_valid(input_valid),
        .i_alpha    (i_alpha),
        .i_beta     (i_beta),
        .cos_theta  (cos_theta),
        .sin_theta  (sin_theta),
        .output_valid(output_valid),
        .i_d        (i_d),
        .i_q        (i_q)
    );

    always #5 clk = ~clk;

    task automatic run_vector (
        input integer alpha_val,
        input integer beta_val,
        input integer cos_val,
        input integer sin_val,
        input integer expected_d,
        input integer expected_q
    );

        integer actual_d;
        integer actual_q;
        integer err_d;
        integer err_q;

        begin
            @(negedge clk);

            i_alpha   = alpha_val;
            i_beta    = beta_val;
            cos_theta = cos_val;
            sin_theta = sin_val;
            input_valid = 1'b1;

            @(negedge clk);
            input_valid = 1'b0;

            @(posedge clk);
            #1;

            actual_d = $signed(i_d);
            actual_q = $signed(i_q);

            err_d = actual_d - expected_d;
            err_q = actual_q - expected_q;

            if (err_d < 0)
                err_d = -err_d;

            if (err_q < 0)
                err_q = -err_q;

            if (err_d > max_id_error)
                max_id_error = err_d;

            if (err_q > max_iq_error)
                max_iq_error = err_q;

            if ((actual_d == expected_d) && (actual_q == expected_q)) begin
                pass_count = pass_count + 1;

                $display(
                    "Ialpha=%0d Ibeta=%0d cos=%0d sin=%0d | Id=%0d exp=%0d err=%0d | Iq=%0d exp=%0d err=%0d",
                    alpha_val, beta_val,
                    cos_val, sin_val,
                    actual_d, expected_d, err_d,
                    actual_q, expected_q, err_q
                );
            end
            else begin
                fail_count = fail_count + 1;

                $display(
                    "FAIL: Ialpha=%0d Ibeta=%0d cos=%0d sin=%0d | Id=%0d exp=%0d err=%0d | Iq=%0d exp=%0d err=%0d",
                    alpha_val, beta_val,
                    cos_val, sin_val,
                    actual_d, expected_d, err_d,
                    actual_q, expected_q, err_q
                );
            end
        end
    endtask


    initial begin

        clk = 1'b0;
        rst = 1'b1;
        input_valid = 1'b0;

        i_alpha   = 16'sd0;
        i_beta    = 16'sd0;
        cos_theta = 16'sd0;
        sin_theta = 16'sd0;

        pass_count   = 0;
        fail_count   = 0;
        max_id_error = 0;
        max_iq_error = 0;

        repeat (2)
            @(posedge clk);

        rst = 1'b0;

        // ------------------------------------------------------------
        // Golden vector 1
        // Ialpha=32767, Ibeta=0, theta=0 deg
        // ------------------------------------------------------------
        run_vector(
            32767,
            0,
            32767,
            0,
            32766,
            0
        );

        // ------------------------------------------------------------
        // Golden vector 2
        // Ialpha=0, Ibeta=32767, theta=0 deg
        // ------------------------------------------------------------
        run_vector(
            0,
            32767,
            32767,
            0,
            0,
            32766
        );

        // ------------------------------------------------------------
        // Golden vector 3
        // Ialpha=32767, Ibeta=0, theta=90 deg
        // cos=0, sin=32767
        // ------------------------------------------------------------
        run_vector(
            32767,
            0,
            0,
            32767,
            0,
            -32766
        );

        // ------------------------------------------------------------
        // Golden vector 4
        // Ialpha=0, Ibeta=32767, theta=90 deg
        // ------------------------------------------------------------
        run_vector(
            0,
            32767,
            0,
            32767,
            32766,
            0
        );

        // ------------------------------------------------------------
        // Golden vector 5
        // Frozen theta=45 deg round-trip vector
        // ------------------------------------------------------------
        run_vector(
            26214,
            9830,
            23170,
            23170,
            25487,
            -11585
        );

        // ------------------------------------------------------------
        // Golden vector 6
        // Mixed-sign 45 deg vector
        // ------------------------------------------------------------
        run_vector(
            -16384,
            8192,
            23170,
            23170,
            -5792,
            17378
        );

        // ------------------------------------------------------------
        // Golden vector 7
        // Balanced positive vector at 45 deg
        // ------------------------------------------------------------
        run_vector(
            8192,
            8192,
            23170,
            23170,
            11586,
            1
        );

        $display("");
        $display("==========================================");
        $display("PARK BASIC TEST COMPLETE");
        $display("==========================================");
        $display("Vectors tested       = %0d", pass_count + fail_count);
        $display("Maximum Id error     = %0d LSB", max_id_error);
        $display("Maximum Iq error     = %0d LSB", max_iq_error);
        $display("Failed vectors       = %0d", fail_count);
        $display("==========================================");

        if (fail_count == 0) begin
            $display("PARK BASIC TEST: PASS");
        end
        else begin
            $display("PARK BASIC TEST: FAIL");
        end

        $display("==========================================");

        #20;
        $finish;
    end

endmodule