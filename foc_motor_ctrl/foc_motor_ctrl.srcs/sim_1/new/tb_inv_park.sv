`timescale 1ns/1ps

module tb_inv_park;

    logic clk;
    logic rst;
    logic input_valid;

    logic signed [15:0] v_d;
    logic signed [15:0] v_q;
    logic signed [15:0] cos_theta;
    logic signed [15:0] sin_theta;

    logic output_valid;
    logic signed [15:0] v_alpha;
    logic signed [15:0] v_beta;

    integer pass_count;
    integer fail_count;
    integer max_alpha_error;
    integer max_beta_error;

    inv_park dut (
        .clk         (clk),
        .rst         (rst),
        .input_valid (input_valid),
        .v_d         (v_d),
        .v_q         (v_q),
        .cos_theta   (cos_theta),
        .sin_theta   (sin_theta),
        .output_valid(output_valid),
        .v_alpha     (v_alpha),
        .v_beta      (v_beta)
    );

    always #5 clk = ~clk;


    task automatic run_vector (
        input integer vd_val,
        input integer vq_val,
        input integer cos_val,
        input integer sin_val,
        input integer expected_alpha,
        input integer expected_beta
    );

        integer actual_alpha;
        integer actual_beta;
        integer err_alpha;
        integer err_beta;

        begin
            @(negedge clk);

            v_d       = vd_val;
            v_q       = vq_val;
            cos_theta = cos_val;
            sin_theta = sin_val;
            input_valid = 1'b1;

            @(negedge clk);
            input_valid = 1'b0;

            @(posedge clk);
            #1;

            actual_alpha = $signed(v_alpha);
            actual_beta  = $signed(v_beta);

            err_alpha = actual_alpha - expected_alpha;
            err_beta  = actual_beta - expected_beta;

            if (err_alpha < 0)
                err_alpha = -err_alpha;

            if (err_beta < 0)
                err_beta = -err_beta;

            if (err_alpha > max_alpha_error)
                max_alpha_error = err_alpha;

            if (err_beta > max_beta_error)
                max_beta_error = err_beta;

            if ((actual_alpha == expected_alpha) &&
                (actual_beta  == expected_beta)) begin

                pass_count = pass_count + 1;

                $display(
                    "Vd=%0d Vq=%0d cos=%0d sin=%0d | Valpha=%0d exp=%0d err=%0d | Vbeta=%0d exp=%0d err=%0d",
                    vd_val, vq_val,
                    cos_val, sin_val,
                    actual_alpha, expected_alpha, err_alpha,
                    actual_beta, expected_beta, err_beta
                );

            end
            else begin

                fail_count = fail_count + 1;

                $display(
                    "FAIL: Vd=%0d Vq=%0d cos=%0d sin=%0d | Valpha=%0d exp=%0d err=%0d | Vbeta=%0d exp=%0d err=%0d",
                    vd_val, vq_val,
                    cos_val, sin_val,
                    actual_alpha, expected_alpha, err_alpha,
                    actual_beta, expected_beta, err_beta
                );

            end
        end
    endtask


    initial begin

        clk = 1'b0;
        rst = 1'b1;
        input_valid = 1'b0;

        v_d       = 16'sd0;
        v_q       = 16'sd0;
        cos_theta = 16'sd0;
        sin_theta = 16'sd0;

        pass_count      = 0;
        fail_count      = 0;
        max_alpha_error = 0;
        max_beta_error  = 0;

        repeat (2)
            @(posedge clk);

        rst = 1'b0;


        // ------------------------------------------------------------
        // Golden vector 1
        // Vd=32767, Vq=0, theta=0 deg
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
        // Vd=0, Vq=32767, theta=0 deg
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
        // Vd=32767, Vq=0, theta=90 deg
        // ------------------------------------------------------------
        run_vector(
            32767,
            0,
            0,
            32767,
            0,
            32766
        );


        // ------------------------------------------------------------
        // Golden vector 4
        // Vd=0, Vq=32767, theta=90 deg
        // ------------------------------------------------------------
        run_vector(
            0,
            32767,
            0,
            32767,
            -32766,
            0
        );


        // ------------------------------------------------------------
        // Golden vector 5
        // Frozen theta=45 deg vector
        // ------------------------------------------------------------
        run_vector(
            26214,
            9830,
            23170,
            23170,
            11585,
            25487
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
            -17378,
            -5792
        );


        // ------------------------------------------------------------
        // Golden vector 7
        // Balanced positive 45 deg vector
        // ------------------------------------------------------------
        run_vector(
            8192,
            8192,
            23170,
            23170,
            0,
            11586
        );


        $display("");
        $display("==========================================");
        $display("INVERSE PARK BASIC TEST COMPLETE");
        $display("==========================================");
        $display("Vectors tested       = %0d", pass_count + fail_count);
        $display("Maximum alpha error  = %0d LSB", max_alpha_error);
        $display("Maximum beta error   = %0d LSB", max_beta_error);
        $display("Failed vectors       = %0d", fail_count);
        $display("==========================================");

        if (fail_count == 0) begin
            $display("INVERSE PARK BASIC TEST: PASS");
        end
        else begin
            $display("INVERSE PARK BASIC TEST: FAIL");
        end

        $display("==========================================");

        #20;
        $finish;

    end

endmodule