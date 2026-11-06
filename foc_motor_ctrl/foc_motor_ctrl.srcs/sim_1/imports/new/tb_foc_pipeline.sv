
`timescale 1ns / 1ps

module tb_foc_pipeline;

    localparam integer MAX_VECTORS      = 500;
    localparam integer DUTY_TOL         = 2;
    localparam integer TRANSFORM_TOL    = 2;
    localparam integer EXPECTED_LATENCY = 22; // SVPWM now 2-stage pipelined (was 21)

    reg clk = 0;
    reg rst = 1;
    reg input_valid = 0;

    reg signed [15:0] ia_in, ib_in, ic_in;
    reg        [15:0] theta_e;
    reg        [15:0] omega_meas, omega_ref;
    reg signed [15:0] id_ref;

    reg signed [15:0] speed_kp;
    reg signed [31:0] speed_ki_dt;
    reg signed [15:0] current_kp, current_ki;

    wire output_valid;
    wire [15:0] duty_a, duty_b, duty_c;
    wire [2:0] sector;

    wire signed [15:0] i_alpha_out, i_beta_out;
    wire signed [15:0] id_meas_out, iq_meas_out;
    wire signed [15:0] iq_ref_out;
    wire signed [15:0] vd_out, vq_out;
    wire signed [15:0] valpha_out, vbeta_out;

    foc_pipeline dut (
        .clk(clk),
        .rst(rst),
        .input_valid(input_valid),

        .ia_in(ia_in),
        .ib_in(ib_in),
        .ic_in(ic_in),
        .theta_e(theta_e),

        .omega_meas(omega_meas),
        .omega_ref(omega_ref),
        .id_ref(id_ref),

        .speed_kp(speed_kp),
        .speed_ki_dt(speed_ki_dt),
        .current_kp(current_kp),
        .current_ki(current_ki),

        .output_valid(output_valid),
        .duty_a(duty_a),
        .duty_b(duty_b),
        .duty_c(duty_c),
        .sector(sector),

        .i_alpha_out(i_alpha_out),
        .i_beta_out(i_beta_out),
        .id_meas_out(id_meas_out),
        .iq_meas_out(iq_meas_out),
        .iq_ref_out(iq_ref_out),
        .vd_out(vd_out),
        .vq_out(vq_out),
        .valpha_out(valpha_out),
        .vbeta_out(vbeta_out)
    );

    always #5 clk = ~clk;

    // CSV input vectors
    integer vec_omega_ref [0:MAX_VECTORS-1];
    integer vec_omega_meas[0:MAX_VECTORS-1];
    integer vec_theta     [0:MAX_VECTORS-1];
    integer vec_ia        [0:MAX_VECTORS-1];
    integer vec_ib        [0:MAX_VECTORS-1];
    integer vec_ic        [0:MAX_VECTORS-1];
    integer vec_id_ref    [0:MAX_VECTORS-1];

    // Golden expected outputs
    integer exp_sector  [0:MAX_VECTORS-1];
    integer exp_duty_a  [0:MAX_VECTORS-1];
    integer exp_duty_b  [0:MAX_VECTORS-1];
    integer exp_duty_c  [0:MAX_VECTORS-1];
    integer exp_i_alpha [0:MAX_VECTORS-1];
    integer exp_i_beta  [0:MAX_VECTORS-1];

    integer fd;
    integer scan_count;
    integer vector_count = 0;

    integer t0, t1, t2, t3, t4, t5, t6;
    integer t7, t8, t9, t10, t11, t12;

    integer i;
    integer cycle_count = 0;
    integer first_input_cycle = -1;
    integer measured_latency = -1;

    integer output_count = 0;
    integer pass_count = 0;
    integer failed_vectors = 0;
    integer errors = 0;

    integer max_duty_error = 0;
    integer max_err_a = 0;
    integer max_err_b = 0;
    integer max_err_c = 0;
    integer max_alpha_error = 0;
    integer max_beta_error = 0;

    integer diff_a, diff_b, diff_c;
    integer diff_alpha, diff_beta;
    integer vector_errors;
    integer first_failure_reported = 0;

    // Monitor registered pipeline outputs.
    always @(posedge clk) begin
        cycle_count = cycle_count + 1;
        #1;

        if (output_valid) begin
            if (output_count >= vector_count) begin
                $display(
                    "FAIL: Unexpected extra output_valid pulse at cycle %0d",
                    cycle_count
                );
                errors = errors + 1;
            end
            else begin
                vector_errors = 0;

                // Measure latency from the first accepted input edge
                // using the testbench's documented cycle-count convention.
                if (output_count == 0) begin
                    measured_latency =
                        cycle_count - first_input_cycle;

                    $display(
                        "Measured first-result latency = %0d cycles",
                        measured_latency
                    );

                    if (measured_latency != EXPECTED_LATENCY) begin
                        $display(
                            "FAIL: Expected latency %0d, got %0d",
                            EXPECTED_LATENCY,
                            measured_latency
                        );
                        errors = errors + 1;
                    end
                    else begin
                        $display(
                            "PASS: Pipeline latency is correct (%0d cycles)",
                            measured_latency
                        );
                    end
                end

                // Calculate absolute duty errors.
                diff_a =
                    $signed({1'b0, duty_a}) -
                    exp_duty_a[output_count];

                diff_b =
                    $signed({1'b0, duty_b}) -
                    exp_duty_b[output_count];

                diff_c =
                    $signed({1'b0, duty_c}) -
                    exp_duty_c[output_count];

                if (diff_a < 0) diff_a = -diff_a;
                if (diff_b < 0) diff_b = -diff_b;
                if (diff_c < 0) diff_c = -diff_c;

                // Track maximum duty errors.
                if (diff_a > max_err_a) max_err_a = diff_a;
                if (diff_b > max_err_b) max_err_b = diff_b;
                if (diff_c > max_err_c) max_err_c = diff_c;

                if (diff_a > max_duty_error)
                    max_duty_error = diff_a;
                if (diff_b > max_duty_error)
                    max_duty_error = diff_b;
                if (diff_c > max_duty_error)
                    max_duty_error = diff_c;

                // STRICT duty comparison.
                if ((diff_a > DUTY_TOL) ||
                    (diff_b > DUTY_TOL) ||
                    (diff_c > DUTY_TOL)) begin
                    vector_errors = vector_errors + 1;
                end

                // Duty output range.
                if ((duty_a > 16'd32767) ||
                    (duty_b > 16'd32767) ||
                    (duty_c > 16'd32767)) begin

                    $display(
                        "FAIL vector %0d: duty outside legal range (%0d, %0d, %0d)",
                        output_count, duty_a, duty_b, duty_c
                    );
                    vector_errors = vector_errors + 1;
                end

                // Sector range and exact golden sector.
                if ((sector < 1) || (sector > 6)) begin
                    $display(
                        "FAIL vector %0d: invalid sector=%0d",
                        output_count, sector
                    );
                    vector_errors = vector_errors + 1;
                end

                if (sector != exp_sector[output_count]) begin
                    vector_errors = vector_errors + 1;
                end

                // Compare Clarke outputs against the CSV reference.
                diff_alpha =
                    $signed(i_alpha_out) -
                    exp_i_alpha[output_count];

                diff_beta =
                    $signed(i_beta_out) -
                    exp_i_beta[output_count];

                if (diff_alpha < 0)
                    diff_alpha = -diff_alpha;
                if (diff_beta < 0)
                    diff_beta = -diff_beta;

                if (diff_alpha > max_alpha_error)
                    max_alpha_error = diff_alpha;
                if (diff_beta > max_beta_error)
                    max_beta_error = diff_beta;

                if ((diff_alpha > TRANSFORM_TOL) ||
                    (diff_beta > TRANSFORM_TOL)) begin
                    vector_errors = vector_errors + 1;
                end

                // Print complete diagnostics for the first failing vector.
                if ((vector_errors != 0) &&
                    (first_failure_reported == 0)) begin

                    first_failure_reported = 1;

                    $display("");
                    $display("========== FIRST FAILING VECTOR ==========");
                    $display("Vector index : %0d", output_count);
                    $display("Cycle        : %0d", cycle_count);

                    $display(
                        "Inputs: omega_ref=%0d omega_meas=%0d theta=%0d",
                        vec_omega_ref[output_count],
                        vec_omega_meas[output_count],
                        vec_theta[output_count]
                    );

                    $display(
                        "Inputs: ia=%0d ib=%0d ic=%0d id_ref=%0d",
                        vec_ia[output_count],
                        vec_ib[output_count],
                        vec_ic[output_count],
                        vec_id_ref[output_count]
                    );

                    $display(
                        "Duty expected: A=%0d B=%0d C=%0d",
                        exp_duty_a[output_count],
                        exp_duty_b[output_count],
                        exp_duty_c[output_count]
                    );

                    $display(
                        "Duty actual  : A=%0d B=%0d C=%0d",
                        duty_a, duty_b, duty_c
                    );

                    $display(
                        "Duty errors  : A=%0d B=%0d C=%0d (tol=%0d)",
                        diff_a, diff_b, diff_c, DUTY_TOL
                    );

                    $display(
                        "Sector actual=%0d expected=%0d",
                        sector, exp_sector[output_count]
                    );

                    $display(
                        "Clarke alpha: actual=%0d expected=%0d error=%0d",
                        i_alpha_out,
                        exp_i_alpha[output_count],
                        diff_alpha
                    );

                    $display(
                        "Clarke beta : actual=%0d expected=%0d error=%0d",
                        i_beta_out,
                        exp_i_beta[output_count],
                        diff_beta
                    );

                    $display(
                        "Telemetry: id=%0d iq=%0d iq_ref=%0d",
                        id_meas_out, iq_meas_out, iq_ref_out
                    );

                    $display(
                        "Telemetry: vd=%0d vq=%0d",
                        vd_out, vq_out
                    );

                    $display(
                        "Telemetry: valpha=%0d vbeta=%0d",
                        valpha_out, vbeta_out
                    );

                    $display(
                        "Assertions failed on this vector: %0d",
                        vector_errors
                    );
                    $display("==========================================");
                    $display("");
                end

                // Count each vector only once as passed or failed.
                if (vector_errors == 0) begin
                    pass_count = pass_count + 1;
                end
                else begin
                    failed_vectors = failed_vectors + 1;
                    errors = errors + vector_errors;
                end

                output_count = output_count + 1;
            end
        end
    end

    initial begin
        $display("==================================================");
        $display("FOC PIPELINE GOLDEN-REFERENCE VERIFICATION");
        $display("==================================================");

        // Gains matching the currently documented golden model.
        // Speed Kp = 0.04, Q1.15 = 1311
        // Speed Ki*dt = 0.5 * 50us * 2^31 = 53687
        // Current Kp = 3.0, Q4.12 = 12288
        // Current Ki = 0.025, Q1.15 = 819
        speed_kp    = 16'sd1311;
        speed_ki_dt = 32'sd53687;
        current_kp  = 16'sd12288;
        current_ki  = 16'sd819;

        ia_in = 0;
        ib_in = 0;
        ic_in = 0;
        theta_e = 0;
        omega_meas = 0;
        omega_ref = 0;
        id_ref = 0;

        // Open the bit-true RTL integer reference CSV file.
        fd = $fopen(
            "C:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/golden/foc_pipeline_rtl_testvectors.csv",
            "r"
        );

        if (fd == 0) begin
            $display("ERROR: Could not open foc_pipeline_rtl_testvectors.csv");
            $fatal(1);
        end

        // Read 13 comma-separated integer fields per vector.
        scan_count = $fscanf(
            fd,
            "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            t0, t1, t2, t3, t4, t5, t6,
            t7, t8, t9, t10, t11, t12
        );

        while ((scan_count == 13) &&
               (vector_count < MAX_VECTORS)) begin

            vec_omega_ref [vector_count] = t0;
            vec_omega_meas[vector_count] = t1;
            vec_theta     [vector_count] = t2;
            vec_ia        [vector_count] = t3;
            vec_ib        [vector_count] = t4;
            vec_ic        [vector_count] = t5;
            vec_id_ref    [vector_count] = t6;

            exp_sector    [vector_count] = t7;
            exp_duty_a    [vector_count] = t8;
            exp_duty_b    [vector_count] = t9;
            exp_duty_c    [vector_count] = t10;
            exp_i_alpha   [vector_count] = t11;
            exp_i_beta    [vector_count] = t12;

            vector_count = vector_count + 1;

            scan_count = $fscanf(
                fd,
                "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                t0, t1, t2, t3, t4, t5, t6,
                t7, t8, t9, t10, t11, t12
            );
        end

        $fclose(fd);

        $display("Loaded %0d vectors from CSV", vector_count);

        if (vector_count == 0) begin
            $display("ERROR: CSV contains no valid vectors.");
            $fatal(1);
        end

        // Reset DUT.
        repeat (5) @(negedge clk);
        rst = 0;

        
    // Apply one input sample every 5000 clock cycles.
    // At 100 MHz, 5000 cycles = 50 us = one 20 kHz control period.
    for (i = 0; i < vector_count; i = i + 1) begin
        @(negedge clk);

        omega_ref  = vec_omega_ref[i];
        omega_meas = vec_omega_meas[i];
        theta_e    = vec_theta[i];

        ia_in = vec_ia[i];
        ib_in = vec_ib[i];
        ic_in = vec_ic[i];
        id_ref = vec_id_ref[i];

        input_valid = 1'b1;

        if (i == 0)
            first_input_cycle = cycle_count + 1;

        // Keep input_valid asserted for one clock period only.
        @(negedge clk);
        input_valid = 1'b0;

        // Space successive samples by 5000 clock cycles.
        // The pulse above already occupies one clock period.
        if (i < vector_count - 1)
            repeat (4999) @(negedge clk);
    end


        // Allow final transactions to leave the pipeline.
        repeat (35) @(negedge clk);

        $display("");
        $display("==================================================");
        $display("FOC PIPELINE VERIFICATION SUMMARY");
        $display("==================================================");
        $display("CSV vectors loaded       : %0d", vector_count);
        $display("Valid outputs received   : %0d", output_count);
        $display("Passed vectors           : %0d", pass_count);
        $display("Failed vectors           : %0d", failed_vectors);
        $display(
            "Maximum duty error       : %0d LSB (A: %0d, B: %0d, C: %0d)",
            max_duty_error, max_err_a, max_err_b, max_err_c
        );
        $display(
            "Maximum Clarke errors    : alpha=%0d beta=%0d LSB",
            max_alpha_error, max_beta_error
        );
        $display("Duty tolerance           : %0d LSB", DUTY_TOL);
        $display("Transform tolerance      : %0d LSB", TRANSFORM_TOL);
        $display("Measured latency         : %0d cycles", measured_latency);
        $display("Total assertion errors   : %0d", errors);
        $display("==================================================");

        // Missing or extra outputs must prevent PASS.
        if (output_count != vector_count) begin
            $display(
                "FAIL: Expected %0d outputs but received %0d",
                vector_count, output_count
            );
            errors = errors + 1;
        end

        if ((errors == 0) &&
            (failed_vectors == 0) &&
            (output_count == vector_count) &&
            (measured_latency == EXPECTED_LATENCY)) begin

            $display("FOC PIPELINE GOLDEN VERIFICATION: PASS");
        end
        else begin
            $display("FOC PIPELINE GOLDEN VERIFICATION: FAIL");
        end

        $display("==================================================");

        #20;
        $finish;
    end

endmodule
