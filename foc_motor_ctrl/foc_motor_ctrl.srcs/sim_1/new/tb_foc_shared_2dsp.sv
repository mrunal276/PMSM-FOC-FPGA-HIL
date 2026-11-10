`timescale 1ns/1ps

module tb_foc_shared_2dsp;

    logic        clk;
    logic        rst;
    logic        start;

    logic signed [15:0] i_alpha;
    logic signed [15:0] i_beta;
    logic signed [15:0] cos_theta;
    logic signed [15:0] sin_theta;

    logic        [15:0] omega_ref;
    logic        [15:0] omega_meas;
    logic signed [15:0] speed_kp;
    logic signed [31:0] speed_ki_dt;

    logic signed [15:0] id_ref;
    logic signed [15:0] current_kp;
    logic signed [15:0] current_ki;

    logic        done;
    logic signed [15:0] v_alpha;
    logic signed [15:0] v_beta;
    logic signed [15:0] id_meas;
    logic signed [15:0] iq_meas;
    logic signed [15:0] iq_ref;
    logic signed [15:0] v_d;
    logic signed [15:0] v_q;

    int errors = 0;
    int tests = 0;

    foc_shared_2dsp #(
        .KP_FRAC(12)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .start        (start),
        .i_alpha      (i_alpha),
        .i_beta       (i_beta),
        .cos_theta    (cos_theta),
        .sin_theta    (sin_theta),
        .omega_ref    (omega_ref),
        .omega_meas   (omega_meas),
        .speed_kp     (speed_kp),
        .speed_ki_dt  (speed_ki_dt),
        .id_ref       (id_ref),
        .current_kp   (current_kp),
        .current_ki   (current_ki),
        .done         (done),
        .v_alpha      (v_alpha),
        .v_beta       (v_beta),
        .id_meas      (id_meas),
        .iq_meas      (iq_meas),
        .iq_ref       (iq_ref),
        .v_d          (v_d),
        .v_q          (v_q)
    );

    always #4 clk = ~clk; // 125 MHz clock (8 ns period)

    initial begin
        clk         = 0;
        rst         = 1;
        start       = 0;
        i_alpha     = 0;
        i_beta      = 0;
        cos_theta   = 16'sh7fff; // cos(0) = 1.0
        sin_theta   = 16'sh0000; // sin(0) = 0.0
        omega_ref   = 16'd0;
        omega_meas  = 16'd0;
        speed_kp    = 16'sd1311;
        speed_ki_dt = 32'sd53687;
        id_ref      = 16'sd0;
        current_kp  = 16'sd12288; // Kp = 3.0 in Q4.12
        current_ki  = 16'sd819;   // Ki = 0.025 in Q1.15

        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // =====================================================================
        // Test 1: Identity angle theta=0 (cos=32767, sin=0)
        // Ialpha = 10000, Ibeta = 0 -> Id should be ~10000, Iq should be ~0
        // =====================================================================
        tests++;
        $display("[INFO] Running Test 1: Identity angle theta=0, Ialpha=10000");
        @(posedge clk);
        i_alpha   = 16'sd10000;
        i_beta    = 16'sd0;
        cos_theta = 16'sh7fff;
        sin_theta = 16'sh0000;
        omega_ref = 16'd0;
        omega_meas= 16'd0;
        id_ref    = 16'sd0;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        // Wait for done pulse
        @(posedge done);
        #1;
        $display("       id_meas=%0d, iq_meas=%0d, vd=%0d, vq=%0d, valpha=%0d, vbeta=%0d",
                 id_meas, iq_meas, v_d, v_q, v_alpha, v_beta);

        if (abs(id_meas - 10000) <= 2 && abs(iq_meas) <= 2) begin
            $display("[PASS] Test 1: Park transform at theta=0 matches expected values");
        end else begin
            $display("[FAIL] Test 1: Park transform mismatch! id=%0d, iq=%0d", id_meas, iq_meas);
            errors++;
        end

        repeat (5) @(posedge clk);

        // =====================================================================
        // Test 2: Angle theta = 90 deg (cos=0, sin=32767)
        // Ialpha = 0, Ibeta = 10000 -> Id should be ~10000, Iq should be ~0
        // =====================================================================
        tests++;
        $display("[INFO] Running Test 2: Angle theta=90 deg, Ibeta=10000");
        @(posedge clk);
        i_alpha   = 16'sd0;
        i_beta    = 16'sd10000;
        cos_theta = 16'sh0000;
        sin_theta = 16'sh7fff;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        @(posedge done);
        #1;
        $display("       id_meas=%0d, iq_meas=%0d, vd=%0d, vq=%0d, valpha=%0d, vbeta=%0d",
                 id_meas, iq_meas, v_d, v_q, v_alpha, v_beta);

        if (abs(id_meas - 10000) <= 2 && abs(iq_meas) <= 2) begin
            $display("[PASS] Test 2: Park transform at theta=90 deg matches expected values");
        end else begin
            $display("[FAIL] Test 2: Park transform mismatch! id=%0d, iq=%0d", id_meas, iq_meas);
            errors++;
        end

        repeat (5) @(posedge clk);

        // =====================================================================
        // Test 3: Speed PI Step Command
        // omega_ref = 640 (10 rad/s), omega_meas = 0
        // iq_ref should become positive
        // =====================================================================
        tests++;
        $display("[INFO] Running Test 3: Speed PI step response");
        @(posedge clk);
        omega_ref  = 16'd640;
        omega_meas = 16'd0;
        i_alpha    = 16'sd0;
        i_beta     = 16'sd0;
        cos_theta  = 16'sh7fff;
        sin_theta  = 16'sh0000;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        @(posedge done);
        #1;
        $display("       iq_ref=%0d, v_q=%0d", iq_ref, v_q);

        if (iq_ref > 0 && v_q > 0) begin
            $display("[PASS] Test 3: Speed PI and Iq PI generated positive torque/voltage commands");
        end else begin
            $display("[FAIL] Test 3: Expected positive commands, got iq_ref=%0d, v_q=%0d", iq_ref, v_q);
            errors++;
        end

        repeat (5) @(posedge clk);

        // =====================================================================
        // Test 4: Inverse Park rotation check
        // At theta=0, Valpha should equal Vd, Vbeta should equal Vq
        // =====================================================================
        tests++;
        $display("[INFO] Running Test 4: Inverse Park check at theta=0");
        if (abs(v_alpha - v_d) <= 2 && abs(v_beta - v_q) <= 2) begin
            $display("[PASS] Test 4: Inverse Park matches (Valpha=%0d, Vd=%0d, Vbeta=%0d, Vq=%0d)",
                     v_alpha, v_d, v_beta, v_q);
        end else begin
            $display("[FAIL] Test 4: Inverse Park mismatch! Valpha=%0d, Vd=%0d, Vbeta=%0d, Vq=%0d",
                     v_alpha, v_d, v_beta, v_q);
            errors++;
        end

        // =====================================================================
        // Summary
        // =====================================================================
        $display("--------------------------------------------------");
        $display("tb_foc_shared_2dsp summary: Tests=%0d, Errors=%0d", tests, errors);
        $display("--------------------------------------------------");
        if (errors == 0) begin
            $display("[ALL PASS] 2-DSP shared FOC engine verified successfully!");
        end else begin
            $display("[FAIL] Testbench encountered %0d errors.", errors);
        end

        $finish;
    end

    function int abs(input int x);
        return (x >= 0) ? x : -x;
    endfunction

endmodule
