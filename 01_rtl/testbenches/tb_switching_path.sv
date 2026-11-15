`timescale 1ns/1ps

module tb_switching_path;

    localparam integer DEAD_CYCLES = 2;
    localparam integer PWM_PERIOD  = 5000;

    logic clk = 1'b0;
    logic rst = 1'b1;
    logic fault = 1'b0;

    // SVPWM input
    logic input_valid = 1'b0;
    logic signed [15:0] v_alpha = 16'sd0;
    logic signed [15:0] v_beta  = 16'sd0;

    // SVPWM output
    logic [15:0] duty_a, duty_b, duty_c;
    logic [2:0] sector;
    logic svpwm_valid;

    // PWM generator output
    logic pwm_a, pwm_b, pwm_c;
    logic carrier_sync;

    // Dead-time output
    logic gate_ah, gate_al;
    logic gate_bh, gate_bl;
    logic gate_ch, gate_cl;

    integer errors = 0;
    integer vector_checks = 0;
    integer carrier_sync_count = 0;
    integer carrier_period_checks = 0;
    integer cycles_since_sync = 0;
    integer cycle_count = 0;
    integer deadtime_checks = 0;

    // Per-phase dead-time monitoring.
    // State: 0 = both OFF, 1 = HIGH ON, 2 = LOW ON.
    logic [1:0] previous_state [0:2];
    logic [1:0] current_state;
    integer off_cycle [0:2];
    logic waiting_for_opposite [0:2];

    integer i;

    always #5 clk = ~clk;

    // ------------------------------------------------------------
    // DUT 1: SVPWM
    // ------------------------------------------------------------
    svpwm u_svpwm (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (input_valid),
        .v_alpha      (v_alpha),
        .v_beta       (v_beta),
        .duty_a       (duty_a),
        .duty_b       (duty_b),
        .duty_c       (duty_c),
        .sector       (sector),
        .output_valid (svpwm_valid)
    );

    // ------------------------------------------------------------
    // DUT 2: PWM Generator
    // ------------------------------------------------------------
    pwm_generator u_pwm_generator (
        .clk          (clk),
        .rst          (rst),
        .fault        (fault),
        .duty_valid   (svpwm_valid),
        .duty_a       (duty_a),
        .duty_b       (duty_b),
        .duty_c       (duty_c),
        .pwm_a        (pwm_a),
        .pwm_b        (pwm_b),
        .pwm_c        (pwm_c),
        .carrier_sync (carrier_sync)
    );

    // ------------------------------------------------------------
    // DUT 3: Dead-Time Generator
    // ------------------------------------------------------------
    dead_time #(
        .DEAD_CYCLES(DEAD_CYCLES)
    ) u_dead_time (
        .clk     (clk),
        .rst     (rst),
        .fault   (fault),
        .pwm_a   (pwm_a),
        .pwm_b   (pwm_b),
        .pwm_c   (pwm_c),
        .gate_ah (gate_ah),
        .gate_al (gate_al),
        .gate_bh (gate_bh),
        .gate_bl (gate_bl),
        .gate_ch (gate_ch),
        .gate_cl (gate_cl)
    );

    // ------------------------------------------------------------
    // Continuous safety and timing monitor
    // ------------------------------------------------------------
    always @(posedge clk) begin
        #1;

        if (rst || fault) begin
            cycle_count = 0;

            for (i = 0; i < 3; i = i + 1) begin
                previous_state[i] = 2'd0;
                off_cycle[i] = 0;
                waiting_for_opposite[i] = 1'b0;
            end

            if (gate_ah !== 1'b0 || gate_al !== 1'b0 ||
                gate_bh !== 1'b0 || gate_bl !== 1'b0 ||
                gate_ch !== 1'b0 || gate_cl !== 1'b0) begin
                $display("FAIL: A gate remains ON during reset/fault");
                errors = errors + 1;
            end
        end
        else begin
            cycle_count = cycle_count + 1;

            // No high-side/low-side overlap.
            if ((gate_ah && gate_al) ||
                (gate_bh && gate_bl) ||
                (gate_ch && gate_cl)) begin
                $display("FAIL: High/low gate overlap at cycle %0d",
                         cycle_count);
                errors = errors + 1;
            end

            // Phase A state.
            current_state = gate_ah ? 2'd1 :
                            gate_al ? 2'd2 : 2'd0;

            if (previous_state[0] != 0 && current_state == 0) begin
                off_cycle[0] = cycle_count;
                waiting_for_opposite[0] = 1'b1;
            end
            else if (waiting_for_opposite[0] && current_state != 0) begin
                if ((cycle_count - off_cycle[0]) != DEAD_CYCLES) begin
                    $display(
                        "FAIL: Phase A dead time = %0d clocks, expected %0d",
                        cycle_count - off_cycle[0], DEAD_CYCLES
                    );
                    errors = errors + 1;
                end
                else begin
                    deadtime_checks = deadtime_checks + 1;
                end
                waiting_for_opposite[0] = 1'b0;
            end
            previous_state[0] = current_state;

            // Phase B state.
            current_state = gate_bh ? 2'd1 :
                            gate_bl ? 2'd2 : 2'd0;

            if (previous_state[1] != 0 && current_state == 0) begin
                off_cycle[1] = cycle_count;
                waiting_for_opposite[1] = 1'b1;
            end
            else if (waiting_for_opposite[1] && current_state != 0) begin
                if ((cycle_count - off_cycle[1]) != DEAD_CYCLES) begin
                    $display(
                        "FAIL: Phase B dead time = %0d clocks, expected %0d",
                        cycle_count - off_cycle[1], DEAD_CYCLES
                    );
                    errors = errors + 1;
                end
                else begin
                    deadtime_checks = deadtime_checks + 1;
                end
                waiting_for_opposite[1] = 1'b0;
            end
            previous_state[1] = current_state;

            // Phase C state.
            current_state = gate_ch ? 2'd1 :
                            gate_cl ? 2'd2 : 2'd0;

            if (previous_state[2] != 0 && current_state == 0) begin
                off_cycle[2] = cycle_count;
                waiting_for_opposite[2] = 1'b1;
            end
            else if (waiting_for_opposite[2] && current_state != 0) begin
                if ((cycle_count - off_cycle[2]) != DEAD_CYCLES) begin
                    $display(
                        "FAIL: Phase C dead time = %0d clocks, expected %0d",
                        cycle_count - off_cycle[2], DEAD_CYCLES
                    );
                    errors = errors + 1;
                end
                else begin
                    deadtime_checks = deadtime_checks + 1;
                end
                waiting_for_opposite[2] = 1'b0;
            end
            previous_state[2] = current_state;
        end
    end

    // ------------------------------------------------------------
    // Carrier period monitor
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            cycles_since_sync = 0;
            carrier_sync_count = 0;
        end
        else begin
            cycles_since_sync = cycles_since_sync + 1;

            if (carrier_sync) begin
                if (carrier_sync_count > 0) begin
                    if (cycles_since_sync != PWM_PERIOD) begin
                        $display(
                            "FAIL: Carrier period = %0d, expected %0d",
                            cycles_since_sync, PWM_PERIOD
                        );
                        errors = errors + 1;
                    end
                    else begin
                        carrier_period_checks =
                            carrier_period_checks + 1;
                    end
                end

                carrier_sync_count = carrier_sync_count + 1;
                cycles_since_sync = 0;
            end
        end
    end

    // ------------------------------------------------------------
    // Apply one vector and verify the registered SVPWM result.
    // Wait for a carrier valley so the PWM can apply the new duty.
    // ------------------------------------------------------------
    task automatic apply_vector(
        input logic signed [15:0] alpha,
        input logic signed [15:0] beta,
        input logic [2:0] expected_sector
    );
        begin
            @(negedge clk);
            v_alpha = alpha;
            v_beta = beta;
            input_valid = 1'b1;

            @(negedge clk);
            input_valid = 1'b0;
            #1;

            vector_checks = vector_checks + 1;

            if (svpwm_valid !== 1'b1) begin
                $display("FAIL: SVPWM output_valid was not asserted");
                errors = errors + 1;
            end
            else if (sector !== expected_sector) begin
                $display(
                    "FAIL: alpha=%0d beta=%0d sector=%0d expected=%0d",
                    alpha, beta, sector, expected_sector
                );
                errors = errors + 1;
            end
            else begin
                $display(
                    "PASS: SVPWM sector %0d, duties A/B/C = %0d/%0d/%0d",
                    sector, duty_a, duty_b, duty_c
                );
            end

            // Wait for the next carrier valley.
            @(posedge carrier_sync);
            repeat (3) @(posedge clk);

            // Allow a complete PWM period for switching activity.
            repeat (PWM_PERIOD) @(negedge clk);
        end
    endtask

    // ------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------
    initial begin
        $display("================================================");
        $display(" COMBINED SWITCHING PATH TESTBENCH");
        $display(" SVPWM -> PWM Generator -> Dead-Time Generator");
        $display(" Clock             = 100 MHz");
        $display(" PWM frequency     = 20 kHz");
        $display(" Carrier period    = 5000 clocks");
        $display(" Dead time         = %0d clocks", DEAD_CYCLES);
        $display("================================================");

        // Reset all modules.
        repeat (5) @(negedge clk);
        rst = 1'b0;

        // Allow the initial low-side requests to settle.
        repeat (5) @(negedge clk);

        // Exercise all six SVPWM sectors.
        apply_vector(16'sd12000,  16'sd0,      3'd1);
        apply_vector(16'sd6000,   16'sd12000,  3'd2);
        apply_vector(-16'sd12000, 16'sd12000,  3'd3);
        apply_vector(-16'sd12000, 16'sd0,      3'd4);
        apply_vector(-16'sd6000, -16'sd12000,  3'd5);
        apply_vector(16'sd12000, -16'sd6000,   3'd6);

        // Zero vector: all three duties should be approximately 50%.
        apply_vector(16'sd0, 16'sd0, 3'd1);

        // Fault safety.
        @(negedge clk);
        fault = 1'b1;
        #1;

        if (gate_ah === 1'b0 && gate_al === 1'b0 &&
            gate_bh === 1'b0 && gate_bl === 1'b0 &&
            gate_ch === 1'b0 && gate_cl === 1'b0)
            $display("PASS: Fault forces all six gates LOW");
        else begin
            $display("FAIL: Fault safety");
            errors = errors + 1;
        end

        // Release fault and check reset safety.
        @(negedge clk);
        fault = 1'b0;
        rst = 1'b1;
        #1;

        if (gate_ah === 1'b0 && gate_al === 1'b0 &&
            gate_bh === 1'b0 && gate_bl === 1'b0 &&
            gate_ch === 1'b0 && gate_cl === 1'b0)
            $display("PASS: Reset forces all six gates LOW");
        else begin
            $display("FAIL: Reset safety");
            errors = errors + 1;
        end

        repeat (3) @(negedge clk);
        rst = 1'b0;

        // Allow monitors to observe additional carrier cycles.
        repeat (11000) @(negedge clk);

        $display("");
        $display("=============== TEST SUMMARY ===============");
        $display("SVPWM vectors checked       : %0d", vector_checks);
        $display("Carrier period checks       : %0d",
                 carrier_period_checks);
        $display("Exact dead-time transitions : %0d",
                 deadtime_checks);
        $display("Total errors                : %0d", errors);

        if (errors == 0 &&
            vector_checks == 7 &&
            carrier_period_checks >= 2 &&
            deadtime_checks > 0)
            $display("SWITCHING PATH VERIFICATION: PASS");
        else
            $display("SWITCHING PATH VERIFICATION: FAIL");

        $display("============================================");
        $finish;
    end

endmodule
