
`timescale 1ns/1ps

module tb_pwm_generator;

    logic clk = 1'b0;
    logic rst = 1'b1;
    logic fault = 1'b0;

    logic duty_valid = 1'b0;
    logic [15:0] duty_a = 16'd0;
    logic [15:0] duty_b = 16'd0;
    logic [15:0] duty_c = 16'd0;

    logic pwm_a, pwm_b, pwm_c;
    logic carrier_sync;

    integer errors = 0;
    integer sync_count = 0;
    integer cycles_since_sync = 0;
    integer period_checks = 0;

    // 100 MHz clock = 10 ns period.
    always #5 clk = ~clk;

    pwm_generator dut (
        .clk          (clk),
        .rst          (rst),
        .fault        (fault),
        .duty_valid   (duty_valid),
        .duty_a       (duty_a),
        .duty_b       (duty_b),
        .duty_c       (duty_c),
        .pwm_a        (pwm_a),
        .pwm_b        (pwm_b),
        .pwm_c        (pwm_c),
        .carrier_sync (carrier_sync)
    );

    task automatic check_outputs_low(input string test_name);
        begin
            #1;
            if (pwm_a !== 1'b0 ||
                pwm_b !== 1'b0 ||
                pwm_c !== 1'b0) begin
                $display("FAIL: %s - outputs not all LOW", test_name);
                errors = errors + 1;
            end else begin
                $display("PASS: %s", test_name);
            end
        end
    endtask

    task automatic send_duties(
        input logic [15:0] a,
        input logic [15:0] b,
        input logic [15:0] c
    );
        begin
            @(negedge clk);
            duty_a = a;
            duty_b = b;
            duty_c = c;
            duty_valid = 1'b1;

            @(negedge clk);
            duty_valid = 1'b0;
        end
    endtask

    // Monitor the interval between carrier_sync pulses.
    // Ignore the first pulse after reset; subsequent pulses
    // should be separated by exactly 5000 clocks.
    always @(posedge clk) begin
        if (rst) begin
            cycles_since_sync <= 0;
            sync_count <= 0;
        end else begin
            cycles_since_sync <= cycles_since_sync + 1;

            if (carrier_sync) begin
                if (sync_count > 0) begin
                    if ((cycles_since_sync + 1) != 5000) begin
    $display(
        "FAIL: carrier period = %0d clocks, expected 5000",
        cycles_since_sync + 1
    );
    errors <= errors + 1;
end else begin
    period_checks <= period_checks + 1;
end
                end

                sync_count <= sync_count + 1;
                cycles_since_sync <= 0;
            end
        end
    end

    initial begin
        $display("========================================");
        $display(" PWM GENERATOR TESTBENCH");
        $display(" Clock = 100 MHz");
        $display(" Target PWM = 20 kHz");
        $display(" Expected period = 5000 clocks");
        $display("========================================");

        // Reset.
        repeat (5) @(negedge clk);
        rst = 1'b0;

        check_outputs_low("Reset releases with zero duty");

        // Test 0% duty on all phases.
        send_duties(16'd0, 16'd0, 16'd0);
        repeat (5100) @(negedge clk);

        if (pwm_a === 1'b0 && pwm_b === 1'b0 && pwm_c === 1'b0)
            $display("PASS: 0%% duty outputs stay LOW");
        else begin
            $display("FAIL: 0%% duty");
            errors = errors + 1;
        end

        // Test 50% duty on all phases.
        send_duties(16'd16384, 16'd16384, 16'd16384);
        repeat (5100) @(negedge clk);

        // Count high samples across a full period.
        begin : measure_half_duty
            integer high_samples;
            integer i;

            high_samples = 0;

            for (i = 0; i < 5000; i = i + 1) begin
                @(negedge clk);
                if (pwm_a === 1'b1)
                    high_samples = high_samples + 1;
            end

            $display("50%% duty: phase A HIGH for %0d/5000 samples",
                     high_samples);

            if (high_samples >= 2490 && high_samples <= 2510)
                $display("PASS: 50%% duty within tolerance");
            else begin
                $display("FAIL: 50%% duty sample count");
                errors = errors + 1;
            end
        end

        // Test maximum duty.
        send_duties(16'd32767, 16'd32767, 16'd32767);
        repeat (5100) @(negedge clk);

        if (pwm_a === 1'b1 && pwm_b === 1'b1 && pwm_c === 1'b1)
            $display("PASS: Maximum duty outputs stay HIGH");
        else begin
            $display("FAIL: Maximum duty");
            errors = errors + 1;
        end

        // Test fault safety.
        @(negedge clk);
        fault = 1'b1;
        check_outputs_low("Fault forces all outputs LOW");

        @(negedge clk);
        fault = 1'b0;

        // Test reset safety.
        @(negedge clk);
        rst = 1'b1;
        check_outputs_low("Reset forces all outputs LOW");

        repeat (3) @(negedge clk);
        rst = 1'b0;

        // Allow carrier monitor to observe several complete periods.
        repeat (16000) @(negedge clk);

        $display("----------------------------------------");
        $display("Carrier period checks passed: %0d", period_checks);
        $display("Total errors: %0d", errors);

        if (errors == 0 && period_checks >= 2)
            $display("FINAL RESULT: PASS");
        else
            $display("FINAL RESULT: FAIL");

        $display("----------------------------------------");
        $finish;
    end

endmodule
