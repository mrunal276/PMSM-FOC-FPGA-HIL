
`timescale 1ns/1ps

module tb_dead_time;

    localparam integer DEAD_CYCLES = 2;

    logic clk = 1'b0;
    logic rst = 1'b1;
    logic fault = 1'b0;

    logic pwm_a = 1'b0;
    logic pwm_b = 1'b0;
    logic pwm_c = 1'b0;

    logic gate_ah, gate_al;
    logic gate_bh, gate_bl;
    logic gate_ch, gate_cl;

    integer errors = 0;
    integer checks = 0;

    always #5 clk = ~clk;

    dead_time #(
        .DEAD_CYCLES(DEAD_CYCLES)
    ) dut (
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

    // Change requests away from the active edge, then allow
    // the synchronous RTL to process the new request.
    task automatic drive_pwm(
        input logic a,
        input logic b,
        input logic c
    );
        begin
            @(negedge clk);
            pwm_a = a;
            pwm_b = b;
            pwm_c = c;

            @(posedge clk);
            #1;
        end
    endtask

    // Check the mutually exclusive gate outputs.
    task automatic check_no_overlap(input string test_name);
        begin
            checks = checks + 1;

            if ((gate_ah && gate_al) ||
                (gate_bh && gate_bl) ||
                (gate_ch && gate_cl)) begin
                $display("FAIL: %s - simultaneous gate outputs",
                         test_name);
                errors = errors + 1;
            end else begin
                $display("PASS: %s - no overlap", test_name);
            end
        end
    endtask

    // Check that all six outputs are LOW.
    task automatic check_all_low(input string test_name);
        begin
            checks = checks + 1;

            if (gate_ah !== 1'b0 || gate_al !== 1'b0 ||
                gate_bh !== 1'b0 || gate_bl !== 1'b0 ||
                gate_ch !== 1'b0 || gate_cl !== 1'b0) begin
                $display("FAIL: %s - gate outputs not all LOW",
                         test_name);
                errors = errors + 1;
            end else begin
                $display("PASS: %s", test_name);
            end
        end
    endtask

    // Continuous safety monitor.
    always @(negedge clk) begin
        if (!rst && !fault) begin
            if ((gate_ah && gate_al) ||
                (gate_bh && gate_bl) ||
                (gate_ch && gate_cl)) begin
                $display("FAIL: continuous no-overlap monitor");
                errors = errors + 1;
            end
        end
    end

    initial begin
        $display("========================================");
        $display(" DEAD-TIME TESTBENCH");
        $display(" Clock frequency = 100 MHz");
        $display(" DEAD_CYCLES = %0d", DEAD_CYCLES);
        $display(" Expected dead time = %0d ns",
                 DEAD_CYCLES * 10);
        $display("========================================");

        // ------------------------------------------------
        // 1. Reset safety and initial LOW request
        // ------------------------------------------------
        repeat (3) @(negedge clk);
        #1;
        check_all_low("Reset forces all gates LOW");

        rst = 1'b0;

        // The default request is LOW.
        repeat (4) @(negedge clk);
        #1;
        check_no_overlap("Initial low-side turn-on");

        if (gate_al !== 1'b1 || gate_bl !== 1'b1 ||
            gate_cl !== 1'b1) begin
            $display("FAIL: Initial low-side gates did not turn ON");
            errors = errors + 1;
        end else begin
            $display("PASS: Initial low-side gates turn ON");
        end

        // ------------------------------------------------
        // 2. LOW -> HIGH
        // ------------------------------------------------
        drive_pwm(1'b1, 1'b1, 1'b1);

        if (gate_al !== 1'b0 || gate_bl !== 1'b0 ||
            gate_cl !== 1'b0) begin
            $display("FAIL: Low-side gates did not turn OFF");
            errors = errors + 1;
        end else begin
            $display("PASS: Low-side gates turn OFF on transition");
        end

        // One clock after the turn-off edge: still in dead time.
        @(posedge clk);
        #1;
        check_no_overlap("HIGH transition during dead time");

        if (gate_ah !== 1'b0 || gate_bh !== 1'b0 ||
            gate_ch !== 1'b0) begin
            $display("FAIL: High-side turned ON too early");
            errors = errors + 1;
        end else begin
            $display("PASS: High-side remains OFF during dead time");
        end

        // Second clock after turn-off: opposite side may turn ON.
        @(posedge clk);
        #1;
        check_no_overlap("HIGH transition after dead time");

        if (gate_ah !== 1'b1 || gate_bh !== 1'b1 ||
            gate_ch !== 1'b1) begin
            $display("FAIL: High-side gates did not turn ON");
            errors = errors + 1;
        end else begin
            $display("PASS: High-side gates turn ON after dead time");
        end

        // ------------------------------------------------
        // 3. HIGH -> LOW
        // ------------------------------------------------
        drive_pwm(1'b0, 1'b0, 1'b0);

        if (gate_ah !== 1'b0 || gate_bh !== 1'b0 ||
            gate_ch !== 1'b0) begin
            $display("FAIL: High-side gates did not turn OFF");
            errors = errors + 1;
        end else begin
            $display("PASS: High-side gates turn OFF on transition");
        end

// One clock after turn-off: low-side must still be OFF.
@(posedge clk);
#1;
check_no_overlap("LOW transition during dead time");

if (gate_al !== 1'b0 || gate_bl !== 1'b0 ||
    gate_cl !== 1'b0) begin
    $display("FAIL: Low-side turned ON too early");
    errors = errors + 1;
end else begin
    $display("PASS: Low-side remains OFF during dead time");
end

        // Second clock after turn-off: opposite side may turn ON.
        @(posedge clk);
        #1;
        check_no_overlap("LOW transition after dead time");

        if (gate_al !== 1'b1 || gate_bl !== 1'b1 ||
            gate_cl !== 1'b1) begin
            $display("FAIL: Low-side gates did not turn ON");
            errors = errors + 1;
        end else begin
            $display("PASS: Low-side gates turn ON after dead time");
        end

        // ------------------------------------------------
        // 4. Rapid changes
        // ------------------------------------------------
        drive_pwm(1'b1, 1'b0, 1'b1);
        drive_pwm(1'b0, 1'b1, 1'b0);
        drive_pwm(1'b1, 1'b1, 1'b1);
        drive_pwm(1'b0, 1'b0, 1'b0);

        repeat (8) begin
            @(negedge clk);
            #1;
            check_no_overlap("Rapid request changes");
        end

        // ------------------------------------------------
        // 5. Fault safety
        // ------------------------------------------------
        @(negedge clk);
        fault = 1'b1;
        #1;
        check_all_low("Fault forces all gates LOW");

        // ------------------------------------------------
        // 6. Reset safety
        // ------------------------------------------------
        @(negedge clk);
        fault = 1'b0;
        rst = 1'b1;
        #1;
        check_all_low("Reset forces all gates LOW");

        repeat (2) @(negedge clk);
        rst = 1'b0;

        repeat (5) @(negedge clk);
        #1;
        check_no_overlap("Operation after reset");

        // ------------------------------------------------
        // Summary
        // ------------------------------------------------
        $display("----------------------------------------");
        $display("Total explicit checks: %0d", checks);
        $display("Total errors: %0d", errors);

        if (errors == 0)
            $display("FINAL RESULT: PASS");
        else
            $display("FINAL RESULT: FAIL");

        $display("----------------------------------------");
        $finish;
    end

endmodule
