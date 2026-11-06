`timescale 1ns/1ps

module tb_foc_top;
    logic        clk;
    logic        btn_rst;
    logic        start_btn;

    logic        gate_ah, gate_al;
    logic        gate_bh, gate_bl;
    logic        gate_ch, gate_cl;

    logic        uart_tx;
    logic        led_alive;
    logic        led_startup;
    logic        led_running;
    logic        led_fault;
    logic        ext_irq;

    // Fast simulation parameters with internal HIL plant
    foc_top #(
        .USE_INTERNAL_HIL(1),
        .ALIGN_CYCLES    (20),
        .RAMP_CYCLES     (40),
        .RAMP_STEPS      (4),
        .ID_ALIGN        (3277),
        .IQ_RAMP_FINAL   (6554),
        .OMEGA_HANDOVER  (640),
        .HANDOVER_CYCLES (10),
        .DEAD_CYCLES     (2),
        .UART_CLKS_BIT   (10),
        .ENABLE_ILA      (0)
    ) dut (
        .clk          (clk),
        .btn_rst      (btn_rst),
        .start_btn    (start_btn),
        .gate_ah      (gate_ah),
        .gate_al      (gate_al),
        .gate_bh      (gate_bh),
        .gate_bl      (gate_bl),
        .gate_ch      (gate_ch),
        .gate_cl      (gate_cl),
        .uart_tx      (uart_tx),
        .led_alive    (led_alive),
        .led_startup  (led_startup),
        .led_running  (led_running),
        .led_fault    (led_fault),
        .ext_irq      (ext_irq)
    );

    int errors = 0;
    int tests = 0;
    int shoot_through_count = 0;

    always #4 clk = ~clk; // 125 MHz clock (8 ns period)

    // Continuous shoot-through safety monitor
    always @(posedge clk) begin
        if (gate_ah && gate_al) shoot_through_count++;
        if (gate_bh && gate_bl) shoot_through_count++;
        if (gate_ch && gate_cl) shoot_through_count++;
    end

    initial begin
        $timeformat(-9, 2, " ns", 10);
        clk           = 0;
        btn_rst       = 1;
        start_btn     = 0;
        ext_irq       = 0;

        repeat (20) @(posedge clk);
        // Wait past POR (1000 cycles) or release reset
        btn_rst = 0;
        repeat (1050) @(posedge clk);

        // Test 1: Reset state
        tests++;
        if (led_fault === 1'b0 && led_running === 1'b0) begin
            $display("[PASS] Test 1: System reset clean (led_fault=0, led_running=0, led_alive=%b)", led_alive);
        end else begin
            $display("[FAIL] Test 1: System reset failed");
            errors++;
        end

        // Test 2: Start trigger
        tests++;
        start_btn = 1;
        repeat (3) @(posedge clk); // Hold button across 2-stage synchronizer
        start_btn = 0;
        repeat (10) @(posedge clk);
        if (dut.open_loop_active === 1'b1 && led_startup === 1'b1) begin
            $display("[PASS] Test 2: Start triggered ALIGN state (open_loop_active=1, led_startup=1)");
        end else begin
            $display("[FAIL] Test 2: Startup FSM did not enter open-loop, open_loop=%b, led_startup=%b",
                     dut.open_loop_active, led_startup);
            errors++;
        end

        // Wait through ALIGN (20) + RAMP (40) + HANDOVER (10) = ~80 cycles
        repeat (100) @(posedge clk);

        // Test 3: RUNNING state reached
        tests++;
        if (led_running === 1'b1 && dut.foc_enable === 1'b1 && led_startup === 1'b0) begin
            $display("[PASS] Test 3: System reached closed-loop RUNNING state (led_running=1, led_startup=0)");
        end else begin
            $display("[FAIL] Test 3: Expected RUNNING state, fsm_state=%d", dut.fsm_state);
            errors++;
        end

        // Test 4: Shoot-through safety across transitions
        tests++;
        if (shoot_through_count == 0) begin
            $display("[PASS] Test 4: Zero shoot-through violations across all gate outputs");
        end else begin
            $display("[FAIL] Test 4: Shoot-through detected! Count=%0d", shoot_through_count);
            errors++;
        end

        // Run through at least one full 20/25 kHz PWM carrier period (5000 cycles)
        repeat (5200) @(posedge clk);

        // Test 5: Inverter switching active during running
        tests++;
        if (!dut.pwm_inhibit) begin
            $display("[PASS] Test 5: Inverter gates operating normally in closed-loop");
        end else begin
            $display("[FAIL] Test 5: Inverter inhibited unexpectedly");
            errors++;
        end

        // Test 6: External IRQ / Fault trigger
        tests++;
        ext_irq = 1;
        repeat (4) @(posedge clk); // Hold across 2-stage synchronizer
        ext_irq = 0;
        repeat (10) @(posedge clk);
        if (dut.fault_active === 1'b1 && led_fault === 1'b1) begin
            $display("[PASS] Test 6: External fault dispatched successfully (fault_active=1, led_fault=1)");
        end else begin
            $display("[FAIL] Test 6: External fault not registered");
            errors++;
        end

        // Test 7: UART telemetry activity
        tests++;
        if (dut.u_uart.timestamp > 0) begin
            $display("[PASS] Test 7: UART telemetry active (logged %0d frames)", dut.u_uart.timestamp);
        end else begin
            $display("[FAIL] Test 7: No UART frames logged");
            errors++;
        end

        $display("----------------------------------------------");
        $display("tb_foc_top summary: Tests=%0d, Errors=%0d", tests, errors);
        $display("----------------------------------------------");

        if (errors > 0) begin
            $fatal(0, "tb_foc_top integration verification FAILED");
        end else begin
            $display("[ALL PASS] tb_foc_top integration verified successfully!");
            $finish;
        end
    end
endmodule
