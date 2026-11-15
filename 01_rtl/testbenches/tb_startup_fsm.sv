`timescale 1ns/1ps

module tb_startup_fsm;
    logic        clk;
    logic        rst;
    logic        start;
    logic [15:0] omega_meas;
    logic        fault;
    logic        foc_enable;
    logic        open_loop_active;
    logic [15:0] id_ref_override;
    logic [15:0] iq_ref_override;
    logic [15:0] theta_override;
    logic [2:0]  state_out;

    localparam int ALIGN_CYCLES    = 20;
    localparam int RAMP_CYCLES     = 40;
    localparam int RAMP_STEPS      = 4;
    localparam int ID_ALIGN        = 3277;
    localparam int IQ_RAMP_FINAL   = 6554;
    localparam int OMEGA_HANDOVER  = 640;
    localparam int HANDOVER_CYCLES = 10;

    startup_fsm #(
        .ALIGN_CYCLES   (ALIGN_CYCLES),
        .RAMP_CYCLES    (RAMP_CYCLES),
        .RAMP_STEPS     (RAMP_STEPS),
        .ID_ALIGN       (ID_ALIGN),
        .IQ_RAMP_FINAL  (IQ_RAMP_FINAL),
        .OMEGA_HANDOVER (OMEGA_HANDOVER),
        .HANDOVER_CYCLES(HANDOVER_CYCLES)
    ) dut (
        .clk             (clk),
        .rst             (rst),
        .start           (start),
        .omega_meas      (omega_meas),
        .fault           (fault),
        .foc_enable      (foc_enable),
        .open_loop_active(open_loop_active),
        .id_ref_override (id_ref_override),
        .iq_ref_override (iq_ref_override),
        .theta_override  (theta_override),
        .state_out       (state_out)
    );

    int errors = 0;
    int tests = 0;

    always #5 clk = ~clk; // 100 MHz

    initial begin
        $timeformat(-9, 2, " ns", 10);
        clk = 0;
        rst = 1;
        start = 0;
        omega_meas = 16'd0;
        fault = 0;

        repeat (5) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // Test 1: Reset behavior
        tests++;
        if (state_out === 3'd0 && foc_enable === 1'b0 && open_loop_active === 1'b0) begin
            $display("[PASS] Test 1: Reset behavior verified (state=IDLE)");
        end else begin
            $display("[FAIL] Test 1: Reset failed, state=%d", state_out);
            errors++;
        end

        // Test 2: Start -> ALIGN
        tests++;
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;
        @(posedge clk);
        if (state_out === 3'd1 && open_loop_active === 1'b1 && id_ref_override === 16'(ID_ALIGN)) begin
            $display("[PASS] Test 2: Start trigger entered ALIGN state");
        end else begin
            $display("[FAIL] Test 2: State=%d, open_loop_active=%b, id_ref=%d", state_out, open_loop_active, id_ref_override);
            errors++;
        end

        // Test 3: ALIGN -> RAMP
        tests++;
        repeat (ALIGN_CYCLES + 2) @(posedge clk);
        if (state_out === 3'd2) begin
            $display("[PASS] Test 3: ALIGN completed, entered RAMP state");
        end else begin
            $display("[FAIL] Test 3: Expected RAMP (2), got state=%d", state_out);
            errors++;
        end

        // Test 4: RAMP -> HANDOVER
        tests++;
        repeat (RAMP_CYCLES + 2) @(posedge clk);
        if (state_out === 3'd3 && foc_enable === 1'b1) begin
            $display("[PASS] Test 4: RAMP completed, entered HANDOVER state with foc_enable=1");
        end else begin
            $display("[FAIL] Test 4: Expected HANDOVER (3), got state=%d, foc_enable=%b", state_out, foc_enable);
            errors++;
        end

        // Test 5: HANDOVER -> RUNNING
        tests++;
        repeat (HANDOVER_CYCLES + 2) @(posedge clk);
        if (state_out === 3'd4 && foc_enable === 1'b1 && open_loop_active === 1'b0) begin
            $display("[PASS] Test 5: HANDOVER completed, entered RUNNING state");
        end else begin
            $display("[FAIL] Test 5: Expected RUNNING (4), got state=%d, open_loop_active=%b", state_out, open_loop_active);
            errors++;
        end

        // Test 6: Fault clears to IDLE
        tests++;
        fault = 1;
        @(posedge clk);
        @(posedge clk);
        if (state_out === 3'd0 && foc_enable === 1'b0) begin
            $display("[PASS] Test 6: Fault cleared state to IDLE");
        end else begin
            $display("[FAIL] Test 6: Expected IDLE (0) on fault, got %d", state_out);
            errors++;
        end

        repeat (5) @(posedge clk);

        $display("----------------------------------------------");
        $display("tb_startup_fsm summary: Tests=%0d, Errors=%0d", tests, errors);
        $display("----------------------------------------------");

        if (errors > 0) begin
            $fatal(0, "tb_startup_fsm verification FAILED");
        end else begin
            $display("[ALL PASS] tb_startup_fsm verified successfully.");
            $finish;
        end
    end
endmodule
