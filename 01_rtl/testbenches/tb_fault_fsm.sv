`timescale 1ns/1ps

module tb_fault_fsm;
    logic               clk;
    logic               rst;
    logic signed [15:0] i_alpha;
    logic signed [15:0] i_beta;
    logic        [15:0] omega_meas;
    logic signed [15:0] iq_ref;
    logic               external_fault;
    logic               fault_active;
    logic               pwm_inhibit;
    logic [2:0]         fault_code;

    localparam int OC_THRESHOLD_SQ  = 26214;
    localparam int IQ_STALL_THRESH  = 26214;
    localparam int OMEGA_MIN_THRESH = 128;
    localparam int STALL_TIMEOUT    = 20;

    fault_fsm #(
        .OC_THRESHOLD_SQ (OC_THRESHOLD_SQ),
        .IQ_STALL_THRESH (IQ_STALL_THRESH),
        .OMEGA_MIN_THRESH(OMEGA_MIN_THRESH),
        .STALL_TIMEOUT   (STALL_TIMEOUT)
    ) dut (
        .clk           (clk),
        .rst           (rst),
        .i_alpha       (i_alpha),
        .i_beta        (i_beta),
        .omega_meas    (omega_meas),
        .iq_ref        (iq_ref),
        .external_fault(external_fault),
        .fault_active  (fault_active),
        .pwm_inhibit   (pwm_inhibit),
        .fault_code    (fault_code)
    );

    int errors = 0;
    int tests = 0;

    always #5 clk = ~clk;

    initial begin
        $timeformat(-9, 2, " ns", 10);
        clk = 0;
        rst = 1;
        i_alpha = 0;
        i_beta = 0;
        omega_meas = 16'd640;
        iq_ref = 0;
        external_fault = 0;

        repeat (5) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        // Test 1: No fault
        tests++;
        if (fault_active === 1'b0 && pwm_inhibit === 1'b0 && fault_code === 3'd0) begin
            $display("[PASS] Test 1: Normal operation, no fault detected");
        end else begin
            $display("[FAIL] Test 1: Unexpected fault state: active=%b, code=%d", fault_active, fault_code);
            errors++;
        end

        // Test 2: Overcurrent fault
        tests++;
        i_alpha = 16'sd30000;
        i_beta  = 16'sd0;
        repeat (2) @(posedge clk);
        if (fault_active === 1'b1 && pwm_inhibit === 1'b1 && fault_code === 3'd1) begin
            $display("[PASS] Test 2: Overcurrent detected within 2 cycles (code=1, pwm_inhibit=1)");
        end else begin
            $display("[FAIL] Test 2: OC not tripped properly: active=%b, code=%d", fault_active, fault_code);
            errors++;
        end

        // Test 3: Latch check
        tests++;
        i_alpha = 16'sd0;
        repeat (5) @(posedge clk);
        if (fault_active === 1'b1 && pwm_inhibit === 1'b1) begin
            $display("[PASS] Test 3: Fault state is latched after current returns to 0");
        end else begin
            $display("[FAIL] Test 3: Fault failed to latch");
            errors++;
        end

        // Test 4: Reset clears fault
        tests++;
        rst = 1;
        repeat (2) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);
        if (fault_active === 1'b0 && pwm_inhibit === 1'b0 && fault_code === 3'd0) begin
            $display("[PASS] Test 4: Reset cleared fault state");
        end else begin
            $display("[FAIL] Test 4: Reset failed to clear fault");
            errors++;
        end

        // Test 5: Stall fault
        tests++;
        iq_ref     = 16'sd28000;
        omega_meas = 16'd64; // < 128
        repeat (STALL_TIMEOUT + 3) @(posedge clk);
        if (fault_active === 1'b1 && fault_code === 3'd2) begin
            $display("[PASS] Test 5: Stall fault detected after timeout (code=2)");
        end else begin
            $display("[FAIL] Test 5: Stall fault not tripped: active=%b, code=%d", fault_active, fault_code);
            errors++;
        end

        // Test 6: External fault
        tests++;
        rst = 1;
        repeat (2) @(posedge clk);
        rst = 0;
        iq_ref = 0;
        omega_meas = 16'd640;
        repeat (2) @(posedge clk);
        external_fault = 1;
        repeat (2) @(posedge clk);
        if (fault_active === 1'b1 && fault_code === 3'd3) begin
            $display("[PASS] Test 6: External fault detected (code=3)");
        end else begin
            $display("[FAIL] Test 6: External fault failed: active=%b, code=%d", fault_active, fault_code);
            errors++;
        end

        $display("----------------------------------------------");
        $display("tb_fault_fsm summary: Tests=%0d, Errors=%0d", tests, errors);
        $display("----------------------------------------------");

        if (errors > 0) begin
            $fatal(0, "tb_fault_fsm verification FAILED");
        end else begin
            $display("[ALL PASS] tb_fault_fsm verified successfully.");
            $finish;
        end
    end
endmodule
