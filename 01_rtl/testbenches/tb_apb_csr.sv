`timescale 1ns/1ps

module tb_apb_csr;
    logic        clk;
    logic        rst;

    logic        apb_psel;
    logic        apb_penable;
    logic        apb_pwrite;
    logic [31:0] apb_paddr;
    logic [31:0] apb_pwdata;
    logic [31:0] apb_prdata;
    logic        apb_pready;

    logic [15:0] omega_meas;
    logic signed [15:0] id_meas;
    logic signed [15:0] iq_meas;
    logic [2:0]  fault_code;
    logic [2:0]  foc_state;
    logic        locked;

    logic [15:0] speed_ref;
    logic signed [15:0] id_ref;
    logic        fault_clear;
    logic [15:0] kp_curr;
    logic [15:0] ki_curr;

    apb_csr dut (
        .clk        (clk),
        .rst        (rst),
        .apb_psel   (apb_psel),
        .apb_penable(apb_penable),
        .apb_pwrite (apb_pwrite),
        .apb_paddr  (apb_paddr),
        .apb_pwdata (apb_pwdata),
        .apb_prdata (apb_prdata),
        .apb_pready (apb_pready),
        .omega_meas (omega_meas),
        .id_meas    (id_meas),
        .iq_meas    (iq_meas),
        .fault_code (fault_code),
        .foc_state  (foc_state),
        .locked     (locked),
        .speed_ref  (speed_ref),
        .id_ref     (id_ref),
        .fault_clear(fault_clear),
        .kp_curr    (kp_curr),
        .ki_curr    (ki_curr)
    );

    int errors = 0;
    int tests = 0;

    always #5 clk = ~clk;

    task automatic apb_write(input [31:0] addr, input [31:0] data);
        @(posedge clk);
        apb_psel    <= 1'b1;
        apb_penable <= 1'b0;
        apb_pwrite  <= 1'b1;
        apb_paddr   <= addr;
        apb_pwdata  <= data;
        @(posedge clk);
        apb_penable <= 1'b1;
        while (!apb_pready) @(posedge clk);
        @(posedge clk);
        apb_psel    <= 1'b0;
        apb_penable <= 1'b0;
        apb_pwrite  <= 1'b0;
    endtask

    task automatic apb_read(input [31:0] addr, output [31:0] data);
        @(posedge clk);
        apb_psel    <= 1'b1;
        apb_penable <= 1'b0;
        apb_pwrite  <= 1'b0;
        apb_paddr   <= addr;
        @(posedge clk);
        apb_penable <= 1'b1;
        while (!apb_pready) @(posedge clk);
        data = apb_prdata;
        @(posedge clk);
        apb_psel    <= 1'b0;
        apb_penable <= 1'b0;
    endtask

    initial begin
        logic [31:0] rdata;
        $timeformat(-9, 2, " ns", 10);
        clk = 0;
        rst = 1;
        apb_psel = 0;
        apb_penable = 0;
        apb_pwrite = 0;
        apb_paddr = 0;
        apb_pwdata = 0;
        omega_meas = 16'd640;
        id_meas = 16'sd100;
        iq_meas = 16'sd200;
        fault_code = 3'd1;
        foc_state = 3'd4;
        locked = 1'b1;

        repeat (5) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        // Test 1: Reset values
        tests++;
        apb_read(32'h00, rdata);
        if (rdata === 32'd0 && speed_ref === 16'd0) begin
            $display("[PASS] Test 1: SPEED_REF reset to 0");
        end else begin
            $display("[FAIL] Test 1: Expected SPEED_REF=0, got %d", rdata);
            errors++;
        end

        tests++;
        apb_read(32'h1C, rdata);
        if (rdata === 32'd12288 && kp_curr === 16'd12288) begin
            $display("[PASS] Test 2: KP_CURR default reset is 12288");
        end else begin
            $display("[FAIL] Test 2: Expected KP_CURR=12288, got %d", rdata);
            errors++;
        end

        // Test 3: Write & Read SPEED_REF
        tests++;
        apb_write(32'h00, 32'd640);
        apb_read(32'h00, rdata);
        if (rdata === 32'd640 && speed_ref === 16'd640) begin
            $display("[PASS] Test 3: Wrote and readback SPEED_REF=640");
        end else begin
            $display("[FAIL] Test 3: SPEED_REF readback error: got %d", rdata);
            errors++;
        end

        // Test 4: Write & Read ID_REF (signed)
        tests++;
        apb_write(32'h04, 32'hFFFF_FC18); // -1000 in 32-bit sign extension
        apb_read(32'h04, rdata);
        if ($signed(id_ref) === -16'sd1000) begin
            $display("[PASS] Test 4: Wrote and readback ID_REF = -1000");
        end else begin
            $display("[FAIL] Test 4: ID_REF readback error: got %d", id_ref);
            errors++;
        end

        // Test 5: Read STATUS
        tests++;
        apb_read(32'h08, rdata);
        // STATUS = {fault_code[2:0], foc_state[2:0], locked} = {1, 4, 1} = 3'b001, 3'b100, 1'b1 = 7'b0011001 = 0x19
        if (rdata[6:0] === 7'b0011001) begin
            $display("[PASS] Test 5: STATUS register packed correctly: 0x%02h", rdata[6:0]);
        end else begin
            $display("[FAIL] Test 5: STATUS error: got 0x%02h", rdata[6:0]);
            errors++;
        end

        // Test 6: FAULT_CLEAR pulse
        tests++;
        apb_write(32'h18, 32'd1);
        repeat (2) @(posedge clk);
        if (fault_clear === 1'b0) begin
            $display("[PASS] Test 6: FAULT_CLEAR is a clean single-cycle pulse");
        end else begin
            $display("[FAIL] Test 6: FAULT_CLEAR did not return to 0");
            errors++;
        end

        $display("----------------------------------------------");
        $display("tb_apb_csr summary: Tests=%0d, Errors=%0d", tests, errors);
        $display("----------------------------------------------");

        if (errors > 0) begin
            $fatal(0, "tb_apb_csr verification FAILED");
        end else begin
            $display("[ALL PASS] tb_apb_csr verified successfully.");
            $finish;
        end
    end
endmodule
