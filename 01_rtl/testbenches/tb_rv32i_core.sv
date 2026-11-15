`timescale 1ns/1ps

module tb_rv32i_core;
    logic        clk;
    logic        rst;
    logic        ext_irq;

    logic        apb_psel;
    logic        apb_penable;
    logic        apb_pwrite;
    logic [31:0] apb_paddr;
    logic [31:0] apb_pwdata;
    logic [31:0] apb_prdata;
    logic        apb_pready;

    logic [31:0] dbg_pc;
    logic [31:0] dbg_x1;
    logic [31:0] dbg_x2;
    logic [31:0] dbg_x10;

    rv32i_core dut (
        .clk        (clk),
        .rst        (rst),
        .ext_irq    (ext_irq),
        .apb_psel   (apb_psel),
        .apb_penable(apb_penable),
        .apb_pwrite (apb_pwrite),
        .apb_paddr  (apb_paddr),
        .apb_pwdata (apb_pwdata),
        .apb_prdata (apb_prdata),
        .apb_pready (apb_pready),
        .dbg_pc     (dbg_pc),
        .dbg_x1     (dbg_x1),
        .dbg_x2     (dbg_x2),
        .dbg_x10    (dbg_x10)
    );

    // APB Slave Peripheral Model (Memory-mapped registers at 0x4000_0000)
    logic [31:0] slv_regs [0:15];
    int apb_wait_cnt;

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 16; i++) slv_regs[i] <= 32'h0;
            slv_regs[2] <= 32'h0000_0029; // STATUS = 0x29
            apb_pready   <= 1'b0;
            apb_prdata   <= 32'd0;
            apb_wait_cnt <= 0;
        end else begin
            apb_pready <= 1'b0;
            if (apb_psel && apb_penable) begin
                apb_wait_cnt <= apb_wait_cnt + 1;
                if (apb_wait_cnt >= 1) begin
                    apb_pready   <= 1'b1;
                    apb_wait_cnt <= 0;
                    if (apb_pwrite) begin
                        slv_regs[(apb_paddr - 32'h4000_0000) >> 2] <= apb_pwdata;
                    end else begin
                        apb_prdata <= slv_regs[(apb_paddr - 32'h4000_0000) >> 2];
                    end
                end
            end else begin
                apb_wait_cnt <= 0;
            end
        end
    end

    int errors = 0;
    int tests = 0;

    always #5 clk = ~clk;

    initial begin
        $timeformat(-9, 2, " ns", 10);
        clk = 0;
        rst = 1;
        ext_irq = 0;

        repeat (5) @(posedge clk);
        rst = 0;

        // Run until CPU completes program
        repeat (60) @(posedge clk);

        // Test 1: Verify APB write to SPEED_REF (slv_regs[0])
        tests++;
        if (slv_regs[0] === 32'h000A_0000) begin
            $display("[PASS] Test 1: APB write SPEED_REF = 0x%08h", slv_regs[0]);
        end else begin
            $display("[FAIL] Test 1: Expected slv_regs[0] = 0x000A0000, got 0x%08h", slv_regs[0]);
            errors++;
        end

        // Test 2: Verify APB readback into x10 (dbg_x10)
        tests++;
        if (dbg_x10 === 32'h000A_0000) begin
            $display("[PASS] Test 2: APB readback into x10 = 0x%08h", dbg_x10);
        end else begin
            $display("[FAIL] Test 2: Expected dbg_x10 = 0x000A0000, got 0x%08h", dbg_x10);
            errors++;
        end

        // Test 3: Verify APB write to ID_REF (slv_regs[1])
        tests++;
        if (slv_regs[1] === 32'h0000_0000) begin
            $display("[PASS] Test 3: APB write ID_REF = 0x%08h", slv_regs[1]);
        end else begin
            $display("[FAIL] Test 3: Expected slv_regs[1] = 0x00000000, got 0x%08h", slv_regs[1]);
            errors++;
        end

        // Test 4: Verify PC reaches 0x18 (JAL infinite loop)
        tests++;
        if (dbg_pc === 32'h0000_0018) begin
            $display("[PASS] Test 4: Program reached terminal loop at PC = 0x%08h", dbg_pc);
        end else begin
            $display("[FAIL] Test 4: Expected PC = 0x18, got 0x%08h", dbg_pc);
            errors++;
        end

        $display("----------------------------------------------");
        $display("tb_rv32i_core summary: Tests=%0d, Errors=%0d", tests, errors);
        $display("----------------------------------------------");

        if (errors > 0) begin
            $fatal(0, "tb_rv32i_core verification FAILED");
        end else begin
            $display("[ALL PASS] tb_rv32i_core verified successfully.");
            $finish;
        end
    end
endmodule
