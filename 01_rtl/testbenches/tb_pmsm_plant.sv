`timescale 1ns/1ps

module tb_pmsm_plant;
    logic        clk;
    logic        rst;
    logic        update;
    logic signed [15:0] vd_q15;
    logic signed [15:0] vq_q15;
    logic signed [15:0] id_q15;
    logic signed [15:0] iq_q15;
    logic        [15:0] omega_q6;
    logic        [15:0] theta_q14;
    logic               update_done;

    pmsm_plant dut (
        .clk        (clk),
        .rst        (rst),
        .update     (update),
        .vd_q15     (vd_q15),
        .vq_q15     (vq_q15),
        .id_q15     (id_q15),
        .iq_q15     (iq_q15),
        .omega_q6   (omega_q6),
        .theta_q14  (theta_q14),
        .update_done(update_done)
    );

    int errors = 0;
    int steps_tested = 0;

    always #5 clk = ~clk;

    initial begin
        $timeformat(-9, 2, " ns", 10);
        clk    = 0;
        rst    = 1;
        update = 0;
        vd_q15 = 16'sd0;
        vq_q15 = 16'sd13653; // 10.0 / 24.0 * 32768 = 13653.33

        repeat (5) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        // Open golden CSV
        begin
            int fd;
            string line;
            real t_val, vd_val, vq_val, id_val, iq_val, wm_val, th_val;
            int r;
            int exp_id_q15, exp_iq_q15;
            int diff_id, diff_iq;

            fd = $fopen("c:/Users/Mrunal/rtl_design_verif_project/foc_motor_ctrl/golden/pmsm_plant_openloop.csv", "r");
            if (fd == 0) begin
                $fatal(0, "Could not open pmsm_plant_openloop.csv");
            end

            // Read header
            r = $fgets(line, fd);

            // Read first row (t=0)
            r = $fscanf(fd, "%f,%f,%f,%f,%f,%f,%f\n", t_val, vd_val, vq_val, id_val, iq_val, wm_val, th_val);

            // Run 50 steps
            for (int step = 1; step <= 50; step++) begin
                r = $fscanf(fd, "%f,%f,%f,%f,%f,%f,%f\n", t_val, vd_val, vq_val, id_val, iq_val, wm_val, th_val);

                // Pulse update
                @(posedge clk);
                update = 1;
                @(posedge clk);
                update = 0;

                // Wait for update_done
                while (!update_done) @(posedge clk);

                // Compute expected Q1.15
                exp_id_q15 = $rtoi(id_val / 20.0 * 32768.0);
                exp_iq_q15 = $rtoi(iq_val / 20.0 * 32768.0);

                diff_id = id_q15 - exp_id_q15;
                if (diff_id < 0) diff_id = -diff_id;

                diff_iq = iq_q15 - exp_iq_q15;
                if (diff_iq < 0) diff_iq = -diff_iq;

                steps_tested++;

                if (diff_id > 3 || diff_iq > 5) begin
                    $display("[FAIL] Step %0d: id_act=%0d exp=%0d (diff=%0d), iq_act=%0d exp=%0d (diff=%0d)",
                             step, id_q15, exp_id_q15, diff_id, iq_q15, exp_iq_q15, diff_iq);
                    errors++;
                end else if (step % 10 == 0 || step == 50) begin
                    $display("[PASS] Step %0d: id=%0d (exp=%0d, diff=%0d), iq=%0d (exp=%0d, diff=%0d)",
                             step, id_q15, exp_id_q15, diff_id, iq_q15, exp_iq_q15, diff_iq);
                end

                repeat (3) @(posedge clk);
            end

            $fclose(fd);
        end

        $display("----------------------------------------------");
        $display("tb_pmsm_plant summary: Steps=%0d, Errors=%0d", steps_tested, errors);
        $display("----------------------------------------------");

        if (errors > 0) begin
            $fatal(0, "tb_pmsm_plant verification FAILED");
        end else begin
            $display("[ALL PASS] tb_pmsm_plant verified successfully.");
            $finish;
        end
    end
endmodule
