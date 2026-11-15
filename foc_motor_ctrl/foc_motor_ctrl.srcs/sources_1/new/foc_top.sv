`timescale 1ns/1ps

module foc_top #(
    parameter bit USE_INTERNAL_HIL = 1,          // 1 = Self-contained HIL plant for Zybo Z7-10 demo
    parameter int ALIGN_CYCLES     = 50_000_000, // 0.4s @ 125 MHz
    parameter int RAMP_CYCLES      = 50_000_000, // 0.4s @ 125 MHz
    parameter int RAMP_STEPS       = 100,
    parameter int ID_ALIGN         = 3277,
    parameter int IQ_RAMP_FINAL    = 6554,
    parameter int OMEGA_HANDOVER   = 640,
    parameter int HANDOVER_CYCLES  = 5000,
    parameter int DEAD_CYCLES      = 2,
    parameter int UART_CLKS_BIT    = 1085,       // 125 MHz / 115200 baud = 1085
    parameter bit ENABLE_ILA       = 1           // 1 = Real-time on-chip ILA logic analyzer
)(
    input  logic        clk,          // 125 MHz onboard oscillator (Zybo Z7-10 pin K17)
    input  logic        btn_rst,      // Active-high reset push-button (Btn0 pin K18)
    input  logic        start_btn,    // Active-high start push-button (Btn1 pin P16)
    input  logic        ext_irq,      // Active-high external fault push-button (Btn2 pin K19)

    // Inverter Gate signals (Pmod Header JB pins V15, W15, T11, T10, W14, Y14)
    output logic        gate_ah, gate_al,
    output logic        gate_bh, gate_bl,
    output logic        gate_ch, gate_cl,

    // UART Telemetry output (Pmod Header JA pin N15)
    output logic        uart_tx,

    // 4 Diagnostic LEDs on Zybo Z7 (LD0=M14, LD1=M15, LD2=G14, LD3=D18)
    output logic        led_alive,    // LD0: Heartbeat blinker (~1.86 Hz)
    output logic        led_startup,  // LD1: Align/Ramp open-loop active
    output logic        led_running,  // LD2: Closed-loop FOC running
    output logic        led_fault     // LD3: Fault active indicator
);

    // ------------------------------------------------------------------------
    // Power-On Reset (POR) & Button Synchronization
    // ------------------------------------------------------------------------
    // Automatically releases reset after first 1000 clock cycles on bitstream load
    logic [9:0] por_count = 10'd0;
    logic       por_done  = 1'b0;

    always_ff @(posedge clk) begin
        if (por_count < 10'd1000) begin
            por_count <= por_count + 10'd1;
            por_done  <= 1'b0;
        end else begin
            por_done  <= 1'b1;
        end
    end

    // 2-stage synchronizers for buttons (Zybo pushbuttons are active-HIGH)
    logic btn_rst_s1, btn_rst_s2;
    logic start_btn_s1, start_btn_s2;
    logic ext_irq_s1, ext_irq_s2;

    always_ff @(posedge clk) begin
        btn_rst_s1   <= btn_rst;
        btn_rst_s2   <= btn_rst_s1;
        start_btn_s1 <= start_btn;
        start_btn_s2 <= start_btn_s1;
        ext_irq_s1   <= ext_irq;
        ext_irq_s2   <= ext_irq_s1;
    end

    // Active-high reset: asserted during initial POR or when BTN0 is pressed
    logic rst;
    assign rst = (!por_done) || btn_rst_s2;

    // Heartbeat LED generator: toggles LD0 at ~1.86 Hz from 125 MHz clock
    logic [25:0] heartbeat_cnt = 26'd0;
    always_ff @(posedge clk) begin
        heartbeat_cnt <= heartbeat_cnt + 26'd1;
    end
    assign led_alive = heartbeat_cnt[25];

    // ------------------------------------------------------------------------
    // Interconnect Signals
    // ------------------------------------------------------------------------
    // FSM Control
    logic        foc_enable;
    logic        open_loop_active;
    logic [15:0] id_ref_override;
    logic [15:0] iq_ref_override;
    logic [15:0] theta_override;
    logic [2:0]  fsm_state;

    // Fault Control
    logic        fault_active;
    logic        pwm_inhibit;
    logic [2:0]  fault_code;
    logic        csr_fault_clear;

    // APB Bus
    logic        apb_psel;
    logic        apb_penable;
    logic        apb_pwrite;
    logic [31:0] apb_paddr;
    logic [31:0] apb_pwdata;
    logic [31:0] apb_prdata;
    logic        apb_pready;

    // CSR Outputs
    logic [15:0] csr_speed_ref;
    logic signed [15:0] csr_id_ref;
    logic [15:0] csr_kp_curr;
    logic [15:0] csr_ki_curr;

    // Pipeline Signals
    logic        foc_input_valid;
    logic        foc_output_valid;
    logic [15:0] duty_a, duty_b, duty_c;
    logic [2:0]  sector;
    logic signed [15:0] id_meas, iq_meas, iq_ref_pipe;
    logic signed [15:0] i_alpha_pipe, i_beta_pipe;
    logic signed [15:0] vd_pipe, vq_pipe;
    logic signed [15:0] valpha_pipe, vbeta_pipe;

    // PWM Signals
    logic        carrier_sync;
    logic        pwm_a, pwm_b, pwm_c;

    // ------------------------------------------------------------------------
    // HIL Feedback Generation (pmsm_plant & Inverse Clarke)
    // ------------------------------------------------------------------------
    logic signed [15:0] hil_id, hil_iq;
    logic        [15:0] hil_omega, hil_theta;
    logic signed [15:0] hil_ia, hil_ib, hil_ic;

    pmsm_plant u_plant (
        .clk        (clk),
        .rst        (rst),
        .update     (foc_output_valid),
        .vd_q15     (vd_pipe),
        .vq_q15     (vq_pipe),
        .id_q15     (hil_id),
        .iq_q15     (hil_iq),
        .omega_q6   (hil_omega),
        .theta_q14  (hil_theta),
        .update_done()
    );

    // Inverse Clarke Transform: (I_alpha, I_beta) -> (Ia, Ib, Ic)
    localparam logic signed [15:0] SQRT3_OVER_2_Q15 = 16'sd28378;

    logic signed [31:0] ib_beta_term;
    logic signed [15:0] alpha_pipe_d1;
    logic signed [15:0] hil_ia_r, hil_ib_r, hil_ic_r;
    logic        [15:0] theta_fdbk_r, omega_fdbk_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            ib_beta_term   <= 32'sd0;
            alpha_pipe_d1  <= 16'sd0;
            hil_ia_r       <= 16'sd0;
            hil_ib_r       <= 16'sd0;
            hil_ic_r       <= 16'sd0;
            theta_fdbk_r   <= 16'd0;
            omega_fdbk_r   <= 16'd0;
        end else begin
            // Stage 1: Multiply beta term and buffer alpha/plant signals
            ib_beta_term   <= ($signed(i_beta_pipe) * $signed(SQRT3_OVER_2_Q15)) >>> 15;
            alpha_pipe_d1  <= i_alpha_pipe;
            theta_fdbk_r   <= hil_theta;
            omega_fdbk_r   <= hil_omega;

            // Stage 2: Inverse Clarke additions and registered phase feedback
            hil_ia_r       <= alpha_pipe_d1;
            hil_ib_r       <= 16'(-($signed(alpha_pipe_d1) >>> 1) + $signed(ib_beta_term[15:0]));
            hil_ic_r       <= 16'(-$signed(alpha_pipe_d1) - 16'(-($signed(alpha_pipe_d1) >>> 1) + $signed(ib_beta_term[15:0])));
        end
    end

    assign hil_ia     = hil_ia_r;
    assign hil_ib     = hil_ib_r;
    assign hil_ic     = hil_ic_r;

    // Feedback signals to FOC controller
    logic signed [15:0] ia_fdbk, ib_fdbk, ic_fdbk;
    logic        [15:0] theta_fdbk, omega_fdbk;

    assign ia_fdbk    = hil_ia_r;
    assign ib_fdbk    = hil_ib_r;
    assign ic_fdbk    = hil_ic_r;
    assign theta_fdbk = theta_fdbk_r;
    assign omega_fdbk = omega_fdbk_r;

    // ------------------------------------------------------------------------
    // Multiplexing between Open-Loop Startup and Closed-Loop FOC
    // ------------------------------------------------------------------------
    logic [15:0]        theta_e_mux;
    logic [15:0]        omega_ref_mux;
    logic signed [15:0] id_ref_mux;
    logic signed [15:0] ia_in_mux, ib_in_mux, ic_in_mux;

    assign theta_e_mux   = open_loop_active ? theta_override : theta_fdbk;
    assign omega_ref_mux = open_loop_active ? 16'd0 : ((csr_speed_ref != 16'd0) ? csr_speed_ref : 16'd3200);
    assign id_ref_mux    = open_loop_active ? $signed(id_ref_override) : csr_id_ref;

    // In open loop align/ramp, current feedback is not closed; feed measured or 0
    // Pipelined register stage isolates top-level startup mux from Clarke DSP multipliers and CORDIC
    logic signed [15:0] ia_in_mux_r, ib_in_mux_r, ic_in_mux_r;
    logic        [15:0] theta_e_mux_r;
    always_ff @(posedge clk) begin
        if (rst) begin
            ia_in_mux_r   <= 16'sd0;
            ib_in_mux_r   <= 16'sd0;
            ic_in_mux_r   <= 16'sd0;
            theta_e_mux_r <= 16'd0;
        end else begin
            ia_in_mux_r   <= foc_enable ? ia_fdbk : 16'sd0;
            ib_in_mux_r   <= foc_enable ? ib_fdbk : 16'sd0;
            ic_in_mux_r   <= foc_enable ? ic_fdbk : 16'sd0;
            theta_e_mux_r <= theta_e_mux;
        end
    end

    // Pipeline execution triggered at 20/25 kHz carrier valley
    assign foc_input_valid = carrier_sync && (foc_enable || open_loop_active);

    // Default controller gains
    localparam signed [15:0] SPEED_KP    = 16'sd1311;   // Q1.15
    localparam signed [31:0] SPEED_KI_DT = 32'sd53687;  // Q1.31

    // ------------------------------------------------------------------------
    // Sub-Module Instantiations
    // ------------------------------------------------------------------------
    foc_pipeline u_pipeline (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (foc_input_valid),
        .ia_in        (ia_in_mux_r),
        .ib_in        (ib_in_mux_r),
        .ic_in        (ic_in_mux_r),
        .theta_e      (theta_e_mux_r),
        .omega_meas   (omega_fdbk),
        .omega_ref    (omega_ref_mux),
        .id_ref       (id_ref_mux),
        .speed_kp     (SPEED_KP),
        .speed_ki_dt  (SPEED_KI_DT),
        .current_kp   ($signed(csr_kp_curr)),
        .current_ki   ($signed(csr_ki_curr)),
        .output_valid (foc_output_valid),
        .duty_a       (duty_a),
        .duty_b       (duty_b),
        .duty_c       (duty_c),
        .sector       (sector),
        .i_alpha_out  (i_alpha_pipe),
        .i_beta_out   (i_beta_pipe),
        .id_meas_out  (id_meas),
        .iq_meas_out  (iq_meas),
        .iq_ref_out   (iq_ref_pipe),
        .vd_out       (vd_pipe),
        .vq_out       (vq_pipe),
        .valpha_out   (valpha_pipe),
        .vbeta_out    (vbeta_pipe)
    );

    pwm_generator u_pwm_gen (
        .clk         (clk),
        .rst         (rst),
        .fault       (pwm_inhibit),
        .duty_valid  (foc_output_valid),
        .duty_a      (duty_a),
        .duty_b      (duty_b),
        .duty_c      (duty_c),
        .pwm_a       (pwm_a),
        .pwm_b       (pwm_b),
        .pwm_c       (pwm_c),
        .carrier_sync(carrier_sync)
    );

    dead_time #(.DEAD_CYCLES(DEAD_CYCLES)) u_dead_time (
        .clk    (clk),
        .rst    (rst),
        .fault  (pwm_inhibit),
        .pwm_a  (pwm_a),
        .pwm_b  (pwm_b),
        .pwm_c  (pwm_c),
        .gate_ah(gate_ah),
        .gate_al(gate_al),
        .gate_bh(gate_bh),
        .gate_bl(gate_bl),
        .gate_ch(gate_ch),
        .gate_cl(gate_cl)
    );

    startup_fsm #(
        .ALIGN_CYCLES   (ALIGN_CYCLES),
        .RAMP_CYCLES    (RAMP_CYCLES),
        .RAMP_STEPS     (RAMP_STEPS),
        .ID_ALIGN       (ID_ALIGN),
        .IQ_RAMP_FINAL  (IQ_RAMP_FINAL),
        .OMEGA_HANDOVER (OMEGA_HANDOVER),
        .HANDOVER_CYCLES(HANDOVER_CYCLES)
    ) u_startup (
        .clk             (clk),
        .rst             (rst),
        .start           (start_btn_s2),
        .omega_meas      (omega_fdbk),
        .fault           (fault_active),
        .foc_enable      (foc_enable),
        .open_loop_active(open_loop_active),
        .id_ref_override (id_ref_override),
        .iq_ref_override (iq_ref_override),
        .theta_override  (theta_override),
        .state_out       (fsm_state)
    );

    fault_fsm #(
        .OC_THRESHOLD_SQ (26214),
        .IQ_STALL_THRESH (26214),
        .OMEGA_MIN_THRESH(128),
        .STALL_TIMEOUT   (100_000)
    ) u_fault (
        .clk           (clk),
        .rst           (rst || csr_fault_clear),
        .i_alpha       (i_alpha_pipe),
        .i_beta        (i_beta_pipe),
        .omega_meas    (omega_fdbk),
        .iq_ref        (iq_ref_pipe),
        .external_fault(ext_irq_s2),
        .fault_active  (fault_active),
        .pwm_inhibit   (pwm_inhibit),
        .fault_code    (fault_code)
    );

    apb_csr u_csr (
        .clk        (clk),
        .rst        (rst),
        .apb_psel   (apb_psel),
        .apb_penable(apb_penable),
        .apb_pwrite (apb_pwrite),
        .apb_paddr  (apb_paddr),
        .apb_pwdata (apb_pwdata),
        .apb_prdata (apb_prdata),
        .apb_pready (apb_pready),
        .omega_meas (omega_fdbk),
        .id_meas    (id_meas),
        .iq_meas    (iq_meas),
        .fault_code (fault_code),
        .foc_state  (fsm_state),
        .locked     (foc_enable && !open_loop_active),
        .speed_ref  (csr_speed_ref),
        .id_ref     (csr_id_ref),
        .fault_clear(csr_fault_clear),
        .kp_curr    (csr_kp_curr),
        .ki_curr    (csr_ki_curr)
    );

    rv32i_core u_cpu (
        .clk        (clk),
        .rst        (rst),
        .ext_irq    (ext_irq_s2),
        .apb_psel   (apb_psel),
        .apb_penable(apb_penable),
        .apb_pwrite (apb_pwrite),
        .apb_paddr  (apb_paddr),
        .apb_pwdata (apb_pwdata),
        .apb_prdata (apb_prdata),
        .apb_pready (apb_pready),
        .dbg_pc     (),
        .dbg_x1     (),
        .dbg_x2     (),
        .dbg_x10    ()
    );

    uart_logger #(.CLKS_PER_BIT(UART_CLKS_BIT)) u_uart (
        .clk         (clk),
        .rst         (rst),
        .carrier_sync(carrier_sync),
        .id_q15      (id_meas),
        .iq_q15      (iq_meas),
        .omega_q6    (omega_fdbk),
        .uart_tx     (uart_tx),
        .busy        ()
    );

    // Diagnostic LEDs
    assign led_startup = open_loop_active;
    assign led_running = (fsm_state == 3'd4);
    assign led_fault   = fault_active;

    // ------------------------------------------------------------------------
    // Integrated Logic Analyzer (ILA) Core for Real-Time On-Chip Probing
    // ------------------------------------------------------------------------
    generate
        if (ENABLE_ILA) begin : gen_ila
            ila_0 u_ila (
                .clk    (clk),
                .probe0 (omega_fdbk),
                .probe1 (theta_fdbk),
                .probe2 (id_meas),
                .probe3 (iq_meas),
                .probe4 (duty_a),
                .probe5 (duty_b),
                .probe6 (duty_c),
                .probe7 (fsm_state),
                .probe8 ({foc_enable, open_loop_active, fault_active, carrier_sync, gate_ah, gate_al, gate_bh, gate_bl})
            );
        end
    endgenerate

endmodule
