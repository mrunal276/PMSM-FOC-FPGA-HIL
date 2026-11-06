`timescale 1ns / 1ps

/**
 * Top-Level Field-Oriented Control (FOC) Hardware Pipeline Core
 *
 * Integrated Sub-blocks:
 *   1. clarke      : (ia, ib, ic)            -> (i_alpha, i_beta)
 *   2. cordic      : theta_e                 -> (cos_theta, sin_theta) [17 cycles]
 *   3. park        : (i_alpha, i_beta, CORDIC)-> (i_d, i_q)             [1 cycle]
 *   4. speed_pi    : (omega_ref, omega_meas) -> iq_ref                  [1 cycle]
 *   5. current_pi_d: (id_ref, i_d)           -> v_d                     [1 cycle]
 *   6. current_pi_q: (iq_ref, i_q)           -> v_q                     [1 cycle]
 *   7. inv_park    : (v_d, v_q, CORDIC)      -> (v_alpha, v_beta)      [1 cycle]
 *   8. svpwm       : (v_alpha, v_beta)       -> (duty_a, duty_b, duty_c)[2 cycles]
 *
 * Latency Alignment:
 *
 *   Input accepted:             Edge E0
 *   CORDIC result valid:        Edge E17
 *   Park / Speed PI outputs:    Edge E18
 *   Current PI outputs:         Edge E19
 *   Inverse Park output:        Edge E20
 *   SVPWM stage-1 register:     Edge E21
 *   SVPWM output valid:         Edge E22
 *
 * Total Pipeline Latency = 22 clock cycles.
 * (SVPWM pipelined into 2 stages to close timing at 125 MHz)
 */

module foc_pipeline (
    input  logic               clk,
    input  logic               rst,
    input  logic               input_valid,

    // Feedback from Current Sensing & Plant/Rotor Observer
    input  logic signed [15:0] ia_in,        // Q1.15 (1.0 = 20 A)
    input  logic signed [15:0] ib_in,        // Q1.15
    input  logic signed [15:0] ic_in,        // Q1.15
    input  logic        [15:0] theta_e,      // Q2.14 unsigned [0, 2pi), 16384 = pi
    input  logic        [15:0] omega_meas,   // Q10.6 unsigned (64 = 1 rad/s)

    // Setpoint commands
    input  logic        [15:0] omega_ref,    // Q10.6 unsigned
    input  logic signed [15:0] id_ref,       // Q1.15 (typically 0 for SPMSM)

    // Controller Gain Parameters
    input  logic signed [15:0] speed_kp,     // Q1.15 (e.g. 0.04)
    input  logic signed [31:0] speed_ki_dt,  // Q1.31 (Ki*dt e.g. 0.5 * 50us * 2^31 = 53687)
    input  logic signed [15:0] current_kp,   // Q4.12 (e.g. 3.0 * 4096 = 12288)
    input  logic signed [15:0] current_ki,   // Q1.15 (discrete Ki/update e.g. 0.025 * 32768 = 819)

    // Pipeline Outputs
    output logic               output_valid,
    output logic        [15:0] duty_a,       // 0..32767
    output logic        [15:0] duty_b,
    output logic        [15:0] duty_c,
    output logic        [2:0]  sector,

    // Diagnostic / Telemetry Readback
    output logic signed [15:0] i_alpha_out,
    output logic signed [15:0] i_beta_out,
    output logic signed [15:0] id_meas_out,
    output logic signed [15:0] iq_meas_out,
    output logic signed [15:0] iq_ref_out,
    output logic signed [15:0] vd_out,
    output logic signed [15:0] vq_out,
    output logic signed [15:0] valpha_out,
    output logic signed [15:0] vbeta_out
);

    // ------------------------------------------------------------------------
    // Stage 1: Clarke Transform & CORDIC Unit
    // ------------------------------------------------------------------------
    logic               clarke_valid;
    logic signed [15:0] clarke_alpha;
    logic signed [15:0] clarke_beta;

    clarke u_clarke (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (input_valid),
        .ia_in        (ia_in),
        .ib_in        (ib_in),
        .ic_in        (ic_in),
        .output_valid (clarke_valid),
        .i_alpha      (clarke_alpha),
        .i_beta       (clarke_beta)
    );

    logic               cordic_valid;
    logic signed [15:0] cordic_sin;
    logic signed [15:0] cordic_cos;

    cordic u_cordic (
        .clk          (clk),
        .rst          (rst),
        .theta_valid  (input_valid),
        .theta_in     (theta_e),
        .result_valid (cordic_valid),
        .sin_out      (cordic_sin),
        .cos_out      (cordic_cos)
    );

    // ------------------------------------------------------------------------
    // Delay matching:
    // Clarke output ready at cycle 1. CORDIC outputs ready at cycle 18.
    // Delay Clarke outputs by 17 cycles (1 + 17 = 18).
    // ------------------------------------------------------------------------
    localparam int CLARKE_DELAY = 17;

    logic signed [15:0] alpha_delay_pipe [0:CLARKE_DELAY-1];
    logic signed [15:0] beta_delay_pipe  [0:CLARKE_DELAY-1];

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < CLARKE_DELAY; i++) begin
                alpha_delay_pipe[i] <= 16'sd0;
                beta_delay_pipe[i]  <= 16'sd0;
            end
        end else begin
            alpha_delay_pipe[0] <= clarke_alpha;
            beta_delay_pipe[0]  <= clarke_beta;
            for (int i = 1; i < CLARKE_DELAY; i++) begin
                alpha_delay_pipe[i] <= alpha_delay_pipe[i-1];
                beta_delay_pipe[i]  <= beta_delay_pipe[i-1];
            end
        end
    end

    logic signed [15:0] alpha_aligned;
    logic signed [15:0] beta_aligned;
    assign alpha_aligned = alpha_delay_pipe[CLARKE_DELAY-1];
    assign beta_aligned  = beta_delay_pipe[CLARKE_DELAY-1];

    // ------------------------------------------------------------------------
    // Stage 2: Park Transform & Speed PI Loop (at cycle 18 -> ready cycle 19)
    // ------------------------------------------------------------------------
    logic               park_valid;
    logic signed [15:0] park_id;
    logic signed [15:0] park_iq;

    park u_park (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (cordic_valid),
        .i_alpha      (alpha_aligned),
        .i_beta       (beta_aligned),
        .cos_theta    (cordic_cos),
        .sin_theta    (cordic_sin),
        .output_valid (park_valid),
        .i_d          (park_id),
        .i_q          (park_iq)
    );

    // Delay speed references & omega_meas by 18 cycles to enter speed_pi at cycle 18
    localparam int SPEED_DELAY = 18;
    logic [15:0] omega_ref_pipe  [0:SPEED_DELAY-1];
    logic [15:0] omega_meas_pipe [0:SPEED_DELAY-1];

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < SPEED_DELAY; i++) begin
                omega_ref_pipe[i]  <= 16'd0;
                omega_meas_pipe[i] <= 16'd0;
            end
        end else begin
            omega_ref_pipe[0]  <= omega_ref;
            omega_meas_pipe[0] <= omega_meas;
            for (int i = 1; i < SPEED_DELAY; i++) begin
                omega_ref_pipe[i]  <= omega_ref_pipe[i-1];
                omega_meas_pipe[i] <= omega_meas_pipe[i-1];
            end
        end
    end

    logic               speed_pi_valid;
    logic signed [15:0] speed_iq_ref;

    speed_pi u_speed_pi (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (cordic_valid),
        .omega_ref    (omega_ref_pipe[SPEED_DELAY-1]),
        .omega_meas   (omega_meas_pipe[SPEED_DELAY-1]),
        .kp           (speed_kp),
        .ki_dt_q31    (speed_ki_dt),
        .iq_ref       (speed_iq_ref),
        .output_valid (speed_pi_valid)
    );

    // Delay id_ref by 19 cycles to enter current_pi_d at cycle 19
    localparam int ID_REF_DELAY = 19;
    logic signed [15:0] id_ref_pipe [0:ID_REF_DELAY-1];
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < ID_REF_DELAY; i++) id_ref_pipe[i] <= 16'sd0;
        end else begin
            id_ref_pipe[0] <= id_ref;
            for (int i = 1; i < ID_REF_DELAY; i++) id_ref_pipe[i] <= id_ref_pipe[i-1];
        end
    end

    // ------------------------------------------------------------------------
    // Stage 3: Current PI Loops (Id and Iq) (at cycle 19 -> ready cycle 20)
    // KP_FRAC=12 allows gains up to 7.999 (Kp=3.0 = 12288)
    // ------------------------------------------------------------------------
    logic               pi_d_valid;
    logic signed [15:0] v_d_calc;

    current_pi #(
        .KP_FRAC(12)
    ) u_current_pi_d (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (park_valid),
        .ref_in       (id_ref_pipe[ID_REF_DELAY-1]),
        .meas_in      (park_id),
        .kp           (current_kp),
        .ki           (current_ki),
        .output_data  (v_d_calc),
        .output_valid (pi_d_valid)
    );

    logic               pi_q_valid;
    logic signed [15:0] v_q_calc;

    current_pi #(
        .KP_FRAC(12)
    ) u_current_pi_q (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (park_valid),
        .ref_in       (speed_iq_ref),
        .meas_in      (park_iq),
        .kp           (current_kp),
        .ki           (current_ki),
        .output_data  (v_q_calc),
        .output_valid (pi_q_valid)
    );

    // ------------------------------------------------------------------------
    // Stage 4: Inverse Park Transform (at cycle 20 -> ready cycle 21)
    // CORDIC sin/cos delayed by 2 cycles (from cycle 18 to cycle 20)
    // ------------------------------------------------------------------------
    logic signed [15:0] cordic_cos_d1, cordic_cos_d2;
    logic signed [15:0] cordic_sin_d1, cordic_sin_d2;

    always_ff @(posedge clk) begin
        if (rst) begin
            cordic_cos_d1 <= 16'sd0;
            cordic_cos_d2 <= 16'sd0;
            cordic_sin_d1 <= 16'sd0;
            cordic_sin_d2 <= 16'sd0;
        end else begin
            cordic_cos_d1 <= cordic_cos;
            cordic_cos_d2 <= cordic_cos_d1;
            cordic_sin_d1 <= cordic_sin;
            cordic_sin_d2 <= cordic_sin_d1;
        end
    end

    logic               inv_park_valid;
    logic signed [15:0] v_alpha_calc;
    logic signed [15:0] v_beta_calc;

    inv_park u_inv_park (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (pi_d_valid),
        .v_d          (v_d_calc),
        .v_q          (v_q_calc),
        .cos_theta    (cordic_cos_d2),
        .sin_theta    (cordic_sin_d2),
        .output_valid (inv_park_valid),
        .v_alpha      (v_alpha_calc),
        .v_beta       (v_beta_calc)
    );

    // ------------------------------------------------------------------------
    // Stage 5: Space Vector PWM (at cycle 21 -> ready cycle 22)
    // ------------------------------------------------------------------------
    logic               svpwm_valid;
    logic        [15:0] svpwm_da;
    logic        [15:0] svpwm_db;
    logic        [15:0] svpwm_dc;
    logic        [2:0]  svpwm_sec;

    svpwm u_svpwm (
        .clk          (clk),
        .rst          (rst),
        .input_valid  (inv_park_valid),
        .v_alpha      (v_alpha_calc),
        .v_beta       (v_beta_calc),
        .duty_a       (svpwm_da),
        .duty_b       (svpwm_db),
        .duty_c       (svpwm_dc),
        .sector       (svpwm_sec),
        .output_valid (svpwm_valid)
    );

    // ------------------------------------------------------------------------
    // Pipeline Outputs
    // ------------------------------------------------------------------------
    assign output_valid = svpwm_valid;
    assign duty_a       = svpwm_da;
    assign duty_b       = svpwm_db;
    assign duty_c       = svpwm_dc;
    assign sector       = svpwm_sec;

    assign i_alpha_out  = alpha_aligned;
    assign i_beta_out   = beta_aligned;
    assign id_meas_out  = park_id;
    assign iq_meas_out  = park_iq;
    assign iq_ref_out   = speed_iq_ref;
    assign vd_out       = v_d_calc;
    assign vq_out       = v_q_calc;
    assign valpha_out   = v_alpha_calc;
    assign vbeta_out    = v_beta_calc;

endmodule
