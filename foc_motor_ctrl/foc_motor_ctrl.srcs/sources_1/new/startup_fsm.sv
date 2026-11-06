`timescale 1ns/1ps

module startup_fsm #(
    parameter int ALIGN_CYCLES  = 10_000_000, // 2000 ms @ 100 MHz
    parameter int RAMP_CYCLES   = 5_000_000,  // 1000 ms ramp
    parameter int RAMP_STEPS    = 100,        // iq_ref incremented this many times during ramp
    parameter int ID_ALIGN      = 3277,       // 0.1 pu * 32768 in Q1.15
    parameter int IQ_RAMP_FINAL = 6554,       // 0.2 pu final ramp iq
    parameter int OMEGA_HANDOVER= 640,        // 10 rad/s in Q10.6 (64*10)
    parameter int HANDOVER_CYCLES = 5000      // 1 control period
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        start,
    input  logic [15:0] omega_meas,      // Q10.6 unsigned
    input  logic        fault,
    output logic        foc_enable,      // 1 = closed-loop active
    output logic        open_loop_active,
    output logic [15:0] id_ref_override, // Q1.15 signed stored in 16-bit
    output logic [15:0] iq_ref_override, // Q1.15
    output logic [15:0] theta_override,  // Q2.14
    output logic [2:0]  state_out        // 0=IDLE, 1=ALIGN, 2=RAMP, 3=HANDOVER, 4=RUNNING
);

    typedef enum logic [2:0] {
        ST_IDLE     = 3'd0,
        ST_ALIGN    = 3'd1,
        ST_RAMP     = 3'd2,
        ST_HANDOVER = 3'd3,
        ST_RUNNING  = 3'd4
    } state_t;

    state_t state, next_state;
    logic [31:0] cycle_cnt;
    logic [31:0] step_timer;
    logic [31:0] current_step;

    localparam int CYCLES_PER_STEP = (RAMP_STEPS > 0) ? (RAMP_CYCLES / RAMP_STEPS) : RAMP_CYCLES;
    localparam int THETA_INC = (RAMP_STEPS > 0) ? (4096 / RAMP_STEPS) : 40;

    assign state_out = state;

    always_ff @(posedge clk) begin
        if (rst || fault) begin
            state            <= ST_IDLE;
            cycle_cnt        <= 32'd0;
            step_timer       <= 32'd0;
            current_step     <= 32'd0;
            foc_enable       <= 1'b0;
            open_loop_active <= 1'b0;
            id_ref_override  <= 16'd0;
            iq_ref_override  <= 16'd0;
            theta_override   <= 16'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    foc_enable       <= 1'b0;
                    open_loop_active <= 1'b0;
                    id_ref_override  <= 16'd0;
                    iq_ref_override  <= 16'd0;
                    theta_override   <= 16'd0;
                    cycle_cnt        <= 32'd0;
                    step_timer       <= 32'd0;
                    current_step     <= 32'd0;
                    if (start) begin
                        state            <= ST_ALIGN;
                        open_loop_active <= 1'b1;
                        id_ref_override  <= 16'(ID_ALIGN);
                    end
                end

                ST_ALIGN: begin
                    open_loop_active <= 1'b1;
                    foc_enable       <= 1'b0;
                    id_ref_override  <= 16'(ID_ALIGN);
                    iq_ref_override  <= 16'd0;
                    theta_override   <= 16'd0;
                    if (cycle_cnt >= ALIGN_CYCLES - 1) begin
                        state      <= ST_RAMP;
                        cycle_cnt  <= 32'd0;
                        step_timer <= 32'd0;
                    end else begin
                        cycle_cnt <= cycle_cnt + 32'd1;
                    end
                end

                ST_RAMP: begin
                    open_loop_active <= 1'b1;
                    foc_enable       <= 1'b0;
                    id_ref_override  <= 16'(ID_ALIGN);
                    cycle_cnt        <= cycle_cnt + 32'd1;

                    if (step_timer >= CYCLES_PER_STEP - 1) begin
                        step_timer <= 32'd0;
                        if (current_step < RAMP_STEPS) begin
                            current_step    <= current_step + 32'd1;
                            theta_override  <= (theta_override + 16'(THETA_INC)) & 16'h7FFF;
                            iq_ref_override <= 16'(((current_step + 1) * IQ_RAMP_FINAL) / RAMP_STEPS);
                        end
                    end else begin
                        step_timer <= step_timer + 32'd1;
                    end

                    if (cycle_cnt >= RAMP_CYCLES - 1) begin
                        state      <= ST_HANDOVER;
                        cycle_cnt  <= 32'd0;
                        foc_enable <= 1'b1;
                    end
                end

                ST_HANDOVER: begin
                    foc_enable <= 1'b1;
                    if (cycle_cnt >= HANDOVER_CYCLES - 1) begin
                        open_loop_active <= 1'b0;
                        id_ref_override  <= 16'd0;
                        iq_ref_override  <= 16'd0;
                        state            <= ST_RUNNING;
                        cycle_cnt        <= 32'd0;
                    end else begin
                        cycle_cnt <= cycle_cnt + 32'd1;
                    end
                end

                ST_RUNNING: begin
                    foc_enable       <= 1'b1;
                    open_loop_active <= 1'b0;
                    id_ref_override  <= 16'd0;
                    iq_ref_override  <= 16'd0;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
