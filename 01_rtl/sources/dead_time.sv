
module dead_time #(
    parameter integer DEAD_CYCLES = 2
) (
    input  logic clk,
    input  logic rst,       // Active-high reset
    input  logic fault,     // Active-high safety override

    input  logic pwm_a,
    input  logic pwm_b,
    input  logic pwm_c,

    output logic gate_ah,
    output logic gate_al,
    output logic gate_bh,
    output logic gate_bl,
    output logic gate_ch,
    output logic gate_cl
);

    // State encoding for each inverter phase.
    localparam logic [1:0] OFF  = 2'd0;
    localparam logic [1:0] HIGH = 2'd1;
    localparam logic [1:0] LOW  = 2'd2;

    localparam integer COUNT_WIDTH =
        (DEAD_CYCLES <= 1) ? 1 : $clog2(DEAD_CYCLES + 1);

    logic [2:0] pwm_request;
    logic [1:0] phase_state [0:2];
    logic [COUNT_WIDTH-1:0] dead_count [0:2];
    logic [2:0] pending_target;

    assign pwm_request = {pwm_c, pwm_b, pwm_a};

    initial begin
        if (DEAD_CYCLES < 1)
            $error("DEAD_CYCLES must be at least 1");
    end

    // Each phase is controlled independently.
    // HIGH -> OFF -> wait -> LOW
    // LOW  -> OFF -> wait -> HIGH
    genvar i;
    generate
        for (i = 0; i < 3; i = i + 1) begin : gen_dead_time
            always_ff @(posedge clk) begin
                if (rst || fault) begin
                    phase_state[i]   <= OFF;
                    dead_count[i]     <= '0;
                    pending_target[i] <= 1'b0;
                end else begin
                    case (phase_state[i])

                        HIGH: begin
                            if (!pwm_request[i]) begin
                                // Turn HIGH off immediately.
                                // LOW cannot turn on yet.
                                phase_state[i]   <= OFF;
                                dead_count[i]     <= '0;
                                pending_target[i] <= 1'b0;
                            end
                        end

                        LOW: begin
                            if (pwm_request[i]) begin
                                // Turn LOW off immediately.
                                // HIGH cannot turn on yet.
                                phase_state[i]   <= OFF;
                                dead_count[i]     <= '0;
                                pending_target[i] <= 1'b1;
                            end
                        end

                        OFF: begin
                            if (pwm_request[i] != pending_target[i]) begin
                                // The requested direction changed while
                                // waiting. Restart the dead-time interval.
                                pending_target[i] <= pwm_request[i];
                                dead_count[i]     <= '0;
                            end else if (dead_count[i] >= DEAD_CYCLES - 1) begin
                                // Dead time has elapsed; enable only
                                // the requested switch.
                                if (pending_target[i])
                                    phase_state[i] <= HIGH;
                                else
                                    phase_state[i] <= LOW;

                                dead_count[i] <= '0;
                            end else begin
                                dead_count[i] <= dead_count[i] + 1'b1;
                            end
                        end

                        default: begin
                            phase_state[i] <= OFF;
                            dead_count[i]  <= '0;
                        end

                    endcase
                end
            end
        end
    endgenerate

    // Gate outputs are decoded from mutually exclusive states.
    // Force every gate LOW during reset or fault.
    always_comb begin
        gate_ah = 1'b0;
        gate_al = 1'b0;
        gate_bh = 1'b0;
        gate_bl = 1'b0;
        gate_ch = 1'b0;
        gate_cl = 1'b0;

        if (!rst && !fault) begin
            gate_ah = (phase_state[0] == HIGH);
            gate_al = (phase_state[0] == LOW);

            gate_bh = (phase_state[1] == HIGH);
            gate_bl = (phase_state[1] == LOW);

            gate_ch = (phase_state[2] == HIGH);
            gate_cl = (phase_state[2] == LOW);
        end
    end

endmodule
