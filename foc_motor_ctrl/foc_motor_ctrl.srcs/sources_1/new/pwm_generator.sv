
module pwm_generator (
    input  logic        clk,
    input  logic        rst,          // Active-high reset
    input  logic        fault,        // Active-high safety override

    input  logic        duty_valid,
    input  logic [15:0] duty_a,       // Unsigned Q1.15: 0 to 32767
    input  logic [15:0] duty_b,
    input  logic [15:0] duty_c,

    output logic        pwm_a,
    output logic        pwm_b,
    output logic        pwm_c,

    output logic        carrier_sync   // One-clock pulse at carrier valley
);

    // 100 MHz clock / 20 kHz PWM = 5000 clocks per period.
    // The triangular counter traverses 0 -> 2500 -> 0.
    localparam logic [11:0] HALF_PERIOD = 12'd2500;

    logic [11:0] carrier_count;
    logic        count_up;

    // Duty update buffering:
    // duty_valid captures new values into shadow registers.
    // Active thresholds change only at the next carrier valley.
    logic [15:0] duty_a_shadow;
    logic [15:0] duty_b_shadow;
    logic [15:0] duty_c_shadow;

    logic [11:0] threshold_a_active;
    logic [11:0] threshold_b_active;
    logic [11:0] threshold_c_active;

    logic [11:0] threshold_a_shadow;
    logic [11:0] threshold_b_shadow;
    logic [11:0] threshold_c_shadow;

    logic        shadow_pending;

    // Convert unsigned Q1.15 duty to a threshold from 0 to 2500.
    // Round to nearest integer. Values above 32767 are clamped.
    function automatic logic [11:0] duty_to_threshold(
        input logic [15:0] duty
    );
        logic [15:0] duty_clamped;
        logic [31:0] product;
        logic [31:0] rounded_value;
        begin
            if (duty > 16'd32767)
                duty_clamped = 16'd32767;
            else
                duty_clamped = duty;

            product = duty_clamped * 32'd2500;
            rounded_value = (product + 32'd16383) / 32'd32767;

            duty_to_threshold = rounded_value[11:0];
        end
    endfunction

    // Capture incoming duty values. Apply a complete set together
    // at a subsequent carrier valley.
    always_ff @(posedge clk) begin
        if (rst) begin
            duty_a_shadow <= 16'd0;
            duty_b_shadow <= 16'd0;
            duty_c_shadow <= 16'd0;

            threshold_a_shadow <= 12'd0;
            threshold_b_shadow <= 12'd0;
            threshold_c_shadow <= 12'd0;

            threshold_a_active <= 12'd0;
            threshold_b_active <= 12'd0;
            threshold_c_active <= 12'd0;

            shadow_pending <= 1'b0;
        end else if (duty_valid) begin
            duty_a_shadow <= duty_a;
            duty_b_shadow <= duty_b;
            duty_c_shadow <= duty_c;

            threshold_a_shadow <= duty_to_threshold(duty_a);
            threshold_b_shadow <= duty_to_threshold(duty_b);
            threshold_c_shadow <= duty_to_threshold(duty_c);

            shadow_pending <= 1'b1;
        end else if (carrier_count == 12'd0 && count_up &&
                     shadow_pending) begin
            threshold_a_active <= threshold_a_shadow;
            threshold_b_active <= threshold_b_shadow;
            threshold_c_active <= threshold_c_shadow;

            shadow_pending <= 1'b0;
        end
    end

    // Exact 5000-clock triangular carrier:
    // valley 0 -> ... -> peak 2500 -> ... -> valley 0.
    // The endpoints are not held for an extra clock.
    always_ff @(posedge clk) begin
        if (rst) begin
            carrier_count <= 12'd0;
            count_up      <= 1'b1;
            carrier_sync  <= 1'b0;
        end else begin
            carrier_sync <= 1'b0;

            if (count_up) begin
                if (carrier_count == HALF_PERIOD) begin
                    count_up      <= 1'b0;
                    carrier_count <= HALF_PERIOD - 12'd1;
                end else begin
                    carrier_count <= carrier_count + 12'd1;
                end
            end else begin
                if (carrier_count == 12'd1) begin
                    carrier_count <= 12'd0;
                    count_up      <= 1'b1;
                    carrier_sync  <= 1'b1;
                end else begin
                    carrier_count <= carrier_count - 12'd1;
                end
            end
        end
    end

    // Center-aligned raw PWM outputs.
    // Special cases guarantee exact 0% and 100% endpoints.
    always_comb begin
        if (rst || fault) begin
            pwm_a = 1'b0;
            pwm_b = 1'b0;
            pwm_c = 1'b0;
        end else begin
            pwm_a = (threshold_a_active == 12'd2500) ? 1'b1 :
                    (threshold_a_active == 12'd0)    ? 1'b0 :
                    (carrier_count < threshold_a_active);

            pwm_b = (threshold_b_active == 12'd2500) ? 1'b1 :
                    (threshold_b_active == 12'd0)    ? 1'b0 :
                    (carrier_count < threshold_b_active);

            pwm_c = (threshold_c_active == 12'd2500) ? 1'b1 :
                    (threshold_c_active == 12'd0)    ? 1'b0 :
                    (carrier_count < threshold_c_active);
        end
    end

endmodule
