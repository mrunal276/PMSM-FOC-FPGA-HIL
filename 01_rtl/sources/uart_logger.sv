`timescale 1ns/1ps

module uart_logger #(
    parameter int CLKS_PER_BIT = 868 // 100 MHz / 115200 baud
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        carrier_sync,
    input  logic signed [15:0] id_q15,
    input  logic signed [15:0] iq_q15,
    input  logic        [15:0] omega_q6,
    output logic        uart_tx,
    output logic        busy
);

    logic [31:0] timestamp;
    logic [79:0] tx_buffer;
    logic [3:0]  byte_idx;      // 0..9
    logic [3:0]  bit_idx;       // 0..9 (0=start, 1..8=data, 9=stop)
    logic [15:0] baud_cnt;
    logic        active;

    assign busy = active;

    // Current byte to transmit
    logic [7:0] current_byte;
    always_comb begin
        case (byte_idx)
            4'd0: current_byte = tx_buffer[7:0];
            4'd1: current_byte = tx_buffer[15:8];
            4'd2: current_byte = tx_buffer[23:16];
            4'd3: current_byte = tx_buffer[31:24];
            4'd4: current_byte = tx_buffer[39:32];
            4'd5: current_byte = tx_buffer[47:40];
            4'd6: current_byte = tx_buffer[55:48];
            4'd7: current_byte = tx_buffer[63:56];
            4'd8: current_byte = tx_buffer[71:64];
            4'd9: current_byte = tx_buffer[79:72];
            default: current_byte = 8'h00;
        endcase
    end

    // Transmit bit generation
    always_comb begin
        if (!active) begin
            uart_tx = 1'b1; // Idle high
        end else if (bit_idx == 4'd0) begin
            uart_tx = 1'b0; // Start bit
        end else if (bit_idx == 4'd9) begin
            uart_tx = 1'b1; // Stop bit
        end else begin
            uart_tx = current_byte[bit_idx - 1]; // Data bit LSB first
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            timestamp <= 32'd0;
            tx_buffer <= 80'd0;
            byte_idx  <= 4'd0;
            bit_idx   <= 4'd0;
            baud_cnt  <= 16'd0;
            active    <= 1'b0;
        end else begin
            // Increment timestamp counter on each carrier_sync pulse
            if (carrier_sync) begin
                timestamp <= timestamp + 32'd1;
            end

            if (!active) begin
                if (carrier_sync) begin
                    active    <= 1'b1;
                    tx_buffer <= {timestamp, omega_q6, iq_q15, id_q15};
                    byte_idx  <= 4'd0;
                    bit_idx   <= 4'd0;
                    baud_cnt  <= 16'd0;
                end
            end else begin
                if (baud_cnt >= CLKS_PER_BIT - 1) begin
                    baud_cnt <= 16'd0;
                    if (bit_idx == 4'd9) begin
                        bit_idx <= 4'd0;
                        if (byte_idx == 4'd9) begin
                            active <= 1'b0; // Complete frame sent
                        end else begin
                            byte_idx <= byte_idx + 4'd1;
                        end
                    end else begin
                        bit_idx <= bit_idx + 4'd1;
                    end
                end else begin
                    baud_cnt <= baud_cnt + 16'd1;
                end
            end
        end
    end

endmodule
