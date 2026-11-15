`timescale 1ns/1ps

module shoot_through (
    input logic clk,
    input logic rst,
    input logic fault,
    input logic pwm_a,
    input logic pwm_b,
    input logic pwm_c
);

    logic gate_ah, gate_al;
    logic gate_bh, gate_bl;
    logic gate_ch, gate_cl;

    dead_time #(.DEAD_CYCLES(2)) dut (
        .clk    (clk),
        .rst    (rst),
        .fault  (fault),
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

    always_comb begin
        // Safety Property 1: Zero Shoot-Through on Phase A
        assert (!(gate_ah && gate_al));

        // Safety Property 2: Zero Shoot-Through on Phase B
        assert (!(gate_bh && gate_bl));

        // Safety Property 3: Zero Shoot-Through on Phase C
        assert (!(gate_ch && gate_cl));

        // Safety Property 4: Safe inhibit when fault or reset
        if (rst || fault) begin
            assert (!gate_ah && !gate_al && !gate_bh && !gate_bl && !gate_ch && !gate_cl);
        end
    end

endmodule
