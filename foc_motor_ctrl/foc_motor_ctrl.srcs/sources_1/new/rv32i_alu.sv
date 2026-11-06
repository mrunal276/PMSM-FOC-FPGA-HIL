`timescale 1ns/1ps

module rv32i_alu (
    input  logic [3:0]  alu_op,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] result,
    output logic        zero
);

    logic [4:0] shamt;
    assign shamt = b[4:0];

    always_comb begin
        case (alu_op)
            4'b0000: result = a + b;
            4'b0001: result = a - b;
            4'b0010: result = a << shamt;
            4'b0011: result = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            4'b0100: result = (a < b) ? 32'd1 : 32'd0;
            4'b0101: result = a ^ b;
            4'b0110: result = a >> shamt;
            4'b0111: result = $signed(a) >>> shamt;
            4'b1000: result = a | b;
            4'b1001: result = a & b;
            4'b1010: result = b; // PASS_B (for LUI)
            default: result = 32'd0;
        endcase
    end

    assign zero = (result == 32'd0);

endmodule
