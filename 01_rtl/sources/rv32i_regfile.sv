`timescale 1ns/1ps

module rv32i_regfile (
    input  logic        clk,
    input  logic        rst,
    // Read ports
    input  logic [4:0]  rs1_addr,
    input  logic [4:0]  rs2_addr,
    output logic [31:0] rs1_data,
    output logic [31:0] rs2_data,
    // Write port
    input  logic        wr_en,
    input  logic [4:0]  wr_addr,
    input  logic [31:0] wr_data
);

    logic [31:0] regs [1:31];

    // Async read with write-through forwarding
    assign rs1_data = (rs1_addr == 5'd0) ? 32'd0 :
                      (wr_en && rs1_addr == wr_addr) ? wr_data :
                      regs[rs1_addr];

    assign rs2_data = (rs2_addr == 5'd0) ? 32'd0 :
                      (wr_en && rs2_addr == wr_addr) ? wr_data :
                      regs[rs2_addr];

    // Sync write, x0 hardwired to 0
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 1; i < 32; i++) regs[i] <= 32'd0;
        end else if (wr_en && wr_addr != 5'd0) begin
            regs[wr_addr] <= wr_data;
        end
    end

endmodule
