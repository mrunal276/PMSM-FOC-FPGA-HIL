`timescale 1ns/1ps

module rv32i_core #(
    parameter int IMEM_WORDS = 256
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        ext_irq,

    // APB Master interface
    output logic        apb_psel,
    output logic        apb_penable,
    output logic        apb_pwrite,
    output logic [31:0] apb_paddr,
    output logic [31:0] apb_pwdata,
    input  logic [31:0] apb_prdata,
    input  logic        apb_pready,

    // Debug / Telemetry
    output logic [31:0] dbg_pc,
    output logic [31:0] dbg_x1,
    output logic [31:0] dbg_x2,
    output logic [31:0] dbg_x10
);

    // ------------------------------------------------------------------------
    // Instruction Memory (1 KB ROM)
    // ------------------------------------------------------------------------
    logic [31:0] imem [0:IMEM_WORDS-1];
    initial begin
        for (int i = 0; i < IMEM_WORDS; i++) imem[i] = 32'h0000_0013; // NOP
        imem[0] = 32'h4000_00B7; // LUI x1, 0x40000  (x1 = 0x4000_0000)
        imem[1] = 32'h000A_0137; // LUI x2, 0x000A0  (x2 = 0x000A_0000)
        imem[2] = 32'h0020_A023; // SW x2, 0(x1)     (slv[0] <= x2)
        imem[3] = 32'h0000_A503; // LW x10, 0(x1)    (x10 <= slv[0])
        imem[4] = 32'h0000_A223; // SW x0, 4(x1)     (slv[1] <= 0)
        imem[5] = 32'h0080_A583; // LW x11, 8(x1)    (x11 <= slv[2])
        imem[6] = 32'h0000_006F; // JAL x0, 0        (infinite loop)
    end

    // ------------------------------------------------------------------------
    // Program Counter & Pipeline Registers
    // ------------------------------------------------------------------------
    logic [31:0] pc;
    assign dbg_pc = pc;

    logic [31:0] if_instr;
    assign if_instr = imem[pc[9:2]];

    // Register File
    logic [4:0]  rf_rs1_addr, rf_rs2_addr, rf_wr_addr;
    logic [31:0] rf_rs1_data, rf_rs2_data, rf_wr_data;
    logic        rf_wr_en;

    rv32i_regfile u_rf (
        .clk     (clk),
        .rst     (rst),
        .rs1_addr(rf_rs1_addr),
        .rs2_addr(rf_rs2_addr),
        .rs1_data(rf_rs1_data),
        .rs2_data(rf_rs2_data),
        .wr_en   (rf_wr_en),
        .wr_addr (rf_wr_addr),
        .wr_data (rf_wr_data)
    );

    // Read ports directly connected to fetched instruction
    assign rf_rs1_addr = if_instr[19:15];
    assign rf_rs2_addr = if_instr[24:20];

    // Debug taps from regfile
    assign dbg_x1  = (rf_wr_en && rf_wr_addr == 5'd1)  ? rf_wr_data : u_rf.regs[1];
    assign dbg_x2  = (rf_wr_en && rf_wr_addr == 5'd2)  ? rf_wr_data : u_rf.regs[2];
    assign dbg_x10 = (rf_wr_en && rf_wr_addr == 5'd10) ? rf_wr_data : u_rf.regs[10];

    // Decoder
    logic [4:0]  dec_rs1, dec_rs2, dec_rd;
    logic [31:0] dec_imm;
    logic [3:0]  dec_alu_op;
    logic        dec_alu_src, dec_mem_read, dec_mem_write, dec_reg_write;
    logic        dec_branch, dec_jal, dec_jalr, dec_lui, dec_auipc, dec_sys, dec_ill;

    rv32i_decode u_dec (
        .instr    (if_instr),
        .rs1      (dec_rs1),
        .rs2      (dec_rs2),
        .rd       (dec_rd),
        .imm      (dec_imm),
        .alu_op   (dec_alu_op),
        .alu_src  (dec_alu_src),
        .mem_read (dec_mem_read),
        .mem_write(dec_mem_write),
        .reg_write(dec_reg_write),
        .branch   (dec_branch),
        .jal      (dec_jal),
        .jalr     (dec_jalr),
        .lui      (dec_lui),
        .auipc    (dec_auipc),
        .is_system(dec_sys),
        .is_illegal(dec_ill)
    );

    // ALU
    logic [31:0] alu_in_a, alu_in_b, alu_out;
    logic        alu_zero;

    assign alu_in_a = (dec_auipc) ? pc : rf_rs1_data;
    assign alu_in_b = (dec_alu_src) ? dec_imm : rf_rs2_data;

    rv32i_alu u_alu (
        .alu_op(dec_alu_op),
        .a     (alu_in_a),
        .b     (alu_in_b),
        .result(alu_out),
        .zero  (alu_zero)
    );

    // ------------------------------------------------------------------------
    // APB Master FSM & Execution Controller
    // ------------------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_EXEC,
        ST_APB_SETUP,
        ST_APB_ACCESS
    } cpu_state_t;

    cpu_state_t state;

    always_ff @(posedge clk) begin
        if (rst) begin
            pc          <= 32'd0;
            state       <= ST_EXEC;
            apb_psel    <= 1'b0;
            apb_penable <= 1'b0;
            apb_pwrite  <= 1'b0;
            apb_paddr   <= 32'd0;
            apb_pwdata  <= 32'd0;
            rf_wr_en    <= 1'b0;
            rf_wr_addr  <= 5'd0;
            rf_wr_data  <= 32'd0;
        end else begin
            case (state)
                ST_EXEC: begin
                    rf_wr_en <= 1'b0;

                    if (ext_irq) begin
                        pc <= 32'h0000_0100; // Trap vector
                    end else if (dec_mem_read || dec_mem_write) begin
                        state       <= ST_APB_SETUP;
                        apb_psel    <= 1'b1;
                        apb_penable <= 1'b0;
                        apb_pwrite  <= dec_mem_write;
                        apb_paddr   <= alu_out;
                        apb_pwdata  <= rf_rs2_data;
                    end else begin
                        if (dec_reg_write && dec_rd != 5'd0) begin
                            rf_wr_en   <= 1'b1;
                            rf_wr_addr <= dec_rd;
                            if (dec_jal || dec_jalr) begin
                                rf_wr_data <= pc + 32'd4;
                            end else begin
                                rf_wr_data <= alu_out;
                            end
                        end

                        if (dec_jal) begin
                            pc <= pc + dec_imm;
                        end else if (dec_jalr) begin
                            pc <= (rf_rs1_data + dec_imm) & ~32'd1;
                        end else if (dec_branch) begin
                            case (if_instr[14:12])
                                3'b000: pc <= (alu_zero)  ? (pc + dec_imm) : (pc + 32'd4);
                                3'b001: pc <= (!alu_zero) ? (pc + dec_imm) : (pc + 32'd4);
                                3'b100: pc <= (alu_out == 32'd1) ? (pc + dec_imm) : (pc + 32'd4);
                                3'b101: pc <= (alu_out == 32'd0) ? (pc + dec_imm) : (pc + 32'd4);
                                default: pc <= pc + 32'd4;
                            endcase
                        end else begin
                            pc <= pc + 32'd4;
                        end
                    end
                end

                ST_APB_SETUP: begin
                    state       <= ST_APB_ACCESS;
                    apb_penable <= 1'b1;
                end

                ST_APB_ACCESS: begin
                    if (apb_pready) begin
                        apb_psel    <= 1'b0;
                        apb_penable <= 1'b0;
                        state       <= ST_EXEC;
                        pc          <= pc + 32'd4;

                        if (dec_mem_read && dec_rd != 5'd0) begin
                            rf_wr_en   <= 1'b1;
                            rf_wr_addr <= dec_rd;
                            rf_wr_data <= apb_prdata;
                        end
                    end
                end

                default: state <= ST_EXEC;
            endcase
        end
    end

endmodule
