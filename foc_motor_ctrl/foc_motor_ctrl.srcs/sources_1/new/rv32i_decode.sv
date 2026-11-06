`timescale 1ns/1ps

module rv32i_decode (
    input  logic [31:0] instr,
    output logic [4:0]  rs1, rs2, rd,
    output logic [31:0] imm,
    output logic [3:0]  alu_op,
    output logic        alu_src,      // 0=rs2, 1=imm
    output logic        mem_read,     // load
    output logic        mem_write,    // store
    output logic        reg_write,
    output logic        branch,
    output logic        jal,
    output logic        jalr,
    output logic        lui,
    output logic        auipc,
    output logic        is_system,
    output logic        is_illegal
);

    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;

    assign opcode = instr[6:0];
    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];
    assign rs1    = instr[19:15];
    assign rs2    = instr[24:20];
    assign rd     = instr[11:7];

    // Immediate generation
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;
    assign imm_i = {{20{instr[31]}}, instr[31:20]};
    assign imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
    assign imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    assign imm_u = {instr[31:12], 12'd0};
    assign imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    always_comb begin
        imm       = 32'd0;
        alu_op    = 4'b0000;
        alu_src   = 1'b0;
        mem_read  = 1'b0;
        mem_write = 1'b0;
        reg_write = 1'b0;
        branch    = 1'b0;
        jal       = 1'b0;
        jalr      = 1'b0;
        lui       = 1'b0;
        auipc     = 1'b0;
        is_system = 1'b0;
        is_illegal= 1'b0;

        case (opcode)
            7'b0110011: begin // R-type
                reg_write = 1'b1;
                alu_src   = 1'b0;
                imm       = 32'd0;
                case ({funct7[5], funct3})
                    4'b0000: alu_op = 4'b0000; // ADD
                    4'b1000: alu_op = 4'b0001; // SUB
                    4'b0001: alu_op = 4'b0010; // SLL
                    4'b0010: alu_op = 4'b0011; // SLT
                    4'b0011: alu_op = 4'b0100; // SLTU
                    4'b0100: alu_op = 4'b0101; // XOR
                    4'b0101: alu_op = 4'b0110; // SRL
                    4'b1101: alu_op = 4'b0111; // SRA
                    4'b0110: alu_op = 4'b1000; // OR
                    4'b0111: alu_op = 4'b1001; // AND
                    default: is_illegal = 1'b1;
                endcase
            end

            7'b0010011: begin // I-type ALU
                reg_write = 1'b1;
                alu_src   = 1'b1;
                imm       = imm_i;
                case (funct3)
                    3'b000: alu_op = 4'b0000; // ADDI
                    3'b010: alu_op = 4'b0011; // SLTI
                    3'b011: alu_op = 4'b0100; // SLTIU
                    3'b100: alu_op = 4'b0101; // XORI
                    3'b110: alu_op = 4'b1000; // ORI
                    3'b111: alu_op = 4'b1001; // ANDI
                    3'b001: alu_op = 4'b0010; // SLLI
                    3'b101: alu_op = (funct7[5]) ? 4'b0111 : 4'b0110; // SRAI/SRLI
                    default: is_illegal = 1'b1;
                endcase
            end

            7'b0000011: begin // LOAD (LW)
                reg_write = 1'b1;
                mem_read  = 1'b1;
                alu_src   = 1'b1;
                alu_op    = 4'b0000;
                imm       = imm_i;
            end

            7'b0100011: begin // STORE (SW)
                mem_write = 1'b1;
                alu_src   = 1'b1;
                alu_op    = 4'b0000;
                imm       = imm_s;
            end

            7'b1100011: begin // BRANCH
                branch    = 1'b1;
                alu_src   = 1'b0;
                imm       = imm_b;
                case (funct3)
                    3'b000: alu_op = 4'b0001; // BEQ
                    3'b001: alu_op = 4'b0001; // BNE
                    3'b100: alu_op = 4'b0011; // BLT
                    3'b101: alu_op = 4'b0011; // BGE
                    3'b110: alu_op = 4'b0100; // BLTU
                    3'b111: alu_op = 4'b0100; // BGEU
                    default: is_illegal = 1'b1;
                endcase
            end

            7'b1101111: begin // JAL
                jal       = 1'b1;
                reg_write = 1'b1;
                imm       = imm_j;
                alu_op    = 4'b0000;
            end

            7'b1100111: begin // JALR
                jalr      = 1'b1;
                reg_write = 1'b1;
                alu_src   = 1'b1;
                alu_op    = 4'b0000;
                imm       = imm_i;
            end

            7'b0110111: begin // LUI
                lui       = 1'b1;
                reg_write = 1'b1;
                alu_op    = 4'b1010; // PASS_B
                alu_src   = 1'b1;
                imm       = imm_u;
            end

            7'b0010111: begin // AUIPC
                auipc     = 1'b1;
                reg_write = 1'b1;
                alu_op    = 4'b0000;
                alu_src   = 1'b1;
                imm       = imm_u;
            end

            7'b1110011: begin // SYSTEM
                is_system = 1'b1;
                if (funct3 != 3'b000) reg_write = 1'b1;
            end

            default: is_illegal = 1'b1;
        endcase
    end

endmodule
