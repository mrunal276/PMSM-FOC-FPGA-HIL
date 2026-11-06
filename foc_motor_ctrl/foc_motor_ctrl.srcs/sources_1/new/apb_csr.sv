`timescale 1ns/1ps

module apb_csr (
    input  logic        clk,
    input  logic        rst,

    // APB Slave interface
    input  logic        apb_psel,
    input  logic        apb_penable,
    input  logic        apb_pwrite,
    input  logic [31:0] apb_paddr,
    input  logic [31:0] apb_pwdata,
    output logic [31:0] apb_prdata,
    output logic        apb_pready,

    // Telemetry inputs
    input  logic [15:0] omega_meas,
    input  logic signed [15:0] id_meas,
    input  logic signed [15:0] iq_meas,
    input  logic [2:0]  fault_code,
    input  logic [2:0]  foc_state,
    input  logic        locked,

    // Control outputs
    output logic [15:0] speed_ref,
    output logic signed [15:0] id_ref,
    output logic        fault_clear,
    output logic [15:0] kp_curr,
    output logic [15:0] ki_curr
);

    // Address decoding (using byte address bits [7:2])
    localparam logic [5:0] ADDR_SPEED_REF   = 6'h00; // 0x00
    localparam logic [5:0] ADDR_ID_REF      = 6'h01; // 0x04
    localparam logic [5:0] ADDR_STATUS      = 6'h02; // 0x08
    localparam logic [5:0] ADDR_OMEGA_MEAS  = 6'h03; // 0x0C
    localparam logic [5:0] ADDR_ID_MEAS     = 6'h04; // 0x10
    localparam logic [5:0] ADDR_IQ_MEAS     = 6'h05; // 0x14
    localparam logic [5:0] ADDR_FAULT_CLEAR = 6'h06; // 0x18
    localparam logic [5:0] ADDR_KP_CURR     = 6'h07; // 0x1C
    localparam logic [5:0] ADDR_KI_CURR     = 6'h08; // 0x20

    logic [5:0] word_addr;
    assign word_addr = apb_paddr[7:2];

    // Single-cycle APB slave
    assign apb_pready = apb_psel && apb_penable;

    // Registers
    logic [15:0] reg_speed_ref;
    logic signed [15:0] reg_id_ref;
    logic [15:0] reg_kp_curr;
    logic [15:0] reg_ki_curr;

    assign speed_ref = reg_speed_ref;
    assign id_ref    = reg_id_ref;
    assign kp_curr   = reg_kp_curr;
    assign ki_curr   = reg_ki_curr;

    // Read logic
    always_comb begin
        case (word_addr)
            ADDR_SPEED_REF:   apb_prdata = {16'd0, reg_speed_ref};
            ADDR_ID_REF:      apb_prdata = {{16{reg_id_ref[15]}}, reg_id_ref};
            ADDR_STATUS:      apb_prdata = {25'd0, fault_code, foc_state, locked};
            ADDR_OMEGA_MEAS:  apb_prdata = {16'd0, omega_meas};
            ADDR_ID_MEAS:     apb_prdata = {{16{id_meas[15]}}, id_meas};
            ADDR_IQ_MEAS:     apb_prdata = {{16{iq_meas[15]}}, iq_meas};
            ADDR_FAULT_CLEAR: apb_prdata = 32'd0;
            ADDR_KP_CURR:     apb_prdata = {16'd0, reg_kp_curr};
            ADDR_KI_CURR:     apb_prdata = {16'd0, reg_ki_curr};
            default:          apb_prdata = 32'd0;
        endcase
    end

    // Write logic
    always_ff @(posedge clk) begin
        if (rst) begin
            reg_speed_ref <= 16'd0;
            reg_id_ref    <= 16'sd0;
            reg_kp_curr   <= 16'd12288;
            reg_ki_curr   <= 16'd819;
            fault_clear   <= 1'b0;
        end else begin
            fault_clear <= 1'b0;
            if (apb_psel && apb_penable && apb_pwrite) begin
                case (word_addr)
                    ADDR_SPEED_REF:   reg_speed_ref <= apb_pwdata[15:0];
                    ADDR_ID_REF:      reg_id_ref    <= apb_pwdata[15:0];
                    ADDR_FAULT_CLEAR: fault_clear   <= apb_pwdata[0];
                    ADDR_KP_CURR:     reg_kp_curr   <= apb_pwdata[15:0];
                    ADDR_KI_CURR:     reg_ki_curr   <= apb_pwdata[15:0];
                    default: ;
                endcase
            end
        end
    end

endmodule
