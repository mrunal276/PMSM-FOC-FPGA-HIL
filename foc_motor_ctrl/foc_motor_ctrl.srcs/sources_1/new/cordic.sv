module cordic (
    input  logic               clk,
    input  logic               rst,
    input  logic               theta_valid,
    input  logic [15:0]        theta_in,

    output logic               result_valid,
    output logic signed [15:0] sin_out,
    output logic signed [15:0] cos_out
);

    // ============================================================
    // Fixed-point configuration
    //
    // External angle:
    //   unsigned Q2.14
    //
    // Internal CORDIC:
    //   X/Y : Q20
    //   Z   : Q22
    // ============================================================

    localparam int XY_WIDTH   = 24;
    localparam int Z_WIDTH    = 24;
    localparam int STAGES     = 16;
    localparam int ANGLE_FRAC = 22;

    // CORDIC gain compensation:
    //
    // 0.607252935103139 * 2^20 = 636751
    //
    localparam logic signed [XY_WIDTH-1:0] CORDIC_K =
        24'sd636751;


    // ============================================================
    // atan(2^-i) table in Q22 angle format
    //
    // π/2 = 8192 external Q2.14
    // π/2 = 8192 << 8 = 2097152 in Q22
    // ============================================================

    localparam logic signed [Z_WIDTH-1:0] ATAN_TABLE [0:15] = '{
        24'sd1048576,
        24'sd619011,
        24'sd327068,
        24'sd166025,
        24'sd83335,
        24'sd41708,
        24'sd20859,
        24'sd10430,
        24'sd5215,
        24'sd2608,
        24'sd1304,
        24'sd652,
        24'sd326,
        24'sd163,
        24'sd81,
        24'sd41
    };


    // ============================================================
    // Pipeline registers
    // ============================================================

    logic signed [XY_WIDTH-1:0] x_pipe [0:STAGES];
    logic signed [XY_WIDTH-1:0] y_pipe [0:STAGES];

    logic signed [Z_WIDTH-1:0]  z_pipe [0:STAGES];

    logic                       valid_pipe [0:STAGES];
    logic [1:0]                 quadrant_pipe [0:STAGES];


    // ============================================================
    // Angle reduction
    //
    // External angle convention:
    //
    //   0x0000 =   0°
    //   0x2000 =  90°
    //   0x4000 = 180°
    //   0x6000 = 270°
    //   0x8000 = 360° = 0°
    //
    // reduced_angle remains in external Q2.14.
    // It is converted to Q22 when loaded into z_pipe[0].
    // ============================================================

    logic [1:0] quadrant;
    logic [15:0] reduced_angle_q214;

    always_comb begin

        if (theta_in == 16'h8000) begin

            // 360° == 0°
            quadrant        = 2'd0;
            reduced_angle_q214 = 16'h0000;

        end else if (theta_in < 16'h2000) begin

            // Q0: 0° ... <90°
            quadrant        = 2'd0;
            reduced_angle_q214 = theta_in;

        end else if (theta_in < 16'h4000) begin

            // Q1: 90° ... <180°
            //
            // reduced = 180° - theta
            quadrant        = 2'd1;
            reduced_angle_q214 = 16'h4000 - theta_in;

        end else if (theta_in < 16'h6000) begin

            // Q2: 180° ... <270°
            //
            // reduced = theta - 180°
            quadrant        = 2'd2;
            reduced_angle_q214 = theta_in - 16'h4000;

        end else begin

            // Q3: 270° ... <360°
            //
            // reduced = 360° - theta
            quadrant        = 2'd3;
            reduced_angle_q214 = 16'h8000 - theta_in;

        end

    end


    // ============================================================
    // Q20 -> Q15 conversion
    //
    // Add 16 before shifting right by 5:
    //
    //   Q20 -> Q15
    //   2^20 / 2^15 = 32
    //
    // +16 implements round-to-nearest for positive values.
    //
    // Saturate +1.0 because +32768 is not representable
    // in signed Q1.15.
    // ============================================================

    function automatic signed [15:0] q20_to_q15_sat (
        input logic signed [XY_WIDTH-1:0] value
    );

        logic signed [XY_WIDTH-1:0] rounded;
        logic signed [XY_WIDTH-1:0] shifted;

        begin

            rounded = value + 24'sd16;
            shifted = rounded >>> 5;

            if (shifted > 24'sd32767) begin
                q20_to_q15_sat = 16'sh7fff;

            end else if (shifted < -24'sd32768) begin
                q20_to_q15_sat = 16'sh8000;

            end else begin
                q20_to_q15_sat = shifted[15:0];
            end

        end

    endfunction


    // ============================================================
    // CORDIC pipeline
    // ============================================================

    integer i;

    always_ff @(posedge clk) begin

        if (rst) begin

            for (i = 0; i <= STAGES; i = i + 1) begin

                x_pipe[i]        <= '0;
                y_pipe[i]        <= '0;
                z_pipe[i]        <= '0;

                valid_pipe[i]    <= 1'b0;
                quadrant_pipe[i] <= 2'd0;

            end

            result_valid <= 1'b0;
            sin_out      <= 16'sd0;
            cos_out      <= 16'sd0;

        end else begin

            // ----------------------------------------------------
            // Input / stage 0
            // ----------------------------------------------------

            x_pipe[0] <= CORDIC_K;
            y_pipe[0] <= 24'sd0;

            // Convert external Q2.14 angle to internal Q22:
            //
            // Q22 = Q14 << 8
            //
            z_pipe[0] <=
                $signed({8'd0, reduced_angle_q214}) <<< 8;

            valid_pipe[0]    <= theta_valid;
            quadrant_pipe[0] <= quadrant;


            // ----------------------------------------------------
            // CORDIC iterations
            // ----------------------------------------------------

            for (i = 0; i < STAGES; i = i + 1) begin

                valid_pipe[i+1]    <= valid_pipe[i];
                quadrant_pipe[i+1] <= quadrant_pipe[i];

                if (z_pipe[i] >= 0) begin

                    x_pipe[i+1] <=
                        x_pipe[i] - (y_pipe[i] >>> i);

                    y_pipe[i+1] <=
                        y_pipe[i] + (x_pipe[i] >>> i);

                    z_pipe[i+1] <=
                        z_pipe[i] - ATAN_TABLE[i];

                end else begin

                    x_pipe[i+1] <=
                        x_pipe[i] + (y_pipe[i] >>> i);

                    y_pipe[i+1] <=
                        y_pipe[i] - (x_pipe[i] >>> i);

                    z_pipe[i+1] <=
                        z_pipe[i] + ATAN_TABLE[i];

                end

            end


            // ----------------------------------------------------
            // Output valid
            // ----------------------------------------------------

            result_valid <= valid_pipe[STAGES];


            // ----------------------------------------------------
            // Quadrant reconstruction
            //
            // Q0: cos =  X, sin =  Y
            // Q1: cos = -X, sin =  Y
            // Q2: cos = -X, sin = -Y
            // Q3: cos =  X, sin = -Y
            // ----------------------------------------------------

            case (quadrant_pipe[STAGES])

                2'd0: begin

                    cos_out <=
                        q20_to_q15_sat(x_pipe[STAGES]);

                    sin_out <=
                        q20_to_q15_sat(y_pipe[STAGES]);

                end


                2'd1: begin

                    cos_out <=
                        -q20_to_q15_sat(x_pipe[STAGES]);

                    sin_out <=
                        q20_to_q15_sat(y_pipe[STAGES]);

                end


                2'd2: begin

                    cos_out <=
                        -q20_to_q15_sat(x_pipe[STAGES]);

                    sin_out <=
                        -q20_to_q15_sat(y_pipe[STAGES]);

                end


                2'd3: begin

                    cos_out <=
                        q20_to_q15_sat(x_pipe[STAGES]);

                    sin_out <=
                        -q20_to_q15_sat(y_pipe[STAGES]);

                end


                default: begin

                    cos_out <= 16'sd0;
                    sin_out <= 16'sd0;

                end

            endcase

        end

    end

endmodule