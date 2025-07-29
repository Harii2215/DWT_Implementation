`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/04/2025 10:57:25 AM
// Design Name: 
// Module Name: dwt_haar_pipelined_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module dwt_pipeline_stage0_load #(
    parameter N = 8
)(
    input  wire                     clk,
    input  wire                     en,          // enable signal from controller
    input  wire [$clog2(N/2)-1:0]   pair_idx,    // index of the current input pair
    input  wire [N*16-1:0]          array_in,    // flattened input array

    output reg  [15:0]              x0_out,
    output reg  [15:0]              x1_out
);

    // Internal wires to extract data from flattened array
    wire [15:0] x0 = array_in[2*pair_idx*16 +: 16];
    wire [15:0] x1 = array_in[(2*pair_idx + 1)*16 +: 16];

    always @(posedge clk) begin
        if (en) begin
            x0_out <= x0;
            x1_out <= x1;
        end
    end

endmodule

`timescale 1ns / 1ps

module reg_stage0_1 (
    input  wire        clk,
    input  wire        rst,
    input  wire        valid_in,
    input  wire [15:0] x0_in,
    input  wire [15:0] x1_in,

    output reg         valid_out,
    output reg [15:0]  x0_out,
    output reg [15:0]  x1_out
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x0_out     <= 16'd0;
            x1_out     <= 16'd0;
            valid_out  <= 1'b0;
        end else begin
            x0_out     <= x0_in;
            x1_out     <= x1_in;
            valid_out  <= valid_in;
        end
    end

endmodule

`timescale 1ns / 1ps

module dwt_pipeline_stage1_mult (
    input  wire         clk,
    input  wire         valid_in,
    input  wire [15:0]  x0_in,
    input  wire [15:0]  x1_in,

    output reg  [31:0]  x0_mul_out,
    output reg  [31:0]  x1_mul_out
);

    wire [31:0] x0_mul, x1_mul;

    mult_by_181 mul_x0 (
        .in(x0_in),
        .result(x0_mul)
    );

    mult_by_181 mul_x1 (
        .in(x1_in),
        .result(x1_mul)
    );

    always @(posedge clk) begin
        if (valid_in) begin
            x0_mul_out <= x0_mul;
            x1_mul_out <= x1_mul;
        end
    end

endmodule

`timescale 1ns / 1ps

module reg_stage1_2 (
    input  wire        clk,
    input  wire        rst,
    input  wire        valid_in,
    input  wire [31:0] x0_in,
    input  wire [31:0] x1_in,

    output reg         valid_out,
    output reg [31:0]  x0_out,
    output reg [31:0]  x1_out
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x0_out    <= 32'd0;
            x1_out    <= 32'd0;
            valid_out <= 1'b0;
        end else begin
            x0_out    <= x0_in;
            x1_out    <= x1_in;
            valid_out <= valid_in;
        end
    end

endmodule

`timescale 1ns / 1ps

module dwt_pipeline_stage2_core (
    input  wire        clk,
    input  wire        valid_in,
    input  wire [31:0] x0_mul,
    input  wire [31:0] x1_mul,

    output reg  [15:0] cA,
    output reg  [15:0] cD
);

    wire [31:0] sum, diff;
    wire [31:0] x1_neg, x1_twos;
    wire        dummy;

    // Sum: x0 + x1
    sklansky_adder_32bit adder_sum (
        .A(x0_mul), .B(x1_mul), .Cin(1'b0), .Sum(sum), .Cout(dummy)
    );

    // Two's complement of x1: ~x1 + 1
    assign x1_neg = ~x1_mul;
    sklansky_adder_32bit adder_twos (
        .A(x1_neg), .B(32'd1), .Cin(1'b0), .Sum(x1_twos), .Cout(dummy)
    );

    // Difference: x0 - x1 = x0 + (~x1 + 1)
    sklansky_adder_32bit adder_diff (
        .A(x0_mul), .B(x1_twos), .Cin(1'b0), .Sum(diff), .Cout(dummy)
    );

    // Register outputs on valid
    always @(posedge clk) begin
        if (valid_in) begin
            cA <= sum[23:8];  // Normalize from Q16.16 to Q8.8
            cD <= diff[23:8];
        end
    end

endmodule

`timescale 1ns / 1ps

module reg_stage2_3 (
    input  wire        clk,
    input  wire        rst,
    input  wire        valid_in,
    input  wire [15:0] cA_in,
    input  wire [15:0] cD_in,

    output reg         valid_out,
    output reg [15:0]  cA_out,
    output reg [15:0]  cD_out
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cA_out    <= 16'd0;
            cD_out    <= 16'd0;
            valid_out <= 1'b0;
        end else begin
            cA_out    <= cA_in;
            cD_out    <= cD_in;
            valid_out <= valid_in;
        end
    end

endmodule

`timescale 1ns / 1ps

module dwt_pipeline_stage3_store #(
    parameter N = 8  // Number of input pixels
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   valid_in,
    input  wire                   write_en,
    input  wire [$clog2(N/2)-1:0] pair_idx,

    input  wire [15:0] cA_in,
    input  wire [15:0] cD_in,

    output reg  [16*(N/2)-1:0] cA_out,
    output reg  [16*(N/2)-1:0] cD_out
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cA_out <= 0;
            cD_out <= 0;
        end else if (valid_in && write_en) begin
            cA_out[pair_idx*16 +: 16] <= cA_in;
            cD_out[pair_idx*16 +: 16] <= cD_in;
        end
    end

endmodule

module dwt_haar_pipelined_ctrl #
(
    parameter N = 8,              // Total number of input samples (must be even)
    parameter LATENCY = 3         // Latency of DWT datapath (not used in this FSM)
)
(
    input wire clk,
    input wire rst,
    input wire start,

    output reg [31:0] pair_idx,   // Index for both loading and writing
    output reg load_en,           // Enable signal for loading inputs
    output reg write_en,          // Enable signal for writing outputs
    output reg done               // Operation complete
);

    // State encoding: 5 states (IDLE, LOAD, FLUSH, WRITE, DONE)
    localparam IDLE  = 3'b000;
    localparam LOAD  = 3'b001;
    localparam FLUSH = 3'b010;  // Extra cycle to flush last valid
    localparam WRITE = 3'b011;
    localparam DONE  = 3'b100;

    reg [2:0] state, next_state;
    reg [31:0] load_count;
    reg [31:0] write_count;

    // Sequential state update.
    always @(posedge clk or posedge rst) begin
        if (rst)
            state <= IDLE;
        else
            state <= next_state;
    end

    // FSM transitions.
    always @(*) begin
        case (state)
            IDLE: begin
                next_state = (start) ? LOAD : IDLE;
            end

            // In LOAD state, we count up to N/2.
            LOAD: begin
                // When load_count equals N/2, we have loaded the last pair.
                // Instead of immediately going to WRITE, we insert a FLUSH cycle.
                next_state = (load_count == (N/2)) ? FLUSH : LOAD;
            end

            // FLUSH: One extra cycle to propagate the last valid.
            FLUSH: begin
                next_state = WRITE;
            end

            WRITE: begin
                // Process each pair in WRITE mode.
                next_state = (write_count == (N/2)) ? DONE : WRITE;
            end

            DONE: begin
                next_state = (!start) ? IDLE : DONE;
            end

            default: next_state = IDLE;
        endcase
    end

    // FSM Outputs and counters
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            load_count  <= 0;
            write_count <= 0;
            pair_idx    <= 0;
            load_en     <= 0;
            write_en    <= 0;
            done        <= 0;
        end else begin
            case (state)
                IDLE: begin
                    load_count  <= 0;
                    write_count <= 0;
                    pair_idx    <= 0;
                    load_en     <= 0;
                    write_en    <= 0;
                    done        <= 0;
                end

                LOAD: begin
                    load_en     <= 1;
                    write_en    <= 0;
                    pair_idx    <= load_count;
                    load_count  <= load_count + 1;
                    done        <= 0;
                end

                // In FLUSH state, hold load_en high (or at least do not force it to 0)
                // so that the last loaded valid flag is captured in the next pipeline register.
                FLUSH: begin
                    // Do not increment load_count; maintain the last pair index.
                    load_en  <= 1;
                    write_en <= 0;
                    pair_idx <= pair_idx; // hold the last index (N/2 - 1)
                    done     <= 0;
                end

                WRITE: begin
                    load_en     <= 0;
                    write_en    <= 1;
                    pair_idx    <= write_count;
                    write_count <= write_count + 1;
                    done        <= 0;
                end

                DONE: begin
                    load_en     <= 0;
                    write_en    <= 0;
                    pair_idx    <= 0;
                    done        <= 1;
                end

            endcase
        end
    end

endmodule

`timescale 1ns / 1ps

module dwt_haar_pipelined_top #(
    parameter N = 8
)(
    input  wire clk,
    input  wire rst,
    input  wire start,
    input  wire [N*16-1:0] array_in,

    output wire [16*(N/2)-1:0] cA_out,
    output wire [16*(N/2)-1:0] cD_out,
    output wire done
);

    // Internal control signals
    wire [$clog2(N/2)-1:0] pair_idx_pipe;
    wire load_en, valid_stage1, valid_stage2, valid_stage3, write_en;

    // Stage 0: Load input pair
    wire [15:0] x0_s0, x1_s0;
    dwt_pipeline_stage0_load #(.N(N)) stage0 (
        .clk(clk),
        .en(load_en),
        .pair_idx(pair_idx_pipe),
        .array_in(array_in),
        .x0_out(x0_s0),
        .x1_out(x1_s0)
    );

    // Stage 0?1 pipeline registers
    wire [15:0] x0_s1, x1_s1;
    reg_stage0_1 reg01 (
        .clk(clk),
        .rst(rst),
        .valid_in(load_en),
        .x0_in(x0_s0),
        .x1_in(x1_s0),
        .valid_out(valid_stage1),
        .x0_out(x0_s1),
        .x1_out(x1_s1)
    );

    // Stage 1: Multiply by 181
    wire [31:0] x0_mul_s1, x1_mul_s1;
    dwt_pipeline_stage1_mult stage1 (
        .clk(clk),
        .valid_in(valid_stage1),
        .x0_in(x0_s1),
        .x1_in(x1_s1),
        .x0_mul_out(x0_mul_s1),
        .x1_mul_out(x1_mul_s1)
    );

    // Stage 1?2 pipeline registers
    wire [31:0] x0_mul_s2, x1_mul_s2;
    reg_stage1_2 reg12 (
        .clk(clk),
        .rst(rst),
        .valid_in(valid_stage1),
        .x0_in(x0_mul_s1),
        .x1_in(x1_mul_s1),
        .valid_out(valid_stage2),
        .x0_out(x0_mul_s2),
        .x1_out(x1_mul_s2)
    );

    // Stage 2: Core DWT sum/diff and right shift
    wire [15:0] cA_s2, cD_s2;
    dwt_pipeline_stage2_core stage2 (
        .clk(clk),
        .valid_in(valid_stage2),
        .x0_mul(x0_mul_s2),
        .x1_mul(x1_mul_s2),
        .cA(cA_s2),
        .cD(cD_s2)
    );

    // Stage 2?3 pipeline registers
    wire [15:0] cA_s3, cD_s3;
    reg_stage2_3 reg23 (
        .clk(clk),
        .rst(rst),
        .valid_in(valid_stage2),
        .cA_in(cA_s2),
        .cD_in(cD_s2),
        .valid_out(valid_stage3),
        .cA_out(cA_s3),
        .cD_out(cD_s3)
    );

    // Stage 3: Output write
    dwt_pipeline_stage3_store #(.N(N)) stage3 (
        .clk(clk),
        .rst(rst),
        .valid_in(valid_stage3),
        .write_en(write_en),
        .pair_idx(pair_idx_pipe),
        .cA_in(cA_s3),
        .cD_in(cD_s3),
        .cA_out(cA_out),
        .cD_out(cD_out)
    );

    // Controller FSM
    dwt_haar_pipelined_ctrl #(.N(N)) ctrl (
        .clk(clk),
        .rst(rst),
        .start(start),
        .pair_idx(pair_idx_pipe),
        .load_en(load_en),
        .write_en(write_en),
        .done(done)
    );

endmodule

module sklansky_adder_32bit (
    input  [31:0] A,
    input  [31:0] B,
    input         Cin,
    output [31:0] Sum,
    output        Cout
);

    wire [31:0] G, P;        // Generate and Propagate
    wire [31:0] C;           // Carry

    // Initial generate and propagate
    assign G = A & B;
    assign P = A ^ B;

    // Level-wise carry generation (Sklansky Prefix Tree)
    wire [31:0] G1, G2, G3, G4, G5;
    wire [31:0] P1, P2, P3, P4, P5;

    // Level 1: distance = 1
    assign G1[0] = G[0];
    assign P1[0] = P[0];
    genvar i;
    generate
        for (i = 1; i < 32; i = i + 1) begin
            assign G1[i] = G[i] | (P[i] & G[i - 1]);
            assign P1[i] = P[i] & P[i - 1];
        end
    endgenerate

    // Level 2: distance = 2
    generate
        for (i = 0; i < 2; i = i + 1) begin
            assign G2[i] = G1[i];
            assign P2[i] = P1[i];
        end
        for (i = 2; i < 32; i = i + 1) begin
            assign G2[i] = G1[i] | (P1[i] & G1[i - 2]);
            assign P2[i] = P1[i] & P1[i - 2];
        end
    endgenerate

    // Level 3: distance = 4
    generate
        for (i = 0; i < 4; i = i + 1) begin
            assign G3[i] = G2[i];
            assign P3[i] = P2[i];
        end
        for (i = 4; i < 32; i = i + 1) begin
            assign G3[i] = G2[i] | (P2[i] & G2[i - 4]);
            assign P3[i] = P2[i] & P2[i - 4];
        end
    endgenerate

    // Level 4: distance = 8
    generate
        for (i = 0; i < 8; i = i + 1) begin
            assign G4[i] = G3[i];
            assign P4[i] = P3[i];
        end
        for (i = 8; i < 32; i = i + 1) begin
            assign G4[i] = G3[i] | (P3[i] & G3[i - 8]);
            assign P4[i] = P3[i] & P3[i - 8];
        end
    endgenerate

    // Level 5: distance = 16
    generate
        for (i = 0; i < 16; i = i + 1) begin
            assign G5[i] = G4[i];
            assign P5[i] = P4[i];
        end
        for (i = 16; i < 32; i = i + 1) begin
            assign G5[i] = G4[i] | (P4[i] & G4[i - 16]);
            assign P5[i] = P4[i] & P4[i - 16];
        end
    endgenerate

    // Carry calculation
    assign C[0] = Cin;
    generate
        for (i = 1; i < 32; i = i + 1) begin
            assign C[i] = G5[i - 1] | (P5[i - 1] & Cin);
        end
    endgenerate

    // Sum and final carry-out
    assign Sum = P ^ C;
    assign Cout = G5[31] | (P5[31] & Cin);

endmodule

module mult_by_181 (
    input  [15:0] in,
    output [31:0] result
);
    wire [31:0] s1 = in << 7; // 128×in
    wire [31:0] s2 = in << 5; // 32×in
    wire [31:0] s3 = in << 4; // 16×in
    wire [31:0] s4 = in << 2; // 4×in
    wire [31:0] s5 = in;      // 1×in

    wire [31:0] temp1, temp2, temp3, temp4;
    wire c1, c2, c3, c4;

    // Sum: 128 + 32 + 16 + 4 + 1 = 181
    sklansky_adder_32bit ksa1 (.A(s1), .B(s2), .Cin(1'b0), .Sum(temp1), .Cout(c1));
    sklansky_adder_32bit ksa2 (.A(temp1), .B(s3), .Cin(1'b0), .Sum(temp2), .Cout(c2));
    sklansky_adder_32bit ksa3 (.A(temp2), .B(s4), .Cin(1'b0), .Sum(temp3), .Cout(c3));
    sklansky_adder_32bit ksa4 (.A(temp3), .B(s5), .Cin(1'b0), .Sum(result), .Cout(c4));
endmodule