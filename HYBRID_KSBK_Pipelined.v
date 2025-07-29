`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/04/2025 03:08:58 PM
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


// ==============================================
// Hybrid Kogge-Stone/Brent-Kung Adder (32-bit)
// ==============================================
module hybrid_ksbk_adder (
    input  [31:0] A, B,
    input         Cin,
    output [31:0] Sum,
    output        Cout
);
    wire [31:0] G, P;
    wire [15:0] C_ksa, C_bk;
    wire        C16;

    // Generate and Propagate signals
    assign G = A & B;
    assign P = A ^ B;

    // ---------- Kogge-Stone Section (Lower 16 bits) ----------
    assign C_ksa[0] = Cin;
    genvar i;
    generate
        for (i = 1; i < 16; i = i + 1) begin : KSA_STAGE
            assign C_ksa[i] = G[i-1] | (P[i-1] & C_ksa[i-1]);
        end
    endgenerate

    generate
        for (i = 0; i < 16; i = i + 1) begin : SUM_LOW
            assign Sum[i] = P[i] ^ C_ksa[i];
        end
    endgenerate

    assign C16 = C_ksa[15];

    // ---------- Brent-Kung Section (Upper 16 bits) ----------
    wire [15:0] G_bk, P_bk;
    assign G_bk = G[31:16];
    assign P_bk = P[31:16];

    wire [15:1] G1, P1;
    wire [15:3] G2, P2;

    generate
        for (i = 1; i < 16; i = i + 1) begin : BK_LVL1
            assign G1[i] = G_bk[i] | (P_bk[i] & G_bk[i-1]);
            assign P1[i] = P_bk[i] & P_bk[i-1];
        end
    endgenerate

    generate
        for (i = 3; i < 16; i = i + 2) begin : BK_LVL2
            assign G2[i] = G1[i] | (P1[i] & G1[i-2]);
            assign P2[i] = P1[i] & P1[i-2];
        end
    endgenerate

    assign C_bk[0] = C16;
    assign C_bk[1] = G_bk[0] | (P_bk[0] & C_bk[0]);
    assign C_bk[2] = G1[1] | (P1[1] & C_bk[0]);
    assign C_bk[3] = G_bk[2] | (P_bk[2] & C_bk[2]);
    assign C_bk[4] = G2[3] | (P2[3] & C_bk[0]);
    assign C_bk[5] = G_bk[4] | (P_bk[4] & C_bk[4]);
    assign C_bk[6] = G1[5] | (P1[5] & C_bk[4]);
    assign C_bk[7] = G_bk[6] | (P_bk[6] & C_bk[6]);
    assign C_bk[8] = G2[7] | (P2[7] & C_bk[4]);
    assign C_bk[9] = G_bk[8] | (P_bk[8] & C_bk[8]);
    assign C_bk[10] = G1[9] | (P1[9] & C_bk[8]);
    assign C_bk[11] = G_bk[10] | (P_bk[10] & C_bk[10]);
    assign C_bk[12] = G2[11] | (P2[11] & C_bk[8]);
    assign C_bk[13] = G_bk[12] | (P_bk[12] & C_bk[12]);
    assign C_bk[14] = G1[13] | (P1[13] & C_bk[12]);
    assign C_bk[15] = G_bk[14] | (P_bk[14] & C_bk[14]);

    generate
        for (i = 0; i < 16; i = i + 1) begin : SUM_HIGH
            assign Sum[16 + i] = P_bk[i] ^ C_bk[i];
        end
    endgenerate

    assign Cout = G_bk[15] | (P_bk[15] & C_bk[15]);
endmodule

// ==============================================
// Multiplier Module (using hybrid adder)
// ==============================================
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

    hybrid_ksbk_adder hka1 (.A(s1), .B(s2), .Cin(1'b0), .Sum(temp1), .Cout(c1));
    hybrid_ksbk_adder hka2 (.A(temp1), .B(s3), .Cin(1'b0), .Sum(temp2), .Cout(c2));
    hybrid_ksbk_adder hka3 (.A(temp2), .B(s4), .Cin(1'b0), .Sum(temp3), .Cout(c3));
    hybrid_ksbk_adder hka4 (.A(temp3), .B(s5), .Cin(1'b0), .Sum(result), .Cout(c4));
endmodule

// ==============================================
// DWT Pipeline Stage 0: Load Input
// ==============================================
module dwt_pipeline_stage0_load #(
    parameter N = 8
)(
    input  wire                     clk,
    input  wire                     en,
    input  wire [$clog2(N/2)-1:0]   pair_idx,
    input  wire [N*16-1:0]          array_in,
    output reg  [15:0]              x0_out,
    output reg  [15:0]              x1_out
);
    wire [15:0] x0 = array_in[2*pair_idx*16 +: 16];
    wire [15:0] x1 = array_in[(2*pair_idx + 1)*16 +: 16];

    always @(posedge clk) begin
        if (en) begin
            x0_out <= x0;
            x1_out <= x1;
        end
    end
endmodule

// ==============================================
// Pipeline Register Stage 0-1
// ==============================================
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

// ==============================================
// DWT Pipeline Stage 1: Multiplication
// ==============================================
module dwt_pipeline_stage1_mult (
    input  wire         clk,
    input  wire         valid_in,
    input  wire [15:0]  x0_in,
    input  wire [15:0]  x1_in,
    output reg  [31:0]  x0_mul_out,
    output reg  [31:0]  x1_mul_out
);
    wire [31:0] x0_mul, x1_mul;

    mult_by_181 mul_x0 (.in(x0_in), .result(x0_mul));
    mult_by_181 mul_x1 (.in(x1_in), .result(x1_mul));

    always @(posedge clk) begin
        if (valid_in) begin
            x0_mul_out <= x0_mul;
            x1_mul_out <= x1_mul;
        end
    end
endmodule

// ==============================================
// Pipeline Register Stage 1-2
// ==============================================
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

// ==============================================
// DWT Pipeline Stage 2: Core DWT Computation
// ==============================================
module dwt_pipeline_stage2_core (
    input  wire        clk,
    input  wire        valid_in,
    input  wire [31:0] x0_mul,
    input  wire [31:0] x1_mul,
    output reg  [15:0] cA,
    output reg  [15:0] cD
);
    wire [31:0] sum, diff;
    wire [31:0] x1_neg = ~x1_mul;
    wire [31:0] x1_twos;
    wire        dummy;

    hybrid_ksbk_adder adder_sum (.A(x0_mul), .B(x1_mul), .Cin(1'b0), .Sum(sum), .Cout(dummy));
    hybrid_ksbk_adder adder_twos (.A(x1_neg), .B(32'd1), .Cin(1'b0), .Sum(x1_twos), .Cout(dummy));
    hybrid_ksbk_adder adder_diff (.A(x0_mul), .B(x1_twos), .Cin(1'b0), .Sum(diff), .Cout(dummy));

    always @(posedge clk) begin
        if (valid_in) begin
            cA <= sum[23:8];  // Normalize from Q16.16 to Q8.8
            cD <= diff[23:8];
        end
    end
endmodule

// ==============================================
// Pipeline Register Stage 2-3
// ==============================================
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

// ==============================================
// DWT Pipeline Stage 3: Store Results
// ==============================================
module dwt_pipeline_stage3_store #(
    parameter N = 8
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   valid_in,
    input  wire                   write_en,
    input  wire [$clog2(N/2)-1:0] pair_idx,
    input  wire [15:0]            cA_in,
    input  wire [15:0]            cD_in,
    output reg  [16*(N/2)-1:0]    cA_out,
    output reg  [16*(N/2)-1:0]    cD_out
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

// ==============================================
// Controller Module
// ==============================================
module dwt_haar_pipelined_ctrl #(
    parameter N = 8,
    parameter LATENCY = 3
)(
    input wire clk,
    input wire rst,
    input wire start,
    output reg [31:0] pair_idx,
    output reg load_en,
    output reg write_en,
    output reg done
);
    localparam IDLE  = 3'b000;
    localparam LOAD  = 3'b001;
    localparam FLUSH = 3'b010;
    localparam WRITE = 3'b011;
    localparam DONE  = 3'b100;

    reg [2:0] state, next_state;
    reg [31:0] load_count;
    reg [31:0] write_count;

    always @(posedge clk or posedge rst) begin
        if (rst) state <= IDLE;
        else state <= next_state;
    end

    always @(*) begin
        case (state)
            IDLE:  next_state = start ? LOAD : IDLE;
            LOAD:  next_state = (load_count == (N/2)) ? FLUSH : LOAD;
            FLUSH: next_state = WRITE;
            WRITE: next_state = (write_count == (N/2)) ? DONE : WRITE;
            DONE:  next_state = !start ? IDLE : DONE;
            default: next_state = IDLE;
        endcase
    end

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
                FLUSH: begin
                    load_en  <= 1;
                    write_en <= 0;
                    pair_idx <= pair_idx;
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

// ==============================================
// Top Level Module
// ==============================================
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
    wire [$clog2(N/2)-1:0] pair_idx_pipe;
    wire load_en, valid_stage1, valid_stage2, valid_stage3, write_en;

    // Stage 0
    wire [15:0] x0_s0, x1_s0;
    dwt_pipeline_stage0_load #(.N(N)) stage0 (
        .clk(clk),
        .en(load_en),
        .pair_idx(pair_idx_pipe),
        .array_in(array_in),
        .x0_out(x0_s0),
        .x1_out(x1_s0)
    );

    // Stage 0?1
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

    // Stage 1
    wire [31:0] x0_mul_s1, x1_mul_s1;
    dwt_pipeline_stage1_mult stage1 (
        .clk(clk),
        .valid_in(valid_stage1),
        .x0_in(x0_s1),
        .x1_in(x1_s1),
        .x0_mul_out(x0_mul_s1),
        .x1_mul_out(x1_mul_s1)
    );

    // Stage 1?2
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

    // Stage 2
    wire [15:0] cA_s2, cD_s2;
    dwt_pipeline_stage2_core stage2 (
        .clk(clk),
        .valid_in(valid_stage2),
        .x0_mul(x0_mul_s2),
        .x1_mul(x1_mul_s2),
        .cA(cA_s2),
        .cD(cD_s2)
    );

    // Stage 2?3
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

    // Stage 3
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

    // Controller
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