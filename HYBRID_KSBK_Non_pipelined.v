`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/04/2025 04:37:13 PM
// Design Name: 
// Module Name: dwt_haar_non_pipelined_top
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


module dwt_haar_non_pipelined_top #(
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

    wire [$clog2(N/2):0] pair_idx;
    wire load_en, write_en;

    dwt_haar_non_pipelined_dp #(.N(N)) dp_inst (
        .clk(clk),
        .rst(rst),
        .load_en(load_en),
        .write_en(write_en),
        .pair_idx(pair_idx),
        .array_in(array_in),
        .cA_out(cA_out),
        .cD_out(cD_out)
    );

    dwt_haar_non_pipelined_ctrl #(.N(N)) ctrl_inst (
        .clk(clk),
        .rst(rst),
        .start(start),
        .pair_idx(pair_idx),
        .load_en(load_en),
        .write_en(write_en),
        .done(done)
    );
endmodule

module dwt_haar_non_pipelined_dp #(
    parameter N = 8
)(
    input  wire clk,
    input  wire rst,
    input  wire load_en,
    input  wire write_en,
    input  wire [$clog2(N/2)-1:0] pair_idx,
    input  wire [N*16-1:0] array_in,
    output reg  [16*(N/2)-1:0] cA_out,
    output reg  [16*(N/2)-1:0] cD_out
);

    wire [15:0] x0, x1;
    reg  [15:0] x0_reg, x1_reg;
    wire [15:0] cA_wire, cD_wire;
    reg  [15:0] cA_reg, cD_reg;

    // Extract current pair
    assign x0 = array_in[2*pair_idx*16 +: 16];
    assign x1 = array_in[(2*pair_idx + 1)*16 +: 16];

    // Hybrid DWT core
    haar_dwt_pair_core_hybrid core (
        .x0(x0_reg),
        .x1(x1_reg),
        .cA(cA_wire),
        .cD(cD_wire)
    );

    // Input registers
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x0_reg <= 16'd0;
            x1_reg <= 16'd0;
        end else if (load_en) begin
            x0_reg <= x0;
            x1_reg <= x1;
        end
    end

    // Output registers
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cA_reg <= 16'd0;
            cD_reg <= 16'd0;
        end else begin
            cA_reg <= cA_wire;
            cD_reg <= cD_wire;
        end
    end

    // Output storage
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cA_out <= {((N/2)*16){1'b0}};
            cD_out <= {((N/2)*16){1'b0}};
        end else if (write_en) begin
            cA_out[pair_idx*16 +: 16] <= cA_reg;
            cD_out[pair_idx*16 +: 16] <= cD_reg;
        end
    end
endmodule

module haar_dwt_pair_core_hybrid (
    input  [15:0] x0,
    input  [15:0] x1,
    output [15:0] cA,
    output [15:0] cD
);
    wire [31:0] x0_mul, x1_mul;
    wire [31:0] sum, diff;
    wire [31:0] x1_neg, x1_twos;
    wire dummy;

    mult_by_181_hybrid m0 (.in(x0), .result(x0_mul));
    mult_by_181_hybrid m1 (.in(x1), .result(x1_mul));

    hybrid_ksbk_adder add (
        .A(x0_mul), 
        .B(x1_mul), 
        .Cin(1'b0), 
        .Sum(sum), 
        .Cout(dummy)
    );

    assign x1_neg = ~x1_mul;
    hybrid_ksbk_adder twos (
        .A(x1_neg), 
        .B(32'd1), 
        .Cin(1'b0), 
        .Sum(x1_twos), 
        .Cout(dummy)
    );
    
    hybrid_ksbk_adder sub (
        .A(x0_mul), 
        .B(x1_twos), 
        .Cin(1'b0), 
        .Sum(diff), 
        .Cout(dummy)
    );

    assign cA = sum[23:8];  // Q16.16 to Q8.8
    assign cD = diff[23:8];
endmodule

module mult_by_181_hybrid (
    input  [15:0] in,
    output [31:0] result
);
    wire [31:0] s1 = in << 7;  // 128x
    wire [31:0] s2 = in << 5;  // 32x
    wire [31:0] s3 = in << 4;  // 16x
    wire [31:0] s4 = in << 2;  // 4x
    wire [31:0] s5 = in;       // 1x

    wire [31:0] temp1, temp2, temp3, temp4;
    wire c1, c2, c3, c4;

    hybrid_ksbk_adder hka1 (.A(s1), .B(s2), .Cin(1'b0), .Sum(temp1), .Cout(c1));
    hybrid_ksbk_adder hka2 (.A(temp1), .B(s3), .Cin(1'b0), .Sum(temp2), .Cout(c2));
    hybrid_ksbk_adder hka3 (.A(temp2), .B(s4), .Cin(1'b0), .Sum(temp3), .Cout(c3));
    hybrid_ksbk_adder hka4 (.A(temp3), .B(s5), .Cin(1'b0), .Sum(result), .Cout(c4));
endmodule

// Controller remains identical to original
module dwt_haar_non_pipelined_ctrl #(
    parameter N = 8
)(
    input  wire clk,
    input  wire rst,
    input  wire start,
    output reg  [$clog2(N/2)-1:0] pair_idx,
    output reg  load_en,
    output reg  write_en,
    output reg  done
);
    // FSM states
    parameter IDLE = 0;
    parameter LOAD = 1;
    parameter PROCESS = 2;
    parameter STORE = 3;
    parameter FINISH = 4;
    
    reg [2:0] state, next_state;
    
    // State register
    always @(posedge clk or posedge rst) begin
        if (rst) state <= IDLE;
        else state <= next_state;
    end
    
    // Next state logic
    always @(*) begin
        case (state)
            IDLE: next_state = start ? LOAD : IDLE;
            LOAD: next_state = PROCESS;
            PROCESS: next_state = STORE;
            STORE: next_state = (pair_idx == N/2-1) ? FINISH : LOAD;
            FINISH: next_state = IDLE;
            default: next_state = IDLE;
        endcase
    end
    
    // Counter logic
    always @(posedge clk or posedge rst) begin
        if (rst) pair_idx <= 0;
        else if (state == STORE) pair_idx <= pair_idx + 1;
        else if (state == IDLE) pair_idx <= 0;
    end
    
    // Output logic
    always @(*) begin
        load_en = (state == LOAD);
        write_en = (state == STORE);
        done = (state == FINISH);
    end
endmodule

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
