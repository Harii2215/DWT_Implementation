`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/03/2025 10:47:42 AM
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
    input  wire [$clog2(N/2)-1:0] write_ptr,
    input  wire [N*16-1:0] array_in,

    output reg  [16*(N/2)-1:0] cA_out,
    output reg  [16*(N/2)-1:0] cD_out
);

    wire [15:0] x0, x1;
    reg  [15:0] x0_reg, x1_reg;
    wire [15:0] cA_wire, cD_wire;
    reg  [15:0] cA_reg, cD_reg;

    // Extract current pair from input
    assign x0 = array_in[2*pair_idx*16 +: 16];
    assign x1 = array_in[(2*pair_idx + 1)*16 +: 16];

    // DWT pair processor
    haar_dwt_pair_core core (
        .x0(x0_reg),
        .x1(x1_reg),
        .cA(cA_wire),
        .cD(cD_wire)
    );

    // Latch inputs
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x0_reg <= 16'd0;
            x1_reg <= 16'd0;
        end else if (load_en) begin
            x0_reg <= x0;
            x1_reg <= x1;
        end
    end

    // Latch DWT core outputs
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cA_reg <= 16'd0;
            cD_reg <= 16'd0;
        end else begin
            cA_reg <= cA_wire;
            cD_reg <= cD_wire;
        end
    end

    // Store to final output buses
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cA_out <= {((N/2)*16){1'b0}};
            cD_out <= {((N/2)*16){1'b0}};
        end else if (write_en) begin
            cA_out[write_ptr*16 +: 16] <= cA_reg;
            cD_out[write_ptr*16 +: 16] <= cD_reg;
        end
    end

endmodule

module dwt_haar_non_pipelined_ctrl #(
    parameter N = 8
)(
    input  wire clk,
    input  wire rst,
    input  wire start,

    output reg  [$clog2(N/2)-1:0] pair_idx,
    output reg  [$clog2(N/2)-1:0] write_ptr,
    output reg  load_en,
    output reg  write_en,
    output reg  done
);

    // State encoding (binary)
    localparam IDLE    = 3'd0;
    localparam LOAD    = 3'd1;
    localparam PROCESS = 3'd2;
    localparam STORE   = 3'd3;
    localparam DONE    = 3'd4;

    reg [2:0] state, next_state;
    reg first_valid;

    // FSM state transition
    always @(*) begin
        case (state)
            IDLE:    next_state = start ? LOAD : IDLE;
            LOAD:    next_state = PROCESS;
            PROCESS: next_state = STORE;
            STORE:   next_state = (pair_idx == (N/2 - 1)) ? DONE : LOAD;
            DONE:    next_state = start ? DONE : IDLE;
            default: next_state = IDLE;
        endcase
    end

    // FSM state update
    always @(posedge clk or posedge rst) begin
        if (rst)
            state <= IDLE;
        else
            state <= next_state;
    end

    // Control logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pair_idx    <= 0;
            write_ptr   <= 0;
            load_en     <= 0;
            write_en    <= 0;
            done        <= 0;
            first_valid <= 0;
        end else begin
            load_en     <= 0;
            write_en    <= 0;
            done        <= 0;

            case (state)
                IDLE: begin
                    pair_idx    <= 0;
                    write_ptr   <= 0;
                    first_valid <= 0;
                end
                LOAD: begin
                    load_en <= 1;
                end
                PROCESS: begin
                    // Wait state
                end
                STORE: begin
                    if (first_valid) begin
                        write_en <= 1;
                        write_ptr <= write_ptr + 1;
                    end else begin
                        first_valid <= 1; // Skip STORE on first cycle
                    end

                    if (pair_idx != (N/2 - 1))
                        pair_idx <= pair_idx + 1;
                end
                DONE: begin
                    done <= 1;
                end
            endcase
        end
    end

endmodule


   module haar_dwt_pair_core (
    input  [15:0] x0,
    input  [15:0] x1,
    output [15:0] cA,
    output [15:0] cD
);
    wire [31:0] x0_mul, x1_mul;
    wire [31:0] sum, diff;
    wire [31:0] x1_neg, x1_twos;
    wire dummy;

    mult_by_181 m0 (.in(x0), .result(x0_mul));
    mult_by_181 m1 (.in(x1), .result(x1_mul));

    sklansky_adder_32bit add (.A(x0_mul), .B(x1_mul), .Cin(1'b0), .Sum(sum), .Cout(dummy));
    sklansky_adder_32bit twos (.A(x1_neg), .B(32'd1), .Cin(1'b0), .Sum(x1_twos), .Cout(dummy));
    sklansky_adder_32bit sub (.A(x0_mul), .B(x1_twos), .Cin(1'b0), .Sum(diff), .Cout(dummy));


    assign cA = sum[23:8];
    assign cD = diff[23:8];
endmodule
module mult_by_181 (
    input  [15:0] in,
    output [31:0] result
);
    wire [31:0] s1 = in << 7;
    wire [31:0] s2 = in << 5;
    wire [31:0] s3 = in << 4;
    wire [31:0] s4 = in << 2;
    wire [31:0] s5 = in;

    wire [31:0] temp1, temp2, temp3, temp4;
    wire c1, c2, c3, c4;

    sklansky_adder_32bit ksa1 (.A(s1), .B(s2), .Cin(1'b0), .Sum(temp1), .Cout(c1));
    sklansky_adder_32bit ksa2 (.A(temp1), .B(s3), .Cin(1'b0), .Sum(temp2), .Cout(c2));
    sklansky_adder_32bit ksa3 (.A(temp2), .B(s4), .Cin(1'b0), .Sum(temp3), .Cout(c3));
    sklansky_adder_32bit ksa4 (.A(temp3), .B(s5), .Cin(1'b0), .Sum(result), .Cout(c4));

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
            assign C[i] = G5[i - 1] | (P5[i - 1] & C[i - 1]);
        end
    endgenerate


    // Sum and final carry-out
    assign Sum = P ^ C;
    assign Cout = G5[31] | (P5[31] & Cin);

endmodule
