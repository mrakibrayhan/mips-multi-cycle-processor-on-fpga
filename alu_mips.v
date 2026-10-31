//====================================================================
// alu_mips.v   (MIPS multi-cycle CPU, step 5)  -  [HH] ALU (section 5.2.4 / fig. 7.27) extended for the MIPS I subset
//
//   ALUControl (4 bits; the [HH] 3-bit codes are kept with a leading 0):
//     0000 AND   0001 OR    0010 ADD   0110 SUB   0111 SLT          ([HH])
//     0011 XOR   0100 NOR   1000 SLTU                               (FEATURES bit 0, ALU group)
//     1001 SLL   1010 SRL   1011 SRA                                (FEATURES bit 1, SHIFT group)
//   One adder: ADD = a + b; SUB / SLT / SLTU = a + ~b + 1.
//   SLT  = signed a < b  = (a[31] != b[31]) ? a[31] : diff[31]   (correct also when a - b overflows)
//   SLTU = unsigned a < b = no carry out of a + ~b + 1.
//   Shifts: value = b (rt, SrcB), amount = 'shamt' (the datapath gives Instr[10:6] or SrcA[4:0] = rs[4:0]).
//   zero = (y == 0) ([HH] Zero flag, used by beq / bne / blez / bgtz).
//   Groups not built (ALUX = 0 / SHIFT = 0) are removed (ablation builds); their codes then give 0.
//====================================================================
module alu_mips #(
    parameter ALUX  = 1,
    parameter SHIFT = 1
) (
    input  [31:0] a,
    input  [31:0] b,
    input  [3:0]  ctl,
    input  [4:0]  shamt,
    output reg [31:0] y,
    output        zero
);
    wire        sub  = (ctl == 4'b0110) | (ctl == 4'b0111) | (ctl == 4'b1000);
    wire [32:0] sum  = {1'b0, a} + {1'b0, sub ? ~b : b} + {32'd0, sub};
    wire        slt  = (a[31] != b[31]) ? a[31] : sum[31];
    wire        sltu = ~sum[32];

    wire [31:0] shl, shr;
    generate if (SHIFT) begin : g_sh
        wire signed [32:0] sx = {ctl == 4'b1011 & b[31], b};             // SRA: sign bit, SRL: 0
        wire        [32:0] sr = sx >>> shamt;
        assign shl = b << shamt;
        assign shr = sr[31:0];
    end else begin : g_nosh
        assign shl = 32'd0;
        assign shr = 32'd0;
    end endgenerate

    always @(*) begin
        case (ctl)
            4'b0000: y = a & b;
            4'b0001: y = a | b;
            4'b0010: y = sum[31:0];
            4'b0110: y = sum[31:0];
            4'b0111: y = {31'd0, slt};
            4'b0011: y = ALUX ? a ^ b : 32'd0;
            4'b0100: y = ALUX ? ~(a | b) : 32'd0;
            4'b1000: y = ALUX ? {31'd0, sltu} : 32'd0;
            4'b1001: y = shl;
            4'b1010: y = shr;
            4'b1011: y = shr;
            default: y = 32'd0;
        endcase
    end
    assign zero = (y == 32'd0);
endmodule
