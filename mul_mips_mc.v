//====================================================================
// mul_mips_mc.v   (MIPS multi-cycle CPU, step 11)  -  mult / multu: {HI, LO} = A x B
//
//   One 33 x 33 signed multiplier (sgn = 1: mult, operands sign-extended; sgn = 0: multu, zero-extended), mapped to the
//   Cyclone IV 18 x 18 DSP blocks (8 x 9-bit elements). The product is written into HI / LO at the end of the MULT state
//   (mips_mc_spec.md section 7): A and B are the register-file M9K output registers, so the path is
//   M9K output -> multiplier -> HI / LO (the ALU is not in the path).
//====================================================================
module mul_mips_mc (
    input  [31:0] a,
    input  [31:0] b,
    input         sgn,
    output [63:0] p
);
    wire signed [32:0] sa = {sgn & a[31], a};
    wire signed [32:0] sb = {sgn & b[31], b};
    wire signed [65:0] pp = sa * sb;
    assign p = pp[63:0];
endmodule
